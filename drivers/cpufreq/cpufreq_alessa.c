/*
* drivers/cpufreq/cpufreq_alessa.c
*
* Copyright (C) 2011 Samsung Electronics co. ltd
* ByungChang Cha <bc.cha@samsung.com>
*
* Based on ondemand governor
* Copyright (C) 2001 Russell King
* 	    (C) 2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
* 		     Jun Nakajima <jun.nakajima@intel.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.
*
* Autor: Carlos "Klozz" Jesús <xxx.reptar.rawrr.xxx@gmail.com>
* 	(C) 2014 Klozz Jesús (TeamMEX@XDA-Developers)
*
* This peace of art are dedicated to Stephanny Marlene...
*
* v1.0
* v2.7.0.7 too much updates like:
* code cleanup
* latency-sensitive bursty workload.
* add more checks and fix cpu inits on gov starts.
* Adjusted sampling time!
* Bug fixes
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define MIN_SAMPLING_RATE	10000

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

static void do_alessa_timer(struct work_struct *work);

struct cpufreq_alessa_cpuinfo {
	u64 prev_cpu_wall;
	u64 prev_cpu_idle;
	struct cpufreq_frequency_table *freq_table;
	struct delayed_work work;
	struct cpufreq_policy *cur_policy;
	bool governor_enabled;
	unsigned int cpu;
	unsigned int prev_load;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
/*
 * mutex that serializes governor limit change with
 * do_alessa_timer invocation. We do not want do_alessa_timer to run
 * when user is changing the governor or limits.
 */
static DEFINE_PER_CPU(struct cpufreq_alessa_cpuinfo, od_alessa_cpuinfo);

static unsigned int alessa_enable;	/* number of CPUs using this policy */
/*
 * alessa_mutex protects alessa_enable in governor start/stop.
 */
static DEFINE_MUTEX(alessa_mutex);

static struct workqueue_struct *alessa_wq;

/* alessa tuners */
static struct alessa_tuners {
	unsigned int sampling_rate;
} alessa_tuners_ins = {
	.sampling_rate = 60000,
};

/************************** sysfs interface ************************/

/* cpufreq_alessa Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", alessa_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);

/* sampling_rate */
static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret = 0;
	int mpd = strcmp(current->comm, "mpdecision");

	if (mpd == 0)
		return ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input,10000);

	if (input == alessa_tuners_ins.sampling_rate)
		return count;

	alessa_tuners_ins.sampling_rate = input;

	return count;
}

define_one_global_rw(sampling_rate);

static struct attribute *alessa_attributes[] = {
	&sampling_rate.attr,
	NULL
};

static struct attribute_group alessa_attr_group = {
	.attrs = alessa_attributes,
	.name = "Alessa",
};

/************************** sysfs end ************************/

static unsigned int adjust_cpufreq_frequency_target(struct cpufreq_policy *policy,
					struct cpufreq_frequency_table *table,
					unsigned int tmp_freq)
{
	unsigned int i = 0, l_freq = 0, h_freq = 0, target_freq = 0;

	if (tmp_freq < policy->min)
		tmp_freq = policy->min;
	if (tmp_freq > policy->max)
		tmp_freq = policy->max;

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		unsigned int freq = table[i].frequency;
		if (freq == CPUFREQ_ENTRY_INVALID) {
			continue;
		}
		if (freq < tmp_freq) {
			h_freq = freq;
		}
		if (freq == tmp_freq) {
			target_freq = freq;
			break;
		}
		if (freq > tmp_freq) {
			l_freq = freq;
			break;
		}
	}
	if (!target_freq) {
		if (policy->cur >= h_freq
			 && policy->cur <= l_freq)
			target_freq = policy->cur;
		else
			target_freq = l_freq;
	}

	return target_freq;
}

static void alessa_check_cpu(struct cpufreq_alessa_cpuinfo *this_alessa_cpuinfo)
{
	struct cpufreq_policy *policy;
	unsigned int max_load = 0;
	unsigned int next_freq = 0;
	unsigned int j;
	unsigned int sampling_rate = alessa_tuners_ins.sampling_rate;

	policy = this_alessa_cpuinfo->cur_policy;
	if (!policy)
		return;

	for_each_cpu(j, policy->cpus) {
		struct cpufreq_alessa_cpuinfo *j_alessa_cpuinfo = &per_cpu(od_alessa_cpuinfo, j);
		u64 cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		unsigned int cur_load;

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, 0);

		wall_time = (unsigned int)
			(cur_wall_time - j_alessa_cpuinfo->prev_cpu_wall);
		j_alessa_cpuinfo->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_alessa_cpuinfo->prev_cpu_idle);
		j_alessa_cpuinfo->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		/*
		 * If the CPU had gone completely idle, and a task just woke up
		 * on this CPU now, it would be unfair to calculate 'load' the
		 * usual way for this elapsed time-window, because it will show
		 * near-zero load, irrespective of how CPU intensive that task
		 * actually is. This is undesirable for latency-sensitive bursty
		 * workloads.
		 *
		 * To avoid this, we reuse the 'load' from the previous
		 * time-window and give this task a chance to start with a
		 * reasonably high CPU frequency. (However, we shouldn't over-do
		 * this copy, lest we get stuck at a high load (high frequency)
		 * for too long, even when the current system load has actually
		 * dropped down. So we perform the copy only once, upon the
		 * first wake-up from idle.)
		 *
		 * Detecting this situation is easy: the governor's deferrable
		 * timer would not have fired during CPU-idle periods. Hence
		 * an unusually large 'wall_time' (as compared to the sampling
		 * rate) indicates this scenario.
		 *
		 * prev_load can be zero in two cases and we must recalculate it
		 * for both cases:
		 * - during long idle intervals
		 * - explicitly set to zero
		 */
		if (unlikely(wall_time > (2 * sampling_rate) &&
			     j_alessa_cpuinfo->prev_load)) {
			cur_load = j_alessa_cpuinfo->prev_load;

			/*
			 * Perform a destructive copy, to ensure that we copy
			 * the previous load only once, upon the first wake-up
			 * from idle.
			 */
			j_alessa_cpuinfo->prev_load = 0;
		} else {
			cur_load = 100 * (wall_time - idle_time) / wall_time;
			j_alessa_cpuinfo->prev_load = cur_load;
		}

		if (cur_load > max_load)
			max_load = cur_load;
	}

	cpufreq_notify_utilization(policy, max_load);

	/* CPUs Online Scale Frequency*/
	next_freq = adjust_cpufreq_frequency_target(policy, this_alessa_cpuinfo->freq_table,
												max_load * (policy->max / 100));
	if (next_freq != policy->cur && next_freq > 0)
		__cpufreq_driver_target(policy, next_freq, CPUFREQ_RELATION_L);
}

static void do_alessa_timer(struct work_struct *work)
{
	struct cpufreq_alessa_cpuinfo *this_alessa_cpuinfo =
		container_of(work, struct cpufreq_alessa_cpuinfo, work.work);
	int delay;

	if (unlikely(!cpu_online(this_alessa_cpuinfo->cpu) ||
				!this_alessa_cpuinfo->cur_policy))
		return;

	mutex_lock(&this_alessa_cpuinfo->timer_mutex);

	alessa_check_cpu(this_alessa_cpuinfo);

	delay = usecs_to_jiffies(alessa_tuners_ins.sampling_rate);
	/* We want all CPUs to do sampling nearly on
	 * same jiffy
	 */
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}

	queue_delayed_work_on(this_alessa_cpuinfo->cpu, alessa_wq,
			&this_alessa_cpuinfo->work, delay);
	mutex_unlock(&this_alessa_cpuinfo->timer_mutex);
}

static int cpufreq_governor_alessa(struct cpufreq_policy *policy,
				unsigned int event)
{
	struct cpufreq_alessa_cpuinfo *this_alessa_cpuinfo;
	unsigned int cpu = policy->cpu, j;
	int rc, delay;

	this_alessa_cpuinfo = &per_cpu(od_alessa_cpuinfo, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy))
			return -EINVAL;

		mutex_lock(&alessa_mutex);
		this_alessa_cpuinfo->freq_table = cpufreq_frequency_get_table(cpu);
		if (!this_alessa_cpuinfo->freq_table) {
			mutex_unlock(&alessa_mutex);
			return -EINVAL;
		}

		for_each_cpu(j, policy->cpus) {
			struct cpufreq_alessa_cpuinfo *j_alessa_cpuinfo = &per_cpu(od_alessa_cpuinfo, j);
			unsigned int prev_load;

			j_alessa_cpuinfo->prev_cpu_idle = get_cpu_idle_time(j,
				&j_alessa_cpuinfo->prev_cpu_wall, 0);

			prev_load = (unsigned int)
				(j_alessa_cpuinfo->prev_cpu_wall -
				j_alessa_cpuinfo->prev_cpu_idle);
			j_alessa_cpuinfo->prev_load = 100 * prev_load /
				(unsigned int) j_alessa_cpuinfo->prev_cpu_wall;
		}

		alessa_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (alessa_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&alessa_attr_group);
			if (rc) {
				alessa_enable--;
				mutex_unlock(&alessa_mutex);
				return rc;
			}
		}
		cpu = policy->cpu;
		this_alessa_cpuinfo->cpu = cpu;
		this_alessa_cpuinfo->cur_policy = policy;
		this_alessa_cpuinfo->governor_enabled = true;
		mutex_unlock(&alessa_mutex);

		mutex_init(&this_alessa_cpuinfo->timer_mutex);

		delay = usecs_to_jiffies(alessa_tuners_ins.sampling_rate);
		/* We want all CPUs to do sampling nearly on same jiffy */
		if (num_online_cpus() > 1) {
			delay -= jiffies % delay;
		}

		INIT_DEFERRABLE_WORK(&this_alessa_cpuinfo->work, do_alessa_timer);
		queue_delayed_work_on(cpu,
			alessa_wq, &this_alessa_cpuinfo->work, delay);

		break;
	case CPUFREQ_GOV_STOP:
		cancel_delayed_work_sync(&this_alessa_cpuinfo->work);

		mutex_lock(&alessa_mutex);
		mutex_destroy(&this_alessa_cpuinfo->timer_mutex);

		this_alessa_cpuinfo->governor_enabled = false;

		this_alessa_cpuinfo->cur_policy = NULL;

		alessa_enable--;
		if (!alessa_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &alessa_attr_group);
		}
		mutex_unlock(&alessa_mutex);

		break;
	case CPUFREQ_GOV_LIMITS:
		if (!this_alessa_cpuinfo->cur_policy
			 || !policy) {
			pr_debug("Unable to limit cpu freq due to cur_policy == NULL\n");
			return -EPERM;
		}
		mutex_lock(&this_alessa_cpuinfo->timer_mutex);
		__cpufreq_driver_target(this_alessa_cpuinfo->cur_policy,
				policy->cur, CPUFREQ_RELATION_L);
		mutex_unlock(&this_alessa_cpuinfo->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ALESSA
static
#endif
struct cpufreq_governor cpufreq_gov_alessa = {
	.name                   = "Alessa",
	.governor               = cpufreq_governor_alessa,
	.owner                  = THIS_MODULE,
};

static int __init cpufreq_gov_alessa_init(void)
{
	alessa_wq = alloc_workqueue("alessa_wq", WQ_HIGHPRI, 0);
	if (!alessa_wq) {
		printk(KERN_ERR "Failed to create alessa_wq workqueue\n");
		return -EFAULT;
	}

	return cpufreq_register_governor(&cpufreq_gov_alessa);
}

static void __exit cpufreq_gov_alessa_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_alessa);
}

MODULE_AUTHOR("Carlos "klozz" Jesus klozz707@gmail.com");
MODULE_DESCRIPTION("'cpufreq_alessa' - A dynamic cpufreq governor for msm devices");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ALESSA
fs_initcall(cpufreq_gov_alessa_init);
#else
module_init(cpufreq_gov_alessa_init);
#endif
module_exit(cpufreq_gov_alessa_exit);

