#ifndef _LINUX_SORT_H
#define _LINUX_SORT_H

#include <linux/types.h>

void sort_r(void *base, size_t num, size_t size,
	    int (*cmp)(const void *, const void *, const void *),
	    void (*swap)(void *, void *, int),
	    const void *priv);

void sort(void *base, size_t num, size_t size,
	  cmp_func_t cmp_func,
	  swap_func_t swap_func);

#endif
