#include "thran.h"
#include <string.h>
#include <kernel.h>

struct thread_info {
	char name[33];
	size_t stack_size;
	size_t stack_used;
};

#define THREAD_ITEMS 15

struct thread_info thread_info_arr[THREAD_ITEMS] = {};
int thread_info_arr_size = 0;

void update_thread_item(struct thread_analyzer_info *info)
{
	int i;
	int idx = -1;

	if (thread_info_arr_size >= THREAD_ITEMS) {
		printk("thread_info_arr full!\n");
		return;
	}

	for (i = 0; i < thread_info_arr_size; i++) {
		if (strcmp(info->name, thread_info_arr[i].name) == 0) {
			idx = i;
			break;
		}
	}

	if (idx == -1) {
		idx = thread_info_arr_size;
		thread_info_arr_size++;
	}

	strcpy(thread_info_arr[idx].name, info->name);
	if (thread_info_arr[idx].stack_size < info->stack_size) {
		thread_info_arr[idx].stack_size = info->stack_size;
	}
	if (thread_info_arr[idx].stack_used < info->stack_used) {
		thread_info_arr[idx].stack_used = info->stack_used;
	}
}

void list_thread_items(void)
{
	int i;

	for (i = 0; i < thread_info_arr_size; i++) {
		printk("%s: %d, %d\n",
			thread_info_arr[i].name,
			thread_info_arr[i].stack_size,
			thread_info_arr[i].stack_used);
	}
	printk("\n");
}
