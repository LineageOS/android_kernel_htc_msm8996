#include <linux/stacktrace.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <htc_debug/stability/debug_page_user_trace.h>

#define page_to_trace(page, free) \
	(free ? \
		&page->trace_free : \
		&page->trace_alloc)
#define page_to_entries_size(page, free) \
	(free ? \
		ARRAY_SIZE(page->trace_free.entries) : \
		ARRAY_SIZE(page->trace_alloc.entries))

void set_page_user_trace(struct page *page, int numpages, int free)
{
	struct page_user_trace *user_trace;
	struct stack_trace trace;
	unsigned long *entries;
	int nr_entries;

	if (unlikely(!page))
		return;

	user_trace = page_to_trace(page, free);
	entries = user_trace->entries;
	nr_entries = page_to_entries_size(page, free);

	user_trace->pid = current->pid;
	user_trace->tgid = current->tgid;
	memcpy(user_trace->comm, current->comm,
			sizeof(user_trace->comm));
	memcpy(user_trace->tgcomm, current->group_leader->comm,
			sizeof(user_trace->comm));

	if (unlikely(!nr_entries))
		return;

	memset(entries, 0, nr_entries * sizeof(*entries));
	trace.max_entries = nr_entries;
	trace.entries = entries;
	trace.nr_entries = 0;
	trace.skip = 5;
	save_stack_trace(&trace);

	for (; page++, numpages > 1; numpages--)
		memcpy(page_to_trace(page, free), user_trace,
				sizeof(*user_trace));
}

void migrate_page_copy_user_trace(struct page *newpage, struct page *page)
{
	memcpy(&newpage->trace_alloc, &page->trace_alloc,
			sizeof(page->trace_alloc));
	memcpy(&newpage->trace_free, &page->trace_free,
			sizeof(page->trace_free));
}
