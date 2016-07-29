#ifndef __HTC_DEBUG_STABILITY_DEBUG_PAGE_USER_TRACE_H__
#define __HTC_DEBUG_STABILITY_DEBUG_PAGE_USER_TRACE_H__

struct page;

struct page_user_trace {
	pid_t pid;
	char comm[8];
	pid_t tgid;
	char tgcomm[8];
	unsigned long entries[UL(CONFIG_HTC_DEBUG_PAGE_ENTRIES_NR)];
};

#ifdef CONFIG_HTC_DEBUG_PAGE_USER_TRACE
#define DECLARE_PAGE_USER_TRACE(name) struct page_user_trace name;
void set_page_user_trace(struct page *page, int numpages, int free);
void migrate_page_copy_user_trace(struct page *newpage, struct page *page);
#else
#define DECLARE_PAGE_USER_TRACE(name)
static inline
void set_page_user_trace(struct page *page, int numpages, int free) { }
static inline
void migrate_page_copy_user_trace(struct page *newpage, struct page *page) { }
#endif 

#endif 
