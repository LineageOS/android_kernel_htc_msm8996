#ifndef _LINUX_INIT_H
#define _LINUX_INIT_H

#include <linux/compiler.h>
#include <linux/types.h>


#define __init		__section(.init.text) __cold notrace
#define __initdata	__section(.init.data)
#define __initconst	__constsection(.init.rodata)
#define __exitdata	__section(.exit.data)
#define __exit_call	__used __section(.exitcall.exit)

#ifdef CONFIG_BROKEN_RODATA
#define __constsection(x)
#else
#define __constsection(x) __section(x)
#endif

#define __ref            __section(.ref.text) noinline
#define __refdata        __section(.ref.data)
#define __refconst       __constsection(.ref.rodata)

#define __init_refok     __ref
#define __initdata_refok __refdata
#define __exit_refok     __ref


#ifdef MODULE
#define __exitused
#else
#define __exitused  __used
#endif

#define __exit          __section(.exit.text) __exitused __cold notrace

#define __cpuinit
#define __cpuinitdata
#define __cpuinitconst
#define __cpuexit
#define __cpuexitdata
#define __cpuexitconst

#define __meminit        __section(.meminit.text) __cold notrace
#define __meminitdata    __section(.meminit.data)
#define __meminitconst   __constsection(.meminit.rodata)
#define __memexit        __section(.memexit.text) __exitused __cold notrace
#define __memexitdata    __section(.memexit.data)
#define __memexitconst   __constsection(.memexit.rodata)

#define __HEAD		.section	".head.text","ax"
#define __INIT		.section	".init.text","ax"
#define __FINIT		.previous

#define __INITDATA	.section	".init.data","aw",%progbits
#define __INITRODATA	.section	".init.rodata","a",%progbits
#define __FINITDATA	.previous

#define __CPUINIT

#define __MEMINIT        .section	".meminit.text", "ax"
#define __MEMINITDATA    .section	".meminit.data", "aw"
#define __MEMINITRODATA  .section	".meminit.rodata", "a"

#define __REF            .section       ".ref.text", "ax"
#define __REFDATA        .section       ".ref.data", "aw"
#define __REFCONST       .section       ".ref.rodata", "a"

#ifndef __ASSEMBLY__
typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

extern initcall_t __con_initcall_start[], __con_initcall_end[];
extern initcall_t __security_initcall_start[], __security_initcall_end[];

typedef void (*ctor_fn_t)(void);

extern int do_one_initcall(initcall_t fn);
extern char __initdata boot_command_line[];
extern char *saved_command_line;
extern char *hashed_command_line;
extern unsigned int reset_devices;

void setup_arch(char **);
void prepare_namespace(void);
void __init load_default_modules(void);
int __init init_rootfs(void);

extern void (*late_time_init)(void);

extern bool initcall_debug;

#endif
  
#ifndef MODULE

#ifndef __ASSEMBLY__

#ifdef CONFIG_LTO
#define LTO_REFERENCE_INITCALL(x) \
	; 			\
	static __used __exit void *reference_##x(void)	\
	{						\
		return &x;				\
	}
#else
#define LTO_REFERENCE_INITCALL(x)
#endif


#define __define_initcall(fn, id) \
	static initcall_t __initcall_##fn##id __used \
	__attribute__((__section__(".initcall" #id ".init"))) = fn; \
	LTO_REFERENCE_INITCALL(__initcall_##fn##id)

#define early_initcall(fn)		__define_initcall(fn, early)

#define pure_initcall(fn)		__define_initcall(fn, 0)

#define core_initcall(fn)		__define_initcall(fn, 1)
#define core_initcall_sync(fn)		__define_initcall(fn, 1s)
#define postcore_initcall(fn)		__define_initcall(fn, 2)
#define postcore_initcall_sync(fn)	__define_initcall(fn, 2s)
#define arch_initcall(fn)		__define_initcall(fn, 3)
#define arch_initcall_sync(fn)		__define_initcall(fn, 3s)
#define subsys_initcall(fn)		__define_initcall(fn, 4)
#define subsys_initcall_sync(fn)	__define_initcall(fn, 4s)
#define fs_initcall(fn)			__define_initcall(fn, 5)
#define fs_initcall_sync(fn)		__define_initcall(fn, 5s)
#define rootfs_initcall(fn)		__define_initcall(fn, rootfs)
#define device_initcall(fn)		__define_initcall(fn, 6)
#define device_initcall_sync(fn)	__define_initcall(fn, 6s)
#define late_initcall(fn)		__define_initcall(fn, 7)
#define late_initcall_sync(fn)		__define_initcall(fn, 7s)

#define __initcall(fn) device_initcall(fn)

#define __exitcall(fn) \
	static exitcall_t __exitcall_##fn __exit_call = fn

#define console_initcall(fn) \
	static initcall_t __initcall_##fn \
	__used __section(.con_initcall.init) = fn

#define security_initcall(fn) \
	static initcall_t __initcall_##fn \
	__used __section(.security_initcall.init) = fn

struct obs_kernel_param {
	const char *str;
	int (*setup_func)(char *);
	int early;
};

#define __setup_param(str, unique_id, fn, early)			\
	static const char __setup_str_##unique_id[] __initconst	\
		__aligned(1) = str; \
	static struct obs_kernel_param __setup_##unique_id	\
		__used __section(.init.setup)			\
		__attribute__((aligned((sizeof(long)))))	\
		= { __setup_str_##unique_id, fn, early }

#define __setup(str, fn)					\
	__setup_param(str, fn, fn, 0)

#define early_param(str, fn)					\
	__setup_param(str, fn, fn, 1)

void __init parse_early_param(void);
void __init parse_early_options(char *cmdline);
#endif 

#define module_init(x)	__initcall(x);

#define module_exit(x)	__exitcall(x);

#else 

#define early_initcall(fn)		module_init(fn)
#define core_initcall(fn)		module_init(fn)
#define core_initcall_sync(fn)		module_init(fn)
#define postcore_initcall(fn)		module_init(fn)
#define postcore_initcall_sync(fn)	module_init(fn)
#define arch_initcall(fn)		module_init(fn)
#define subsys_initcall(fn)		module_init(fn)
#define subsys_initcall_sync(fn)	module_init(fn)
#define fs_initcall(fn)			module_init(fn)
#define fs_initcall_sync(fn)		module_init(fn)
#define rootfs_initcall(fn)		module_init(fn)
#define device_initcall(fn)		module_init(fn)
#define device_initcall_sync(fn)	module_init(fn)
#define late_initcall(fn)		module_init(fn)
#define late_initcall_sync(fn)		module_init(fn)

#define console_initcall(fn)		module_init(fn)
#define security_initcall(fn)		module_init(fn)

#define module_init(initfn)					\
	static inline initcall_t __inittest(void)		\
	{ return initfn; }					\
	int init_module(void) __attribute__((alias(#initfn)));

#define module_exit(exitfn)					\
	static inline exitcall_t __exittest(void)		\
	{ return exitfn; }					\
	void cleanup_module(void) __attribute__((alias(#exitfn)));

#define __setup_param(str, unique_id, fn)	
#define __setup(str, func) 			
#endif

#define __nosavedata __section(.data..nosave)

#ifdef CONFIG_MODULES
#define __init_or_module
#define __initdata_or_module
#define __initconst_or_module
#define __INIT_OR_MODULE	.text
#define __INITDATA_OR_MODULE	.data
#define __INITRODATA_OR_MODULE	.section ".rodata","a",%progbits
#else
#define __init_or_module __init
#define __initdata_or_module __initdata
#define __initconst_or_module __initconst
#define __INIT_OR_MODULE __INIT
#define __INITDATA_OR_MODULE __INITDATA
#define __INITRODATA_OR_MODULE __INITRODATA
#endif 

#ifdef MODULE
#define __exit_p(x) x
#else
#define __exit_p(x) NULL
#endif

#endif 
