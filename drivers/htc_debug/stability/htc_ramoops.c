#include <linux/pstore_ram.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>


static struct ramoops_platform_data ramoops_data = {0};

static struct platform_device ramoops_dev  = {
	.name = "ramoops",
	.dev = {
		.platform_data = &ramoops_data,
	},
};

static int __init htc_ramoops_init(void)
{
	int ret = -EINVAL;
	struct device_node *node;
	struct device_node *pnode;
	u32 val;

	node = of_find_compatible_node(NULL, NULL, "htc,ramoops");
	if (node) {
		if (!of_device_is_available(node))
			goto out;
	} else {
		pr_err("%s: can't find compatible 'htc,ramoops'\n", __func__);
		return ret;
	}

	
	pnode = of_parse_phandle(node, "linux,contiguous-region", 0);
	if (pnode) {
		const u32 *addr;
		u64 len;
		addr = of_get_address(pnode, 0, &len, NULL);
		if (!addr) {
			pr_err("failed to parse the ramoops memory address\n");
			of_node_put(pnode);
			ret = -EINVAL;
			goto out;
		}
		ramoops_data.mem_size = (u32)len;
		ramoops_data.mem_address = (u32) of_read_ulong(addr, 2);
		of_node_put(pnode);
	} else {
		pr_err("%s: mem reservation for ramoops not present\n",
				__func__);
		ret = -EINVAL;
		goto out;
	}

	
	ret = of_property_read_u32(node, "record-size", &val);
	if (ret) {
		pr_err("%s: error reading record-size property of ramoops.\n",
				__func__);
		goto out;
	}
	ramoops_data.record_size = val;

	ret = of_property_read_u32(node, "console-size", &val);
	if (ret) {
		pr_err("%s: error reading console-size property of ramoops.\n",
				__func__);
		goto out;
	}
	ramoops_data.console_size = val;

	ret = of_property_read_u32(node, "pmsg-size", &val);
	if (ret) {
		pr_err("%s: error reading pmsg-size property of ramoops.\n",
				__func__);
		goto out;
	}
	ramoops_data.pmsg_size = val;

	ret = of_property_read_u32(node, "ftrace-size", &val);
	if (ret) {
		pr_err("%s: error reading ftrace-size property of ramoops.\n",
				__func__);
		goto out;
	}
	ramoops_data.ftrace_size = val;

	ret = of_property_read_u32(node, "dump-oops", &val);
	if (ret) {
		pr_err("%s: error reading dump-oops property of ramoops.\n",
				__func__);
		goto out;
	}
	ramoops_data.dump_oops = val;

	if ((ramoops_data.console_size + ramoops_data.ftrace_size) >
			ramoops_data.mem_size) {
		pr_err("Ramoops memory region out of size.\n");
		ret =  -EINVAL;
		goto out;
	}

	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		pr_err("Unable to register ramoops platform device. Resource @ 0x%lx (0x%lx)\n",
				ramoops_data.mem_address,
				ramoops_data.mem_size);
	}

out:
	of_node_put(node);
	return ret;
}
core_initcall(htc_ramoops_init);
