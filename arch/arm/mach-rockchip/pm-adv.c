#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <dt-bindings/gpio/gpio.h>

static int pm_reset_gpio;

void pm_adv_reboot(void)
{
	if(gpio_is_valid(pm_reset_gpio)) {
		gpio_direction_output(pm_reset_gpio,1);
		mdelay(5);
		gpio_direction_output(pm_reset_gpio,0);
		mdelay(500);
	}
}

static int __init pm_adv_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "rockchip,key");
	if(np) {
		pm_reset_gpio = of_get_named_gpio_flags(np,"reset-gpio",0,NULL);
		if(gpio_is_valid(pm_reset_gpio))
			gpio_request(pm_reset_gpio,"pm_reset_gpio");
	}
	
	return 0;
}
module_init(pm_adv_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pm for advantech");
