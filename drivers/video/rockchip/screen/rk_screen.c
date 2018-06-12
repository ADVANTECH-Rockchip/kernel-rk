
#include <linux/rk_fb.h>
#include <linux/device.h>
#include "lcd.h"
#include "../hdmi/rockchip-hdmi.h"

static struct rk_screen *rk_screen;
static struct rk_screen *prmry_screen;
static struct rk_screen *extend_screen;

int rk_fb_get_extern_screen(struct rk_screen *screen)
{
#ifdef CONFIG_ARCH_ADVANTECH
	if(rk_fb_is_dual_lcd_mode()){
		if (unlikely(!extend_screen) || unlikely(!screen))
			return -1;

		memcpy(screen, extend_screen, sizeof(struct rk_screen));
	} else {
#endif
		if (unlikely(!rk_screen) || unlikely(!screen))
			return -1;

		memcpy(screen, rk_screen, sizeof(struct rk_screen));
#ifdef CONFIG_ARCH_ADVANTECH
	}
#endif
	screen->dsp_lut = NULL;
	screen->cabc_lut = NULL;
	screen->type = SCREEN_NULL;

	return 0;
}

int  rk_fb_get_prmry_screen(struct rk_screen *screen)
{
#ifdef CONFIG_ARCH_ADVANTECH
	if(rk_fb_is_dual_lcd_mode()){
		if (unlikely(!prmry_screen) || unlikely(!screen))
			return -1;

		memcpy(screen, prmry_screen, sizeof(struct rk_screen));
	} else {
#endif
		if (unlikely(!rk_screen) || unlikely(!screen))
			return -1;

		memcpy(screen, rk_screen, sizeof(struct rk_screen));
#ifdef CONFIG_ARCH_ADVANTECH
	}
#endif
	return 0;
}

int rk_fb_get_screen(struct rk_screen *screen, int prop)
{
   if (unlikely(!screen))
       return -1;
   if (prop == PRMRY) {
       if (unlikely(!prmry_screen)) {
           pr_err(">>please init %s screen info in dtsi file<<\n",
            (prop == PRMRY) ? "prmry" : "extend");
           return -1;
       }
	   memcpy(screen, prmry_screen, sizeof(struct rk_screen));
   } else {
       if (unlikely(!extend_screen)) {
           pr_err(">>please init %s screen info in dtsi file<<\n",
            (prop == PRMRY) ? "prmry" : "extend");
           return -1;
       }
	   memcpy(screen, extend_screen, sizeof(struct rk_screen));
   }

	return 0;
}

int rk_fb_set_screen(struct rk_screen *screen, int prop)
{
	struct rk_screen *cur_screen = NULL;
	
    if (unlikely(!screen))
        return -1;
    if (prop == PRMRY) {
        if (unlikely(!prmry_screen)) {
            pr_err(">>please init %s screen info in dtsi file<<\n",
            (prop == PRMRY) ? "prmry" : "extend");
            return -1;
        }
        cur_screen = prmry_screen;
    } else {
        if (unlikely(!extend_screen)) {
            pr_err(">>please init %s screen info in dtsi file<<\n",
            (prop == PRMRY) ? "prmry" : "extend");
            return -1;
        }
        cur_screen = extend_screen;
    }

	cur_screen->lcdc_id = screen->lcdc_id;
	cur_screen->screen_id = screen->screen_id;
	cur_screen->x_mirror = screen->x_mirror;
	cur_screen->y_mirror = screen->y_mirror;
	cur_screen->overscan.left = screen->overscan.left;
	cur_screen->overscan.top = screen->overscan.left;
	cur_screen->overscan.right = screen->overscan.left;
	cur_screen->overscan.bottom = screen->overscan.left;

	return 0;
}

int rk_fb_set_prmry_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	rk_screen->lcdc_id = screen->lcdc_id;
	rk_screen->screen_id = screen->screen_id;
	rk_screen->x_mirror = screen->x_mirror;
	rk_screen->y_mirror = screen->y_mirror;
	rk_screen->overscan.left = screen->overscan.left;
	rk_screen->overscan.top = screen->overscan.left;
	rk_screen->overscan.right = screen->overscan.left;
	rk_screen->overscan.bottom = screen->overscan.left;
	return 0;
}

#ifdef CONFIG_ARCH_ADVANTECH
struct rk_screen *rk_fb_get_screen_point(int prop)
{
	if (prop == PRMRY) {
		if (unlikely(!prmry_screen)) {
			pr_err(">>please init %s screen info in dtsi file<<\n",
				(prop == PRMRY) ? "prmry" : "extend");
			return NULL;
		} else
			return prmry_screen;
	} else if (prop == EXTEND) {
		if (unlikely(!extend_screen)) {
			pr_err(">>please init %s screen info in dtsi file<<\n",
			(	prop == PRMRY) ? "prmry" : "extend");
			return NULL;
		} else
			return extend_screen;
	} else {
		if (unlikely(!rk_screen)) {
			pr_err(">>please init screen info in dtsi file<<\n");
			return NULL;
		} else
			return rk_screen;
	}
}
#endif

size_t get_fb_size_dual_lcd(u8 reserved_fb, struct rk_screen *screen)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!screen))
		return 0;

	xres = screen->mode.xres;
	yres = screen->mode.yres;

	/* align as 64 bytes(16*4) in an odd number of times */
	xres = ALIGN_64BYTE_ODD_TIMES(xres, ALIGN_PIXEL_64BYTE_RGB8888);
	if (reserved_fb == ONE_FB_BUFFER)
		size = xres * yres << 2;	/* one buffer */
	else if (reserved_fb == TWO_FB_BUFFER)
		size = (xres * yres << 2) << 1;	/* two buffer */
	else
#if defined(CONFIG_THREE_FB_BUFFER)
		size = (xres * yres << 2) * 3;	/* three buffer */
#else
		size = (xres * yres << 2) << 1; /* two buffer */
#endif

	return ALIGN(size, SZ_1M);
}

size_t get_fb_size(u8 reserved_fb)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!rk_screen))
		return 0;

	xres = rk_screen->mode.xres;
	yres = rk_screen->mode.yres;

	/* align as 64 bytes(16*4) in an odd number of times */
	xres = ALIGN_64BYTE_ODD_TIMES(xres, ALIGN_PIXEL_64BYTE_RGB8888);
	if (reserved_fb == ONE_FB_BUFFER)
		size = xres * yres << 2;	/* one buffer */
	else if (reserved_fb == TWO_FB_BUFFER)
		size = (xres * yres << 2) << 1;	/* two buffer */
	else
#if defined(CONFIG_THREE_FB_BUFFER)
		size = (xres * yres << 2) * 3;	/* three buffer */
#else
		size = (xres * yres << 2) << 1; /* two buffer */
#endif

	return ALIGN(size, SZ_1M);
}

static int rk_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
#ifdef CONFIG_ARCH_ADVANTECH
	int screen_prop = 0;
	struct device_node *screen_np;
	struct rk_screen *screen;
#endif

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

#ifdef CONFIG_ARCH_ADVANTECH
	if(rk_fb_is_dual_lcd_mode()){
		for_each_child_of_node(np, screen_np){
			screen = devm_kzalloc(&pdev->dev,
						 sizeof(struct rk_screen), GFP_KERNEL);
			if (!screen) {
				dev_err(&pdev->dev, "kmalloc for rk screen fail!");
				return  -ENOMEM;
			}
			screen->pwrlist_head = devm_kzalloc(&pdev->dev,
					sizeof(struct list_head), GFP_KERNEL);
			if (!screen->pwrlist_head) {
				dev_err(&pdev->dev, "kmalloc for rk_screen pwrlist_head fail!");
				return  -ENOMEM;
			}
			of_property_read_u32(screen_np, "screen_prop", &screen_prop);
			if (screen_prop == PRMRY)
				prmry_screen = screen;
			else if (screen_prop == EXTEND)
				extend_screen = screen;
			else
				dev_err(&pdev->dev, "unknow screen prop: %d\n",screen_prop);
			screen->prop = screen_prop;
			screen->dev = &pdev->dev;
			ret = rk_fb_prase_timing_dt(screen_np, screen);
			pr_info("%s screen timing parse %s\n",
				(screen_prop == PRMRY) ? "prmry" : "extend",
				ret ? "failed" : "success");
			if(rk_fb_get_lvds_prop() != screen_prop) {
				ret = rk_disp_pwr_ctr_parse_dt_dual_lcd(screen_np, screen);
				pr_info("%s screen power ctrl parse %s\n",
					(screen_prop == PRMRY) ? "prmry" : "extend",
					ret ? "failed" : "success");
			}
		}
	} else {
#endif
		rk_screen = devm_kzalloc(&pdev->dev,
				sizeof(struct rk_screen), GFP_KERNEL);
		if (!rk_screen) {
			dev_err(&pdev->dev, "kmalloc for rk screen fail!");
			return  -ENOMEM;
		}
#ifdef CONFIG_ARCH_ADVANTECH
		rk_screen->prop = PRMRY;
		if(rk_fb_get_lvds_prop()) {
			rk_screen->pwrlist_head = devm_kzalloc(&pdev->dev,
								sizeof(struct list_head), GFP_KERNEL);
			if (!rk_screen->pwrlist_head) {
				dev_err(&pdev->dev, "kmalloc for rk_screen pwrlist_head fail!");
				return  -ENOMEM;
			}
		}
#endif
		ret = rk_fb_prase_timing_dt(np, rk_screen);
#ifdef CONFIG_ARCH_ADVANTECH
	}
#endif

	dev_info(&pdev->dev, "rockchip screen probe %s\n",
				ret ? "failed" : "success");
	return ret;
}

static const struct of_device_id rk_screen_dt_ids[] = {
	{ .compatible = "rockchip,screen", },
	{}
};

static struct platform_driver rk_screen_driver = {
	.probe		= rk_screen_probe,
	.driver		= {
		.name	= "rk-screen",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rk_screen_dt_ids),
	},
};

static int __init rk_screen_init(void)
{
	return platform_driver_register(&rk_screen_driver);
}

static void __exit rk_screen_exit(void)
{
	platform_driver_unregister(&rk_screen_driver);
}

fs_initcall(rk_screen_init);
module_exit(rk_screen_exit);

