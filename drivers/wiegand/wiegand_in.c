/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/timekeeping.h>
#include "wiegand_in.h"

#define MAX_WG_DEVCOUNT (4)

#define DEV_NAME_WG "wiegand"

// 定义最大允许的 data_bits，避免超过缓冲区大小
#define MAX_WG_DATA_BITS 64

// 检验韦根数据有效性
#define MIN_PULSE_WIDTH 50
#define MAX_PULSE_WIDTH 500
#define FRAME_END_DELAY msecs_to_jiffies(2)

#ifdef WGDEBUG
#define wg_printk(fmt, args...) printk("\nwiegand:" fmt,## args)
#else
#define wg_printk(fmt, args...)
#endif

struct file_operations wg_fops;

static struct wiegand_in_platform_data *gl_plat_data;
static struct cdev gl_wg_cdev[MAX_WG_DEVCOUNT];
static dev_t gl_dev;
static struct class *wg_drv_class;

// poll等待队列
DECLARE_WAIT_QUEUE_HEAD(wg_waitq);

static bool check_pulse_valid(struct timespec64 falling_time) 
{
	struct timespec64 rising_time;
	long long pulse_time;

	wg_printk("%s \n", __func__);
	ktime_get_real_ts64(&rising_time);
	pulse_time = (rising_time.tv_sec - falling_time.tv_sec) * 1000000LL +
				 (rising_time.tv_nsec - falling_time.tv_nsec) / 1000;

	wg_printk("pulse_time: %d \n", pulse_time);
	return (pulse_time >= MIN_PULSE_WIDTH && pulse_time <= MAX_PULSE_WIDTH);
}

/*
 * Wiegand IRQ 处理函数
 */
static irqreturn_t wiegand_data_interrupt(int irq, void *dev_id) 
{

	struct wiegand_channel *wiegand_ch = (struct wiegand_channel*)(dev_id);
	int index = (irq == wiegand_ch->irq_num[1]) ? 1 : 0;
	int gpioval = -1;

	wg_printk("%s \n", __func__);

	gpioval = gpio_get_value(wiegand_ch->data_gpio[index]);
	if (gpioval != 0) {

		if (!check_pulse_valid(wiegand_ch->falling_time[index]))
			return IRQ_HANDLED;

		wiegand_ch->wg_data = (wiegand_ch->wg_data << 1) | index;
		wiegand_ch->data_bits++;
	} else {
		ktime_get_real_ts64(&wiegand_ch->falling_time[index]);
	}

	mod_timer(&wiegand_ch->frame_end_timer, jiffies + FRAME_END_DELAY);
	return IRQ_HANDLED;
}

/*
 * 定时器处理函数：检测帧结束
 */
static void frame_end_timer_handler(struct timer_list *t) 
{
	struct wiegand_channel *wiegand_ch = from_timer(wiegand_ch, t, frame_end_timer);
	unsigned long long tmp;
	unsigned int data_bytes;
	int i;

	wg_printk("%s \n", __func__);

	if (wiegand_ch->file_open_flag && wiegand_ch->data_bits)
	{
		memset(wiegand_ch->wg_data_for_user_app, 0, 64);

		// index0 放置数据的bit数量
		wiegand_ch->wg_data_for_user_app[0] = wiegand_ch->data_bits;
		data_bytes = ((wiegand_ch->wg_data_for_user_app[0] + 7) / 8);
		tmp = wiegand_ch->wg_data;

		for (i = 0; i < data_bytes; i++) {
			wiegand_ch->wg_data_for_user_app[data_bytes - i] = tmp & 0xFF;
			tmp >>= 8;
		}

		wake_up_interruptible(&wg_waitq);

		wiegand_ch->wg_data = 0;
		wiegand_ch->data_bits = 0;
	}
}


static int get_gpio_irq_from_devtree(struct device *dev, 
	struct device_node *pp, struct wiegand_channel *wg_channel)
{
	int ret;
	int data_gpio;

	wg_printk("%s \n", __func__);

	data_gpio = of_get_named_gpio(pp, "data0_gpios", 0);
	wg_printk("data_gpio0 = %d \r\n",data_gpio);

	if (data_gpio < 0) {
		ret = data_gpio;
		if (ret != -ENOENT) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get gpio data0, error: %d\n",
						ret);
			return ret;
		}
	}
	ret = devm_gpio_request_one(dev,data_gpio,
			GPIOF_IN,
			"wg_data0");
	if (ret) {
		dev_err(dev, "request gpio%u\n", data_gpio);
		return ret;
	}
	wg_channel->irq_num[0] = gpio_to_irq(data_gpio);

	wg_printk("irq_num0 = %d \r\n",wg_channel->irq_num[0]);

	ret = devm_request_irq(dev, wg_channel->irq_num[0], wiegand_data_interrupt, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "wg_dat0", wg_channel);
	if (ret) {
		dev_err(dev, "failed to claim irq %u\n",  gpio_to_irq(data_gpio));
		return ret;
	}
	wg_channel->data_gpio[0]  = data_gpio;
	wg_printk(" wiegand in channel%d gpio data0 = %d irq %d ok \r\n",wg_channel->line ,wg_channel->data_gpio[0],wg_channel->irq_num[0]);

	wg_channel->data_gpio[1] = of_get_named_gpio(pp, "data1_gpios", 0);

	if (wg_channel->data_gpio[1] < 0) {
		ret = wg_channel->data_gpio[1];
		if (ret != -ENOENT) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get gpio data1 , error: %d\n",
						ret);
			return ret;
		}
	}
	ret = devm_gpio_request_one(dev, wg_channel->data_gpio[1],
			GPIOF_IN,
			"wg_data1");
	if (ret) {
		dev_err(dev, "request gpio%u\n", wg_channel->data_gpio[1]);
		return ret;
	}
	wg_channel->irq_num[1] = gpio_to_irq(wg_channel->data_gpio[1]);

	wg_printk("data_gpio1 = %d \r\n",wg_channel->data_gpio[1]);
	wg_printk("irq_num1 = %d \r\n",wg_channel->irq_num[1]);

	ret = devm_request_irq(dev, wg_channel->irq_num[1], wiegand_data_interrupt, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "wg_dat1", wg_channel);
	if (ret) {
		dev_err(dev, "failed to claim irq %u\n",  wg_channel->irq_num[1]);
		return ret;
	}
	wg_printk(" wiegand in channel%d gpio data1 = %d  irq %d ok \r\n",wg_channel->line ,wg_channel->data_gpio[1],wg_channel->irq_num[1]);

	return ret;
}

/*
* Translate OpenFirmware node properties into platform_data
*/
static struct wiegand_in_platform_data *
	wiegand_in_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct wiegand_in_platform_data *pdata;
	struct wiegand_channel *wg_channel;
	struct device *dn;
	int i, ret = 0;
	dev_t dev_num;
	int nchannels;

	wg_printk("%s \n", __func__);

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nchannels = of_get_child_count(node);
	wg_printk("wiegand in channels = %d \r\n",nchannels);
	if (nchannels == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			sizeof(*pdata) + nchannels * sizeof(*wg_channel),
			GFP_KERNEL);

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->nchannels = nchannels;

	// 为 wiegand 字符设备申请 /dev/* 节点: weigand0 - weigandX
	if ((ret = alloc_chrdev_region(&gl_dev, 0, pdata->nchannels, "wiegand driver")) < 0) {
		wg_printk(KERN_ERR "Failed to register Wiegand device.\n");
		goto ret_error;
	}

	wg_printk("major =%d\n", MAJOR(gl_dev));

	wg_drv_class = class_create(THIS_MODULE, DEV_NAME_WG);
	if (IS_ERR(wg_drv_class)) {
		pr_err("Failed to create class\n");
		ret = PTR_ERR(wg_drv_class);
		goto class_create_error;
	}

	i = -1;
	for_each_child_of_node(node, pp) {
		wg_channel = &(pdata->wiegand_ch[++i]);
		wg_channel->line = i;
		wg_channel->plat_data = pdata;
		wg_printk("wg_channel line :%d\n", wg_channel->line);

		dev_num = MKDEV(MAJOR(gl_dev), MINOR(gl_dev) + i);
		cdev_init(&gl_wg_cdev[i], &wg_fops);
		gl_wg_cdev[i].owner = THIS_MODULE;
		ret = cdev_add(&gl_wg_cdev[i], dev_num, 1);
		if (ret) {
			pr_err("Failed to add cdev for wiegand%d\n", i);
			goto cdev_add_error;
		}

		dn = device_create(wg_drv_class, NULL, dev_num, NULL, "%s%d", DEV_NAME_WG,i);
		if (IS_ERR(dn)) {
			pr_err("Failed to add cdev for wiegand%d\n", i);
			ret = PTR_ERR(dn);
			goto cdev_add_error;
		}

		ret = get_gpio_irq_from_devtree(dev, pp, wg_channel);
		if (ret) {
			dev_err(dev, "wiegand_in_get_devtree_pdata %d\n",ret);
			goto ret_error;
		}
	}

	if (pdata->nchannels == 0)
		return ERR_PTR(-EINVAL);

	return pdata;

cdev_add_error:
	while (i--) {
		cdev_del(&gl_wg_cdev[i]);
	}
	class_destroy(wg_drv_class);
class_create_error:
	unregister_chrdev_region(gl_dev, pdata->nchannels);
ret_error:
	return ERR_PTR(ret);

}

int wg_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	struct wiegand_channel *wch = &gl_plat_data->wiegand_ch[minor];

	wg_printk("minor devnum: %d\n", minor);

	if(wch->file_open_flag) {
		return -ERESTARTSYS;
	}
	wch->file_open_flag = true;
	return 0;
}

int wg_release(struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	struct wiegand_channel *wch = &gl_plat_data->wiegand_ch[minor];

	wg_printk("%s \n", __func__);

	wch->file_open_flag = 0;
	return 0;
}

ssize_t wg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int minor = iminor(file_inode(filp));
	struct wiegand_channel *wch = &gl_plat_data->wiegand_ch[minor];
	size_t available_data = (wch->wg_data_for_user_app[0] + 7) / 8;

	wg_printk("%s \n", __func__);

	if (wch->wg_data_for_user_app[0] == 0) {
		return -EAGAIN;
	}

	/* 限制读取长度，防止越界 */
	count = min(count, available_data);
	// 加 1 是因为第一个字节是存储的实际bit数量，不是实际接收到的数据
	count = count + 1;

	if (copy_to_user(buf, wch->wg_data_for_user_app, count)) {
		return -EFAULT;
	}

	memset(wch->wg_data_for_user_app, 0, count);

	return count;
}


ssize_t  wg_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
	wg_printk("%s %s\n", __func__, buf);
}

long  wg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EFAULT;
	wg_printk("%s \n", __func__);
}


static unsigned int wg_poll(struct file *file, struct poll_table_struct *wait)
{
	int minor = iminor(file_inode(file));
	struct wiegand_channel *wch = &gl_plat_data->wiegand_ch[minor];

	unsigned int mask = 0;
	wg_printk("%s \n", __func__);

	/* 将当前进程添加到等待队列 */
	poll_wait(file, &wg_waitq, wait);

	if (wch->wg_data_for_user_app[0] != 0) {
		mask |= POLLIN;
	}

	return mask;
}

struct file_operations wg_fops = {
	.owner 			= THIS_MODULE,
	.read 			= wg_read,
	.write 			= wg_write,
	.unlocked_ioctl	= wg_ioctl,
	.open			= wg_open,
	.release		= wg_release,
	.poll			= wg_poll
};

static int wiegand_init(struct wiegand_in_platform_data *pdata)
{
	wg_printk("%s \n", __func__);

	if (pdata->nchannels < 1 )
		return -ENODEV;

	// 对每个 wiegand 设备进行初始化，每一路 wiegand 使用单独的定时器
	for ( int n = 0; n< pdata->nchannels && n < MAX_WG_DEVCOUNT; n++ )
	{
		struct wiegand_channel *wch = &pdata->wiegand_ch[n];

		timer_setup(&wch->frame_end_timer, frame_end_timer_handler, 0);
		wch->frame_end_timer.expires = jiffies + msecs_to_jiffies(3);
		wch->data_bits = 0;
		wch->wg_data = 0;
		add_timer(&wch->frame_end_timer);
	}

	return 0;
}

static int wiegand_in_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	gl_plat_data = dev_get_platdata(dev);
	wg_printk("%s %ld %d\n",__func__,msecs_to_jiffies(200),sizeof(long long));

	if (!gl_plat_data) {
		gl_plat_data = wiegand_in_get_devtree_pdata(dev);
		if (IS_ERR(gl_plat_data))
			return PTR_ERR(gl_plat_data);
	}

	wiegand_init(gl_plat_data);
	platform_set_drvdata(pdev, gl_plat_data);

	return 0;
}

static int wiegand_in_remove(struct platform_device *pdev) 
{
	int i = 0;

	for (i = 0; i < gl_plat_data->nchannels; i++) {
		device_destroy(wg_drv_class, MKDEV(MAJOR(gl_dev), MINOR(gl_dev) + i));
		cdev_del(&gl_wg_cdev[i]);
	}

	class_destroy(wg_drv_class);
	unregister_chrdev_region(gl_dev, gl_plat_data->nchannels);
	wg_printk("Wiegand driver removed\n");
	return 0;
}

static const struct of_device_id wiegand_in_of_match[] = 
{
	{ .compatible = "bohai-wiegand-in", },
	{ },
};
MODULE_DEVICE_TABLE(of, wiegand_in_of_match);

static struct platform_driver wiegand_in_device_driver = 
{
	.probe      = wiegand_in_probe,
	.remove     = wiegand_in_remove,
	.driver     = {
		.name   = "bohai-wiegand-in",
		.of_match_table = of_match_ptr(wiegand_in_of_match),
	}
};

static int __init wiegand_in_init(void) 
{
	int ret = 0;

	ret = platform_driver_register(&wiegand_in_device_driver);
	if (ret) {
		pr_err("Failed to register Wiegand platform driver.\n");
		return ret;
	}

	printk("Wiegand driver initialized.\n");
	return ret;
}

static void __exit wiegand_in_exit(void) 
{
	platform_driver_unregister(&wiegand_in_device_driver);

	printk("Wiegand driver exited.\n");
}

module_init(wiegand_in_init);
module_exit(wiegand_in_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Improved by GPT-4");
MODULE_DESCRIPTION("Improved Wiegand Protocol Driver");
