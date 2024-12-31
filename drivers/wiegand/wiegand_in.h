#ifndef _WIEGAND_IN_H
#define _WIEGAND_IN_H

#include <linux/time.h>

struct device;
struct gpio_desc;

struct wiegand_in_platform_data;

/**
 * struct wiegand_channel - 单个 Wiegand 通道的配置
 * @data_gpio:         两根数据线的 GPIO 编号数组
 * @irq_num:           两根数据线对应的 IRQ 编号数组
 * @line:              该通道的线号
 * @data_bits:         当前帧中的比特数
 * @wg_data:           当前接收到的 Wiegand 数据
 * @falling_time:      两条数据线的下降沿时间戳
 */
struct wiegand_channel {
	int data_gpio[2];							// D0 和 D1 的 GPIO 编号
	unsigned int irq_num[2];					// D0 和 D1 的 IRQ 编号
	unsigned char line;							// 通道号
	unsigned char data_bits;					// 接收到的比特数
	unsigned long long wg_data;					// Wiegand 数据
	unsigned char wg_data_for_user_app[64];		// 发送给用户的韦根数据
	struct timer_list frame_end_timer;			// 帧结束定时器
	bool file_open_flag;
	struct wiegand_in_platform_data *plat_data;
	struct timespec64 falling_time[2];			// 两条线的下降沿时间
};

/**
 * struct wiegand_in_platform_data - Wiegand 平台数据
 * @wiegand_ch:        指向 Wiegand 通道的指针
 * @frame_end_timer:   用于检测帧结束的定时器
 * @nchannels:         Wiegand 通道数
 */
struct wiegand_in_platform_data {
	int nchannels;								// 通道数
	struct wiegand_channel wiegand_ch[];		// 指向通道数组[0]元素指针， 可能有 N 个通道
};

#endif /* _WIEGAND_IN_H */
