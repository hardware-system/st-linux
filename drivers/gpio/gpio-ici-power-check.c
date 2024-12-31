#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

struct power_loss_dev {
    int control_gpio;
    int irq;
};

// 中断处理程序
static irqreturn_t power_loss_isr(int irq, void *dev_id) {
    struct power_loss_dev *pdev = (struct power_loss_dev *)dev_id;

    // 置位控制 GPIO
    gpio_set_value(pdev->control_gpio, 1); 
    // pr_info("Power loss detected, GPIO set to high.\n");

    return IRQ_HANDLED;
}

static int power_loss_probe(struct platform_device *pdev) {
    struct power_loss_dev *pl_dev;
    struct device *dev = &pdev->dev;
    int err;

    // 分配设备结构体内存
    pl_dev = devm_kzalloc(dev, sizeof(struct power_loss_dev), GFP_KERNEL);
    if (!pl_dev)
        return -ENOMEM;

    // 从设备树中获取 GPIO 和 IRQ

    pl_dev->control_gpio = of_get_named_gpio(dev->of_node, "control-gpios", 0);
    if (!gpio_is_valid(pl_dev->control_gpio)) {
        dev_err(dev, "Invalid control GPIO\n");
        return -EINVAL;
    }

    // 请求 GPIO
    err = devm_gpio_request_one(dev, pl_dev->control_gpio, GPIOF_OUT_INIT_LOW, "control_gpio");
    if (err) {
        dev_err(dev, "Failed to request control GPIO\n");
        return err;
    }

    // 将 GPIO 转换为中断号
    pl_dev->irq = platform_get_irq(pdev, 0);
    if (pl_dev->irq < 0) {
        dev_err(dev, "Failed to get IRQ number for GPIO\n");
        return pl_dev->irq;
    }

    // 注册中断处理程序
    err = devm_request_irq(dev, pl_dev->irq, power_loss_isr, IRQF_TRIGGER_FALLING, "power_loss_irq", pl_dev);
    if (err) {
        dev_err(dev, "Failed to request IRQ\n");
        return err;
    }

    platform_set_drvdata(pdev, pl_dev);

    pr_info("Power loss driver probed successfully.\n");
    return 0;
}

static int power_loss_remove(struct platform_device *pdev) {
    struct power_loss_dev *pl_dev = platform_get_drvdata(pdev);

    devm_free_irq(&pdev->dev, pl_dev->irq, pl_dev);
    pr_info("Power loss driver removed.\n");
    return 0;
}

static const struct of_device_id power_loss_of_match[] = {
    { .compatible = "ici,power-loss-detector", },
    { }
};
MODULE_DEVICE_TABLE(of, power_loss_of_match);

static struct platform_driver power_loss_driver = {
    .probe = power_loss_probe,
    .remove = power_loss_remove,
    .driver = {
        .name = "power_loss_detector",
        .of_match_table = power_loss_of_match,
    },
};

module_platform_driver(power_loss_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Liuxin");
MODULE_DESCRIPTION("Power loss detector with GPIO control");

