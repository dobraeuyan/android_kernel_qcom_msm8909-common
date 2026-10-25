#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define KEY_HALL_OPEN			KEY_WAKEUP
#define KEY_HALL_CLOSE			KEY_SLEEP
#define KEY_HALL_HOLDER_OPEN		0x254
#define KEY_HALL_HOLDER_CLOSE		0x255

#define HALL_COVER_IRQ	"hall-cover"
#define HALL_CAM_IRQ	"hall-cam"
#define HALL_HOLDER_IRQ	"hall-holder"

struct hall_switch_irq {
	int gpio;
	int irq;
	int state;
	char irq_name[32];
};

struct hall_switch_info {
	struct mutex io_lock;
	struct hall_switch_irq cover;
	struct hall_switch_irq cam;
	struct hall_switch_irq holder;
	struct input_dev *ipdev;
};

static irqreturn_t hall_interrupt(int irq, void *data)
{
	struct hall_switch_irq *hall = data;
	int hall_gpio;
	struct hall_switch_info *hall_info;

	disable_irq_nosync(irq);
	hall_gpio = gpio_get_value(hall->gpio);
	pr_info("hall irq name = %s irq = %d state = %d\n",
		hall->irq_name, irq, hall_gpio);
	if (hall_gpio == hall->state) {
		enable_irq(irq);
		return IRQ_HANDLED;
	}
	hall->state = hall_gpio;

	if (!strcmp(hall->irq_name, HALL_COVER_IRQ)) {
		hall_info = container_of(hall, struct hall_switch_info, cover);
		if (hall->state) {
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_OPEN, 0);
			input_sync(hall_info->ipdev);
		} else {
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 1);
			input_report_key(hall_info->ipdev, KEY_HALL_CLOSE, 0);
			input_sync(hall_info->ipdev);
		}
	} else if (!strcmp(hall->irq_name, HALL_CAM_IRQ)) {
		hall_info = container_of(hall, struct hall_switch_info, cam);
	} else if (!strcmp(hall->irq_name, HALL_HOLDER_IRQ)) {
		hall_info = container_of(hall, struct hall_switch_info, holder);
		if (hall->state) {
			input_report_key(hall_info->ipdev,
					 KEY_HALL_HOLDER_CLOSE, 1);
			input_report_key(hall_info->ipdev,
					 KEY_HALL_HOLDER_CLOSE, 0);
			input_sync(hall_info->ipdev);
		} else {
			input_report_key(hall_info->ipdev,
					 KEY_HALL_HOLDER_OPEN, 1);
			input_report_key(hall_info->ipdev,
					 KEY_HALL_HOLDER_OPEN, 0);
			input_sync(hall_info->ipdev);
		}
	} else {
		pr_err("hall irq: unknown irq name = %s\n", hall->irq_name);
	}

	enable_irq(irq);
	return IRQ_HANDLED;
}

static int hall_init_irq(struct hall_switch_irq *hall)
{
	int rc;

	if (!gpio_is_valid(hall->gpio)) {
		pr_err("hall: irq gpio not provided\n");
		return -EINVAL;
	}

	rc = gpio_request(hall->gpio, hall->irq_name);
	if (rc < 0) {
		pr_err("hall: gpio_request fail rc=%d\n", rc);
		return rc;
	}

	rc = gpio_direction_input(hall->gpio);
	if (rc < 0) {
		pr_err("hall: gpio_direction_input fail rc=%d\n", rc);
		goto free_gpio;
	}
	hall->state = gpio_get_value(hall->gpio);

	hall->irq = gpio_to_irq(hall->gpio);
	rc = request_threaded_irq(hall->irq, NULL, hall_interrupt,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		hall->irq_name, hall);
	if (rc < 0) {
		pr_err("hall: request_irq fail rc=%d\n", rc);
		goto free_gpio;
	}
	irq_set_irq_wake(hall->irq, 1);

	return 0;

free_gpio:
	gpio_free(hall->gpio);
	return rc;
}

static int hall_parse_dt(struct device *dev, struct hall_switch_info *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;

	pdata->cover.gpio = of_get_named_gpio_flags(np, "hall,irq-cover",
			0, &flags);
	if (pdata->cover.gpio < 0)
		pr_warn("hall: no cover gpio\n");
	else
		strlcpy(pdata->cover.irq_name, HALL_COVER_IRQ,
			sizeof(pdata->cover.irq_name));

	pdata->cam.gpio = of_get_named_gpio_flags(np, "hall,irq-cam",
			0, &flags);
	if (pdata->cam.gpio < 0)
		pr_warn("hall: no cam gpio\n");
	else
		strlcpy(pdata->cam.irq_name, HALL_CAM_IRQ,
			sizeof(pdata->cam.irq_name));

	pdata->holder.gpio = of_get_named_gpio_flags(np, "hall,irq-holder",
			0, &flags);
	if (pdata->holder.gpio < 0)
		pr_warn("hall: no holder gpio\n");
	else
		strlcpy(pdata->holder.irq_name, HALL_HOLDER_IRQ,
			sizeof(pdata->holder.irq_name));

	return 0;
}

static int hall_probe(struct platform_device *pdev)
{
	int rc = 0;
	int err;
	struct hall_switch_info *hall_info;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hall_info = kzalloc(sizeof(*hall_info), GFP_KERNEL);
	if (!hall_info)
		return -ENOMEM;

	err = hall_parse_dt(&pdev->dev, hall_info);
	if (err) {
		dev_err(&pdev->dev, "hall DT parsing failed\n");
		goto free_struct;
	}

	mutex_init(&hall_info->io_lock);
	platform_set_drvdata(pdev, hall_info);

	/* input device */
	hall_info->ipdev = input_allocate_device();
	if (!hall_info->ipdev) {
		pr_err("hall: input_allocate_device fail\n");
		rc = -ENOMEM;
		goto free_struct;
	}
	hall_info->ipdev->name = "hall-switch-input";
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_OPEN);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_CLOSE);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_HOLDER_OPEN);
	input_set_capability(hall_info->ipdev, EV_KEY, KEY_HALL_HOLDER_CLOSE);
	/* Stock set INPUT_PROP_NO_DUMMY_RELEASE here — a downstream QCOM input
	 * prop absent in this 4.9 tree. Not needed: every event below is emitted
	 * as an explicit press+release pair, so there is no dummy release to
	 * suppress. */
	rc = input_register_device(hall_info->ipdev);
	if (rc) {
		pr_err("hall: input_register_device fail rc=%d\n", rc);
		goto input_free;
	}

	/* interrupts — only the halls actually wired in DT are initialised */
	if (gpio_is_valid(hall_info->cover.gpio))
		rc = hall_init_irq(&hall_info->cover);
	if (!rc && gpio_is_valid(hall_info->cam.gpio))
		rc = hall_init_irq(&hall_info->cam);
	if (!rc && gpio_is_valid(hall_info->holder.gpio))
		rc = hall_init_irq(&hall_info->holder);
	if (rc) {
		pr_err("hall: hall_init_irq fail rc=%d\n", rc);
		goto unregister_input;
	}

	return 0;

unregister_input:
	input_unregister_device(hall_info->ipdev);
	hall_info->ipdev = NULL;
input_free:
	if (hall_info->ipdev)
		input_free_device(hall_info->ipdev);
free_struct:
	platform_set_drvdata(pdev, NULL);
	kfree(hall_info);
	return rc;
}

static int hall_remove(struct platform_device *pdev)
{
	struct hall_switch_info *hall = platform_get_drvdata(pdev);

	if (gpio_is_valid(hall->cover.gpio)) {
		free_irq(hall->cover.irq, &hall->cover);
		gpio_free(hall->cover.gpio);
	}
	if (gpio_is_valid(hall->cam.gpio)) {
		free_irq(hall->cam.irq, &hall->cam);
		gpio_free(hall->cam.gpio);
	}
	if (gpio_is_valid(hall->holder.gpio)) {
		free_irq(hall->holder.irq, &hall->holder);
		gpio_free(hall->holder.gpio);
	}
	input_unregister_device(hall->ipdev);
	kfree(hall);
	return 0;
}

static const struct of_device_id sn_match_table[] = {
	{ .compatible = "hall-switch,och175", },
	{ },
};
MODULE_DEVICE_TABLE(of, sn_match_table);

static struct platform_driver hall_driver = {
	.probe	= hall_probe,
	.remove	= hall_remove,
	.driver	= {
		.name		= "hall-switch",
		.owner		= THIS_MODULE,
		.of_match_table	= sn_match_table,
	},
};

module_platform_driver(hall_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("mawenke");
MODULE_DESCRIPTION("OCH175 hall-switch driver (Lenovo Yoga Tab 3 / bamboo)");
