#define pr_fmt(fmt) "switch_camera: " fmt

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/switch_camera.h>
#include <linux/workqueue.h>

#define KEY_CAM_SWITCH_FRONT	0x256
#define KEY_CAM_SWITCH_BACK	0x257

/*
 * The barrel is turned by hand and both detectors chatter while it travels:
 * several edges arrive, and the level only settles once the barrel is home.
 * Acting on the raw edge means acting on a transient reading — and since no
 * further edge follows, the sensor can be left flipped the wrong way (seen
 * live: the return to the selfie position corrected the image only sometimes).
 * So debounce: every edge (re)arms a delayed worker, and the decision is taken
 * from the settled level.
 */
#define SWITCH_CAMERA_DEBOUNCE_MS	250
#define SWITCH_CAMERA_VERIFY_MS		800

struct switch_camera_info {
	struct input_dev *ipdev;
	int gpio_out;
	int gpio_mvout;
	int irq_out;
	int irq_mvout;
	int position;			/* CAMERA_BARREL_* */
	struct delayed_work work;
	bool verify_pass;
};

/*
 * Single instance: the camera sensor driver needs the position and has no
 * handle to this device (they are unrelated nodes in DT).
 */
static struct switch_camera_info *switch_camera;

static int switch_camera_read_position(struct switch_camera_info *info)
{
	int out = gpio_get_value(info->gpio_out);
	int mvout = gpio_get_value(info->gpio_mvout);

	/* Both detectors idle (pulled up) => barrel in the selfie position. */
	if (out == 1 && mvout == 1)
		return CAMERA_BARREL_FRONT;

	return CAMERA_BARREL_BACK;
}

/**
 * camera_barrel_position - where the rotating camera barrel currently points
 *
 * Returns CAMERA_BARREL_BACK / CAMERA_BARREL_FRONT, or CAMERA_BARREL_UNKNOWN
 * if this driver is not probed (any other board).
 */
int camera_barrel_position(void)
{
	if (!switch_camera)
		return CAMERA_BARREL_UNKNOWN;

	return switch_camera_read_position(switch_camera);
}
EXPORT_SYMBOL(camera_barrel_position);

static ssize_t camera_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_camera_info *info = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		switch_camera_read_position(info) == CAMERA_BARREL_FRONT ?
		"front" : "back");
}
static DEVICE_ATTR_RO(camera);

static void switch_camera_work(struct work_struct *work)
{
	struct switch_camera_info *info = container_of(to_delayed_work(work),
		struct switch_camera_info, work);
	int position = switch_camera_read_position(info);

	/*
	 * Re-apply the sensor-side flip that compensates the rotation, even when
	 * the settled position is the one we already knew: while the barrel was
	 * travelling, the sensor library may have rewritten those registers from
	 * a transient reading, and this is where that gets straightened out.  The
	 * picture is corrected right away, mid-stream, with the camera app open —
	 * as stock behaved.
	 */
	msm_sensor_barrel_changed();

	/*
	 * ...and once more shortly after, to catch a library write that landed
	 * just now with the barrel still in flight.
	 */
	if (!info->verify_pass) {
		info->verify_pass = true;
		mod_delayed_work(system_wq, &info->work,
			msecs_to_jiffies(SWITCH_CAMERA_VERIFY_MS));
	} else {
		info->verify_pass = false;
	}

	if (position == info->position)
		return;

	info->position = position;
	pr_info("barrel moved to %s position\n",
		position == CAMERA_BARREL_FRONT ? "front" : "back");

	input_report_key(info->ipdev,
		position == CAMERA_BARREL_FRONT ?
		KEY_CAM_SWITCH_FRONT : KEY_CAM_SWITCH_BACK, 1);
	input_report_key(info->ipdev,
		position == CAMERA_BARREL_FRONT ?
		KEY_CAM_SWITCH_FRONT : KEY_CAM_SWITCH_BACK, 0);
	input_sync(info->ipdev);
}

static irqreturn_t switch_camera_irq(int irq, void *data)
{
	struct switch_camera_info *info = data;

	/* Wait for the barrel to settle; see SWITCH_CAMERA_DEBOUNCE_MS. */
	info->verify_pass = false;
	mod_delayed_work(system_wq, &info->work,
		msecs_to_jiffies(SWITCH_CAMERA_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static int switch_camera_request_gpio(struct device *dev, int gpio,
		const char *label, const char *irq_label, int *irq,
		void *data)
{
	int rc;

	rc = devm_gpio_request_one(dev, gpio, GPIOF_IN, label);
	if (rc) {
		dev_err(dev, "failed to request %s (gpio %d): %d\n",
			label, gpio, rc);
		return rc;
	}

	*irq = gpio_to_irq(gpio);
	rc = devm_request_threaded_irq(dev, *irq, NULL, switch_camera_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		irq_label, data);
	if (rc) {
		dev_err(dev, "failed to request irq for %s: %d\n", label, rc);
		return rc;
	}

	return 0;
}

static int switch_camera_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct switch_camera_info *info;
	struct pinctrl *pinctrl;
	int rc;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->gpio_out = of_get_named_gpio(dev->of_node,
		"switch_camera,irq-out", 0);
	info->gpio_mvout = of_get_named_gpio(dev->of_node,
		"switch_camera,irq-mvout", 0);
	if (!gpio_is_valid(info->gpio_out) || !gpio_is_valid(info->gpio_mvout)) {
		dev_err(dev, "missing detector gpios\n");
		return -EINVAL;
	}

	/* The detectors are open-drain: the pins must be pulled up. */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "no pinctrl default state: %ld\n",
			PTR_ERR(pinctrl));

	info->ipdev = devm_input_allocate_device(dev);
	if (!info->ipdev)
		return -ENOMEM;

	info->ipdev->name = "camera-switch-input";
	input_set_capability(info->ipdev, EV_KEY, KEY_CAM_SWITCH_FRONT);
	input_set_capability(info->ipdev, EV_KEY, KEY_CAM_SWITCH_BACK);

	rc = input_register_device(info->ipdev);
	if (rc) {
		dev_err(dev, "failed to register input device: %d\n", rc);
		return rc;
	}

	rc = switch_camera_request_gpio(dev, info->gpio_out, "cam_switch_out",
		"switch_camera_out", &info->irq_out, info);
	if (rc)
		return rc;

	rc = switch_camera_request_gpio(dev, info->gpio_mvout,
		"cam_switch_mvout", "switch_camera_mvout", &info->irq_mvout,
		info);
	if (rc)
		return rc;

	INIT_DELAYED_WORK(&info->work, switch_camera_work);
	info->position = switch_camera_read_position(info);
	dev_set_drvdata(dev, info);

	rc = device_create_file(&info->ipdev->dev, &dev_attr_camera);
	if (rc)
		dev_warn(dev, "failed to create camera sysfs node: %d\n", rc);
	else
		dev_set_drvdata(&info->ipdev->dev, info);

	switch_camera = info;
	dev_info(dev, "barrel in %s position (gpio%d=%d gpio%d=%d)\n",
		info->position == CAMERA_BARREL_FRONT ? "front" : "back",
		info->gpio_out, gpio_get_value(info->gpio_out),
		info->gpio_mvout, gpio_get_value(info->gpio_mvout));

	return 0;
}

static int switch_camera_remove(struct platform_device *pdev)
{
	struct switch_camera_info *info = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&info->work);
	device_remove_file(&info->ipdev->dev, &dev_attr_camera);
	switch_camera = NULL;

	return 0;
}

static const struct of_device_id switch_camera_of_match[] = {
	{ .compatible = "cameradet,switch_camera", },
	{ },
};
MODULE_DEVICE_TABLE(of, switch_camera_of_match);

static struct platform_driver switch_camera_driver = {
	.probe = switch_camera_probe,
	.remove = switch_camera_remove,
	.driver = {
		.name = "switch_camera",
		.of_match_table = switch_camera_of_match,
	},
};
module_platform_driver(switch_camera_driver);

MODULE_DESCRIPTION("Rotating camera barrel position detector");
MODULE_LICENSE("GPL v2");
