// interrupt.c
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/wait.h>
#include <linux/interrupt.h>

#define DRIVER_NAME "mygpio" // Must start with "my"
#define DEVICE_NAME "but0"   // Name of specific device

static struct gpio_desc *gpio_btn;
static struct gpio_desc *gpio_led;
static dev_t dev_num;
static struct cdev gpio_cdev;
static struct class *gpio_class;
static int flag;
static int irq_num;
static DECLARE_WAIT_QUEUE_HEAD(wq);
static int led_toggle = 0;

static irqreturn_t my_isr(int, void *)
{
    // printk(KERN_ALERT "In ISR \n");
    led_toggle = (led_toggle > 0) ? 0 : 1;
    flag = 1;
    wake_up_interruptible(&wq);
    return IRQ_HANDLED;
}

static ssize_t gpio_read(struct file *file, char __user *buf, size_t len, loff_t *off)
{
    char kbuf[16]; // Buffer allocated in kernel
    int value;
    flag = 0;
    int minor = iminor(file->f_inode);
    wait_event_interruptible(wq, flag == 1);

    // get value
    switch (minor)
    {
    case 0:
        value = gpiod_get_value(gpio_btn);
        break;
    case 1:
        value = gpiod_get_value(gpio_led);
        gpiod_set_value(gpio_led, led_toggle);
        break;
    default:
        break;
    }
    // Format string
    int n = snprintf(kbuf, sizeof(kbuf), "%d\n", value);

    // Copy string to user space
    if (copy_to_user(buf, kbuf, n))
        return -EFAULT;

    return n; // Return length of string
}

static ssize_t gpio_write(struct file *file, const char __user *buf, size_t len, loff_t *off)
{
    char kbuf[16];
    int value;
    int minor = iminor(file->f_inode);
    int dir;

    // Return error if data written is bigger than kernel buffer
    if (len >= sizeof(kbuf) - 1)
        return -EINVAL;

    // Copy data from user space to kernel space
    if (copy_from_user(kbuf, buf, len))
        return -EFAULT;

    // Append a zero-termination to ascii buffer (C-string)
    kbuf[len] = '\0';

    // Convert string to int value (aut base)
    if (kstrtoint(kbuf, 0, &value))
        return -EINVAL;
    // Use value for something...
    switch (minor)
    {
    case 0:
        dir = gpiod_get_direction(gpio_btn);
        printk(KERN_INFO "error - cannot set value of GPIO button");
        break;
    case 1:
        dir = gpiod_get_direction(gpio_led);
        gpiod_set_value(gpio_led, value);
        printk(KERN_INFO "Set to %d", value);
        break;
    default:
        break;
    }

    // Return that written length = requested write length => Everything ok
    return len;
}

// Define which file functions to use for file operations
static struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .read = gpio_read,
    .write = gpio_write,
};

/* Button init
   This is boiler-plate code for a character driver.
   It allocates an available Major number and
   reserves minor numbers 0-255.
   Creates a class and one device for minor number 0
*/
static int gpio_init(void)
{
    int ret;

    // Request an available Major number and Minor numbers [0..255]
    ret = alloc_chrdev_region(&dev_num, 0, 255, DRIVER_NAME);
    if (ret)
        return ret;

    // Initialize and add Character Driver that uses fileoperations in
    cdev_init(&gpio_cdev, &gpio_fops);
    ret = cdev_add(&gpio_cdev, dev_num, 255);
    if (ret)
        goto unregister_region;

    // Create a class in /sys/class (needed for devices)
    gpio_class = class_create(DRIVER_NAME);
    if (IS_ERR(gpio_class))
    {
        ret = PTR_ERR(gpio_class);
        goto del_cdev;
    }

    // Create a device:
    // Major = the one allocated with alloc_chrdev_region
    // Minor = 0
    // Name = <DRIVER_NAME>-<DEVICE_NAME> eg /dev/mygpio-but0
    device_create(gpio_class, NULL, MKDEV(MAJOR(dev_num), MINOR(0)), NULL, "%s-%s", DRIVER_NAME, DEVICE_NAME);
    device_create(gpio_class, NULL, MKDEV(MAJOR(dev_num), MINOR(1)), NULL, "%s-%s", DRIVER_NAME, "led");

    printk(KERN_INFO "%s: loaded (major=%d)\n", DRIVER_NAME, MAJOR(dev_num));

    // --- Initialisering af GPIO 598 / BTN5 ---
    gpio_btn = gpio_to_desc(598);
    if (!gpio_btn)
    {
        pr_err("gpio_to_desc() failed for GPIO 539\n");
        return -EINVAL;
    }
    gpio_led = gpio_to_desc(597);
    if (!gpio_led)
    {
        pr_err("gpio_to_desc() failed for GPIO 538\n");
        return -EINVAL;
    }
    ret = gpiod_direction_input(gpio_btn);
    ret = gpiod_direction_output(gpio_led, 0);
    irq_num = gpiod_to_irq(gpio_btn);
    ret = request_irq(irq_num, my_isr, IRQF_TRIGGER_FALLING,
                      "mygpio IRQ", NULL);
    return 0;

    device_destroy(gpio_class, dev_num);
    class_destroy(gpio_class);
del_cdev:
    cdev_del(&gpio_cdev);
unregister_region:
    unregister_chrdev_region(dev_num, 255);
    return ret;
}

static void gpio_exit(void)
{
    // Reverse-order cleanup!!!
    free_irq(irq_num, NULL);
    device_destroy(gpio_class, dev_num);
    device_destroy(gpio_class, dev_num + 1);
    class_destroy(gpio_class);
    cdev_del(&gpio_cdev);
    unregister_chrdev_region(dev_num, 255);
    printk(KERN_INFO "%s: unloaded\n", DRIVER_NAME);
}

module_init(gpio_init);
module_exit(gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter HM <phm@ece.au.dk>");
MODULE_DESCRIPTION("Minimal gpio char driver");

