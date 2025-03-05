// SPDX-License-Identifier: GPL-2.0
/* ########################################################### */
/* #                    Imports                              # */
/* ########################################################### */
#include "linux/spinlock.h"
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <uapi/linux/serial_reg.h>

/* ########################################################### */
/* #                    Private declarations                 # */
/* ########################################################### */
#define SERIAL_BUFSIZE 32

struct serial_dev_data {
  void __iomem *regs;
  struct miscdevice miscdev;
  char buf[SERIAL_BUFSIZE];
  unsigned int buf_rd_i;
  unsigned int buf_wr_i;
  wait_queue_head_t waiting_queue;
  spinlock_t registers_lock;
};

static int serial_probe(struct platform_device *pdev);
static int serial_remove(struct platform_device *pdev);
static u32 reg_read(struct serial_dev_data *serial_data, unsigned int offset);
static void reg_write(struct serial_dev_data *serial_data, u32 val,
                      unsigned int offset);
static void init_ti_proc_specific_settings(struct platform_device *pdev);
static void destroy_ti_proc_specific_settings(struct platform_device *pdev);
static int serial_configure_baud_rate(struct platform_device *pdev,
                                      struct serial_dev_data *serial_data);
static void serial_write_char(struct serial_dev_data *serial_data, char c);
static int generate_unique_device_id(struct platform_device *pdev, char **buf);
static int register_interrupt_service_handler(struct platform_device *pdev);
static irqreturn_t interrupt_service_handler(int irq, void *arg);
static ssize_t serial_read(struct file *file, char __user *buf, size_t buf_size,
                           loff_t *buf_offset);
static ssize_t serial_write(struct file *file, const char __user *buf,
                            size_t buf_size, loff_t *buf_offset);

struct file_operations serial_fops = {
    .read = serial_read,
    .write = serial_write,
};

/* ########################################################### */
/* #                    Public API                           # */
/* ########################################################### */
MODULE_LICENSE("GPL");

static const struct of_device_id serial_of_match[] = {
    {.compatible = "bootlin,serial"},
    {},
};
MODULE_DEVICE_TABLE(of, serial_of_match);

static struct platform_driver serial_driver = {
    .driver =
        {
            .name = "serial",
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(serial_of_match),
        },
    .probe = serial_probe,
    .remove = serial_remove,
};
module_platform_driver(serial_driver);

/* ########################################################### */
/* #                    Private API                          # */
/* ########################################################### */
int serial_probe(struct platform_device *pdev) {
  struct serial_dev_data *serial_data;
  char *dev_id;
  int err;

  pr_info("Called %s\n", __func__);

  serial_data = devm_kzalloc(&pdev->dev, sizeof(*serial_data), GFP_KERNEL);
  if (!serial_data) {
    return -ENOMEM;
  }

  init_waitqueue_head(&serial_data->waiting_queue);
  spin_lock_init(&serial_data->registers_lock);

  serial_data->regs = devm_platform_ioremap_resource(pdev, 0);
  if (IS_ERR(serial_data->regs)) {
    return PTR_ERR(serial_data->regs);
  }

  init_ti_proc_specific_settings(pdev);

  err = serial_configure_baud_rate(pdev, serial_data);
  if (err) {
    pr_err("Unable to configure baud rate.");
    goto cleanup;
  }

  platform_set_drvdata(pdev, serial_data);

  err = generate_unique_device_id(pdev, &dev_id);
  if (err) {
    pr_err("Unable to generate unique device id.");
    goto cleanup;
  }

  // Create minor automatically
  serial_data->miscdev.minor = MISC_DYNAMIC_MINOR;
  // Set unique name
  serial_data->miscdev.name = dev_id;
  // Set parent
  serial_data->miscdev.parent = &pdev->dev;
  // Set file operations
  serial_data->miscdev.fops = &serial_fops;

  err = misc_register(&serial_data->miscdev);
  if (err) {
    pr_err("Unable to register device in misc filesystem");
    goto cleanup;
  }

  err = register_interrupt_service_handler(pdev);
  if (err) {
    pr_err("Unable to register interrupt service handler");
    goto cleanup;
  }

  return 0;

cleanup:
  destroy_ti_proc_specific_settings(pdev);
  return err;
}

int serial_remove(struct platform_device *pdev) {
  struct serial_dev_data *serial_data;

  pr_info("Called %s\n", __func__);

  destroy_ti_proc_specific_settings(pdev);

  serial_data = platform_get_drvdata(pdev);
  misc_deregister(&serial_data->miscdev);

  return 0;
}

u32 reg_read(struct serial_dev_data *serial_data, unsigned int offset) {
  return readl(serial_data->regs + (offset * 4));
}

void reg_write(struct serial_dev_data *serial_data, u32 val,
               unsigned int offset) {
  writel(val, serial_data->regs + (offset * 4));
}

void init_ti_proc_specific_settings(struct platform_device *pdev) {
  pm_runtime_enable(&pdev->dev);
  pm_runtime_get_sync(&pdev->dev);
}

void destroy_ti_proc_specific_settings(struct platform_device *pdev) {
  pm_runtime_disable(&pdev->dev);
}

int serial_configure_baud_rate(struct platform_device *pdev,
                               struct serial_dev_data *serial_data) {
  unsigned int uartclk, baud_divisor;
  int err;

  err = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &uartclk);
  if (err) {
    // It is better than pr_err because if many devices uses the same driver,
    // prints where it camed from.
    dev_err(&pdev->dev, "clock-frequency property not found in Device Tree\n");
    return err;
  }

  baud_divisor = uartclk / 16 / 115200;
  reg_write(serial_data, 0x07, UART_OMAP_MDR1);
  reg_write(serial_data, 0x00, UART_LCR);
  reg_write(serial_data, UART_LCR_DLAB, UART_LCR);
  reg_write(serial_data, baud_divisor & 0xff, UART_DLL);
  reg_write(serial_data, (baud_divisor >> 8) & 0xff, UART_DLM);
  reg_write(serial_data, UART_LCR_WLEN8, UART_LCR);
  reg_write(serial_data, 0x00, UART_OMAP_MDR1);

  /* Clear UART FIFOs */
  reg_write(serial_data, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);

  return 0;
}

void serial_write_char(struct serial_dev_data *serial_data, char c) {
  unsigned long flags;
  spin_lock_irqsave(&serial_data->registers_lock, flags);

  while (!(reg_read(serial_data, UART_LSR) & UART_LSR_THRE)) {
    cpu_relax();
  }

  reg_write(serial_data, c, UART_TX);

  spin_unlock_irqrestore(&serial_data->registers_lock, flags);
}

int generate_unique_device_id(struct platform_device *pdev, char **buf) {
  struct resource *res;
  char *unique_id;

  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

  unique_id = devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-%x", res->start);
  if (!unique_id) {
    return -ENOMEM;
  }

  *buf = unique_id;

  return 0;
}

/* Take data from serial device and write it to userspace. */
ssize_t serial_read(struct file *file, char __user *buf, size_t buf_size,
                    loff_t *buf_offset) {
  struct miscdevice *miscdev = file->private_data;
  struct serial_dev_data *serial_data =
      container_of(miscdev, struct serial_dev_data, miscdev);
  int err;

  // We do not need to lock here because write is already locked, if any change
  // occurs it have to be in locked context.
  err =
      wait_event_interruptible(serial_data->waiting_queue,
                               serial_data->buf_rd_i != serial_data->buf_wr_i);
  if (err) {
    pr_err("Unable to wait for interruptible event.");
    return err;
  }

  err = put_user(serial_data->buf[serial_data->buf_rd_i], buf);
  if (err) {
    pr_err("Unable to copy serial data into userspace.");
    return err;
  }

  serial_data->buf_rd_i++;

  if (serial_data->buf_rd_i == SERIAL_BUFSIZE)
    serial_data->buf_rd_i = 0;

  return 1; // return number of characters read
};

/* Take data from userspace and write it to serial device. */
ssize_t serial_write(struct file *file, const char __user *buf, size_t buf_size,
                     loff_t *buf_offset) {
  // This is the same as "serial_data->miscdev".
  struct miscdevice *miscdev = file->private_data;
  struct serial_dev_data *serial_data =
      container_of(miscdev, struct serial_dev_data, miscdev);

  char local_buf;
  int err, i;

  for (i = 0; i < buf_size; i++) {
    err = get_user(local_buf, buf + i);
    if (err) {
      pr_err("Unable to get char from userspace.");
      return -EFAULT;
    }

    serial_write_char(serial_data, local_buf);

    if (local_buf == '\n') {
      // This is required to fix '\n' displaying
      serial_write_char(serial_data, '\r');
    }
  }

  return buf_size;
};

int register_interrupt_service_handler(struct platform_device *pdev) {
  struct serial_dev_data *serial_data;
  int irq, err;

  /* Get private data */
  serial_data = platform_get_drvdata(pdev);

  /* Enable interrupts */
  reg_write(serial_data, UART_IER, UART_IER_RDI);

  /* Request IRQ id */
  irq = platform_get_irq(pdev, 0);

  /* Register IRQ handler */
  err = devm_request_irq(&(pdev->dev), irq, interrupt_service_handler, 0,
                         serial_data->miscdev.name, serial_data);
  if (err) {
    pr_err("Unable to request interrupt service registration.");
    return -ENOMEM;
  }

  return 0;
}

irqreturn_t interrupt_service_handler(int irq, void *arg) {
  struct serial_dev_data *serial_data = arg;
  unsigned long flags;
  u32 buf;

  pr_info("Called %s\n", __func__);

  spin_lock_irqsave(&serial_data->registers_lock, flags);

  buf = reg_read(serial_data, UART_RX);

  serial_data->buf[serial_data->buf_wr_i] = buf;
  serial_data->buf_wr_i++;

  if (serial_data->buf_wr_i == SERIAL_BUFSIZE)
    serial_data->buf_wr_i = 0;

  spin_unlock_irqrestore(&serial_data->registers_lock, flags);

  wake_up(&serial_data->waiting_queue);

  return IRQ_HANDLED;
}
