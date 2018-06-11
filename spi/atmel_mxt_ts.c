/*
 * Solomon Systech maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 * Copyright (C) 2018 Solomon Systech Ltd.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

// uncomment to enable the dev_dbg prints to dmesg
#define DEBUG
// uncomment to test with input forced open
#define INPUT_DEVICE_ALWAYS_OPEN

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

enum mxt_suspend_mode
{
    MXT_SUSPEND_DEEP_SLEEP = 0,
    MXT_SUSPEND_REGULATOR  = 2,
};

#define GPIO_RESET_NO_HIGH  1
#define GPIO_RESET_YES_LOW  0

/* Platform Data (e.g. populated from Device Tree) */
struct mxt_platform_data
{
    u8 t19_gpio_num_keys;
    const unsigned int *t19_gpio_keymap;
    enum mxt_suspend_mode suspend_mode;
    int t15_keyarray_num_keys;
    const unsigned int *t15_keyarray_keymap;
    unsigned long gpio_reset;
    unsigned long gpio_chg_irq;
    const char *cfg_name;
    const char *fw_name;
    const char *input_name;
};

/* Configuration file format version expected*/
#define MXT_CFG_MAGIC       "OBP_RAW V1"

/* Object types */
#define MXT_GEN_MESSAGEPROCESSOR_T5                5
#define MXT_GEN_COMMANDPROCESSOR_T6                6
#define MXT_GEN_POWERCONFIG_T7                     7
#define MXT_GEN_ACQUISITIONCONFIG_T8               8
#define MXT_TOUCH_KEYARRAY_T15                    15
#define MXT_SPT_COMMSCONFIG_T18                   18
#define MXT_SPT_GPIOPWM_T19                       19
#define MXT_PROCI_GRIPFACE_T20                    20
#define MXT_PROCG_NOISE_T22                       22
#define MXT_TOUCH_PROXIMITY_T23                   23
#define MXT_PROCI_ONETOUCH_T24                    24
#define MXT_SPT_SELFTEST_T25                      25
#define MXT_PROCI_TWOTOUCH_T27                    27
#define MXT_SPT_CTECONFIG_T28                     28
#define MXT_SPT_USERDATA_T38                      38
#define MXT_PROCI_GRIP_T40                        40
#define MXT_PROCI_PALM_T41                        41
#define MXT_PROCI_TOUCHSUPPRESSION_T42            42
#define MXT_SPT_DIGITIZER_T43                     43
#define MXT_SPT_MESSAGECOUNT_T44                  44
#define MXT_SPT_CTECONFIG_T46                     46
#define MXT_PROCI_STYLUS_T47                      47
#define MXT_TOUCH_PROXKEY_T52                     52
#define MXT_GEN_DATASOURCE_T53                    53
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71 71
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR_T92      92
#define MXT_PROCI_TOUCHSEQUENCELOGGER_T93         93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100          100
#define MXT_PROCI_ACTIVESTYLUS_T107              107

/* T5 Message Processor */
#define MXT_T5_REPORTID_VAL_NOMSG     0xff

/* T6 Command Processor */
#define MXT_T6_CFG_RESET_OFFSET       0
#define MXT_T6_CFG_BACKUPNV_OFFSET    1
#define MXT_T6_CFG_CALIBRATE_OFFSET   2
#define MXT_T6_CFG_REPORTALL_OFFSET   3
#define MXT_T6_CFG_DIAGNOSTIC_OFFSET  5

#define MXT_T6_CFG_RESET_VAL_TO_BOOTL    0xa5
#define MXT_T6_CFG_RESET_VAL_TO_APP      0x01 // any non zero and non 0xa5
#define MXT_T6_CFG_BACKUPNV_VAL_TO_NVM   0x55
#define MXT_T6_CFG_CALIBRATE_VAL_EXEC    0x01 // any non zero
#define MXT_T6_CFG_REPORTALL_VAL_EXEC    0x01 // any non zero

#define MXT_T6_MSG_STATUS_RESET_BIT     (1 << 7)
#define MXT_T6_MSG_STATUS_OFL_BIT       (1 << 6)
#define MXT_T6_MSG_STATUS_SIGERR_BIT    (1 << 5)
#define MXT_T6_MSG_STATUS_CAL_BIT       (1 << 4)
#define MXT_T6_MSG_STATUS_CFGERR_BIT    (1 << 3)
#define MXT_T6_MSG_STATUS_COMSERR_BIT   (1 << 2)

/* T7 Power Configuration */
struct t7_powerconfig
{
    u8 idle;
    u8 active;
} __packed;

enum t7_powerconfig_type
{
    MXT_T7_POWER_CFG_RUN       = 0,
    MXT_T7_POWER_CFG_DEEPSLEEP = 1,
    MXT_T7_POWER_CFG_POWERSAVE = 2,
    MXT_T7_POWER_CFG_GESTURE   = 3,
};

/* T18 Comms Configuration */
#define MXT_T18_CFG_CTRL_OFFSET               0
#define MXT_T18_CFG_CTRL_RETRIGEN_BIT  (1 << 6)

/* T100 Multiple Touch Touchscreen CFG*/
#define MXT_T100_CFG_CTRL_OFFSET               0
#define MXT_T100_CFG_CTRL_ENABLE_BIT    (1 << 0)
#define MXT_T100_CFG_CTRL_RPTEN_BIT     (1 << 1)
#define MXT_T100_CFG_CTRL_DISSCRMSG_BIT (1 << 2)
#define MXT_T100_CFG_CTRL_SCANEN_BIT    (1 << 7)

#define MXT_T100_CFG_CFG1_OFFSET               1
#define MXT_T100_CFG_CFG1_SWITCHXY_BIT  (1 << 5)

#define MXT_T100_CFG_TCHAUX_OFFSET             3
#define MXT_T100_CFG_TCHAUX_VECT_BIT    (1 << 0)
#define MXT_T100_CFG_TCHAUX_AMPL_BIT    (1 << 1)
#define MXT_T100_CFG_TCHAUX_AREA_BIT    (1 << 2)

#define MXT_T100_CFG_XRANGE_OFFSET     13
#define MXT_T100_CFG_YRANGE_OFFSET     24

/* T100 Multiple Touch Touchscreen MSG*/
#define MXT_T100_MSG_TCHSTATUS_DETECT_BIT     (1 << 7)
#define MXT_T100_MSG_TCHSTATUS_TYPE_MASK          0x70
#define MXT_T100_MSG_TCHSTATUS_TYPE_OFFSET           4

#define MXT_T100_MSG_AUXDATA_OFFSET                  6

enum t100_touch_type
{
    MXT_T100_TOUCH_TYPE_FINGER            = 1,
    MXT_T100_TOUCH_TYPE_PASSIVE_STYLUS    = 2,
    MXT_T100_TOUCH_TYPE_ACTIVE_STYLUS     = 3,
    MXT_T100_TOUCH_TYPE_HOVERING_FINGER   = 4,
    MXT_T100_TOUCH_TYPE_GLOVE             = 5,
    MXT_T100_TOUCH_TYPE_LARGE_TOUCH       = 6,
    MXT_T100_TOUCH_TYPE_EDGE_TOUCH        = 7,
};

#define MXT_TOUCH_DISTANCE_ACTIVE_TOUCH   0
#define MXT_TOUCH_DISTANCE_HOVERING       1

#define MXT_TOUCH_MAJOR_DEFAULT           1
#define MXT_TOUCH_PRESSURE_DEFAULT        1

/* T107 Active Stylus */
#define MXT_T107_CFG_STYAUX_OFFSET           37
#define MXT_T107_CFG_STYAUX_PRS_BIT    (1 << 0)
#define MXT_T107_CFG_STYAUX_XPEAK_BIT  (1 << 6)
#define MXT_T107_CFG_STYAUX_YPEAK_BIT  (1 << 7)

#define MXT_T100_MSG_AUXDATA0_T107_HOVER_BIT      (1 << 0)
#define MXT_T100_MSG_AUXDATA0_T107_TIPSWITCH_BIT  (1 << 1)
#define MXT_T100_MSG_AUXDATA0_T107_BUTTON0_BIT    (1 << 2)
#define MXT_T100_MSG_AUXDATA0_T107_BUTTON1_BIT    (1 << 3)

/* Delay times */
#define MXT_RESET_TIME       200 /* msec */
#define MXT_RESET_TIMEOUT   3000 /* msec */
#define MXT_CRC_TIMEOUT     1000 /* msec */
#define MXT_FW_RESET_TIME   3000 /* msec */
#define MXT_WAKEUP_TIME       25 /* msec */
#define MXT_REGULATOR_DELAY  150 /* msec */
#define MXT_CHG_DELAY        100 /* msec */
#define MXT_POWERON_DELAY    150 /* msec */

/* Command to unlock bootloader */
#define MXT_BOOTL_UNLOCK_CMD_BYTE0     0xdc
#define MXT_BOOTL_UNLOCK_CMD_BYTE1     0xaa

/* Bootloader mode status bits 7 6 */
#define MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD 0xc0
#define MXT_BOOT_STATUS_WAITING_FRAME_DATA   0x80
#define MXT_BOOT_STATUS_WAITINGS_MASK        0x3f

/* Bootloader mode status bits 7..0 */
#define MXT_BOOT_STATUS_FRAME_CRC_CHECK 0x02
#define MXT_BOOT_STATUS_FRAME_CRC_FAIL  0x03
#define MXT_BOOT_STATUS_FRAME_CRC_PASS  0x04

/* Error codes in use from libmaxtouch */
#define MXT_ERROR_BOOTLOADER_UNLOCKED        18
#define MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL  19

#define MXT_APP_CRC_FAIL            0x40 /* valid 7 8 bit only */
#define MXT_BOOT_EXTENDED_ID    (1 << 5)
#define MXT_BOOT_ID_MASK            0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA      0xff

#define MXT_PIXELS_PER_MM   20

#define DEBUG_MSG_MAX      200

#define MXT_T93_ENABLE      1
#define MXT_T93_DISABLE     2

#define MXT_T93_CTRL_ENABLE  (1 << 0)
#define MXT_T93_CTRL_RPTEN   (1 << 1)

// opcodes
#define SPI_WRITE_REQ    0x01
#define SPI_WRITE_OK     0x81
#define SPI_WRITE_FAIL   0x41
#define SPI_READ_REQ     0x02
#define SPI_READ_OK      0x82
#define SPI_READ_FAIL    0x42
#define SPI_INVALID_REQ  0x04
#define SPI_INVALID_CRC  0x08

#define SPI_APP_DATA_MAX_LEN  64
#define SPI_APP_HEADER_LEN     6
// header 6 bytes + Data[]
// 0 opcode
// 1 address LSB
// 2 address MSB
// 3 length LSB
// 4 length MSB
// 5 CRC
// 6+ Data[]

#define SPI_BOOTL_HEADER_LEN   2

#define SPI_APP_BUF_SIZE  (SPI_APP_HEADER_LEN+SPI_APP_DATA_MAX_LEN)

//static const u32 spi_mode32 = SPI_CPHA | SPI_CPOL;
//static const u8 spi_bits_per_word = 8;
static const u32 spi_app_max_speed_hz = 4000000; // 4 MHz
static const u32 spi_bootl_max_speed_hz = 400000; // 400 KHz
static u8 spi_tx_buf[SPI_APP_BUF_SIZE];
static u8 spi_rx_buf[SPI_APP_BUF_SIZE];
static u8 spi_tx_dummy_buf[SPI_APP_BUF_SIZE] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct mxt_info
{
    u8 family_id;
    u8 variant_id;
    u8 version;
    u8 build;
    u8 matrix_xsize;
    u8 matrix_ysize;
    u8 object_num;
};

struct mxt_object
{
    u8 type;
    u16 start_address;
    u8 size_minus_one;
    u8 instances_minus_one;
    u8 num_report_ids;
} __packed;

#define MXT_OBJECT_START     0x07 // after struct mxt_info
#define MXT_INFO_CHECKSUM_SIZE  3 // after list of struct mxt_object

/* Config update context */
struct mxt_cfg
{
    u8 *raw;
    size_t raw_size;
    off_t raw_pos;

    u8 *mem;
    size_t mem_size;
    int start_ofs;

    struct mxt_info mxtinfo;
};

/* Firmware update context */
struct mxt_flash
{
    struct mxt_data *mxtdata;
    const struct firmware *fw;
    u8 *pframe;
    loff_t fw_pos;
    size_t frame_size;
    unsigned int frame_count;
    unsigned int frame_retry;
};

/* Each client has this additional data */
struct mxt_data
{
    struct spi_device *spidevice;
    struct input_dev *inputdev;
    char phys[64];      /* device physical location */
    const struct mxt_platform_data *mxtplatform;
    struct mxt_object *object_table;
    struct mxt_info *mxtinfo;
    void *raw_info_block;
    unsigned int chg_irq;
    unsigned int t100_max_x;
    unsigned int t100_max_y;
    bool t100_xy_switch;
    bool in_bootloader;
    bool force_update_fw;
    u16 mem_size;
    u8 t100_aux_ampl_idx;
    u8 t100_aux_area_idx;
    u8 t100_aux_vect_idx;
    struct bin_attribute mem_access_attr;
    bool debug_enabled;
    bool debug_v2_enabled;
    u8 *debug_msg_data;
    u16 debug_msg_count;
    struct bin_attribute debug_msg_attr;
    struct mutex debug_msg_lock;
    u8 max_reportid;
    u32 config_crc;
    u32 info_crc;
    u8 *msg_buf;
    u8 t6_status;
    bool update_input;
    u8 last_message_count;
    u8 t100_num_touchids;
    struct t7_powerconfig t7_powercfg;
    unsigned long t15_keyarray_keystatus;
    u8 t100_stylus_aux_pressure_idx;
    u8 t100_stylus_aux_xpeak_idx;
    u8 t100_stylus_aux_ypeak_idx;
    bool use_retrigen_workaround;
    u8 double_tap_enable;
    struct regulator *reg_vdd;
    struct regulator *reg_avdd;
    char *fw_name;
    char *cfg_name;
    struct mxt_flash *mxtflash;

    // objects whose configuration address
    // we want to cache rather than frequently
    // retrieve for read/write
    u16 t5_cfg_address;
    u16 t6_cfg_address;
    u16 t7_cfg_address;
    u16 t15_cfg_address;
    u16 t18_cfg_address;
    u16 t44_cfg_address;
    u16 t71_cfg_address;
    u16 t93_cfg_address;
    u16 t100_cfg_address;
    u16 t107_cfg_address;

    // size of the largest possible message
    // NOTE: all messages are shipped via T5
    u8 t5_msg_size;

    // objects that support sending messages
    // whose report_id we want to cache
    u8 t6_msg_reportid;
    u8 t100_msg_reportid_min;
    u8 t100_msg_reportid_max;

    /* for reset handling */
    struct completion reset_completion;

    /* for config update handling */
    struct completion crc_completion;

    /* for power up handling */
    struct completion chg_completion;

    /* Indicates whether device is in suspend */
    bool suspended;

    /* Indicates whether device is updating configuration */
    bool updating_config;
};

static u8 get_crc8_iter(u8 crc, u8 data)
{
    static const u8 crcpoly = 0x8c;
    u8 index = 8;
    u8 fb;
    do
    {
        fb = (crc ^ data) & 0x01;
        data >>= 1;
        crc >>= 1;
        if (fb)
        {
            crc ^= crcpoly;
        }
    } while (--index);
    return crc;
}

static u8 get_header_crc(u8 *p_msg)
{
    u8 calc_crc = 0;
    int i = 0;
    for (; i < SPI_APP_HEADER_LEN-1; i++)
    {
        calc_crc = get_crc8_iter(calc_crc, p_msg[i]);
    }
    return calc_crc;
}

static void spi_prepare_header(u8 *header,
                               u8 opcode,
                               u16 start_register,
                               u16 count)
{
    header[0] = opcode;
    header[1] = start_register & 0xff;
    header[2] = start_register >> 8;
    header[3] = count & 0xff;
    header[4] = count >> 8;
    header[5] = get_header_crc(header);
}

static size_t mxt_get_obj_size(const struct mxt_object *obj)
{
    return obj->size_minus_one + 1;
}

static size_t mxt_get_obj_num_instances(const struct mxt_object *obj)
{
    return obj->instances_minus_one + 1;
}

static void mxt_debug_msg_enable(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;

    dev_dbg(dev, "%s >\n", __func__);

    if (mxtdata->debug_v2_enabled)
    {
        return;
    }

    mutex_lock(&mxtdata->debug_msg_lock);

    mxtdata->debug_msg_data = kcalloc(DEBUG_MSG_MAX, mxtdata->t5_msg_size, GFP_KERNEL);
    if (mxtdata->debug_msg_data)
    {
        mxtdata->debug_v2_enabled = true;
        dev_dbg(dev, "Enabled message output\n");
    }
    else
    {
        dev_err(dev, "debug_msg_data = kcalloc\n");
    }

    mutex_unlock(&mxtdata->debug_msg_lock);
}

static void mxt_debug_msg_disable(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;

    dev_dbg(dev, "%s >\n", __func__);

    if (!mxtdata->debug_v2_enabled)
    {
        return;
    }

    mxtdata->debug_v2_enabled = false;

    mutex_lock(&mxtdata->debug_msg_lock);
    kfree(mxtdata->debug_msg_data);
    mxtdata->debug_msg_data = NULL;
    mxtdata->debug_msg_count = 0;
    mutex_unlock(&mxtdata->debug_msg_lock);
    dev_dbg(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *mxtdata, u8 *msg)
{
    struct device *dev = &mxtdata->spidevice->dev;

    dev_dbg(dev, "%s >\n", __func__);

    mutex_lock(&mxtdata->debug_msg_lock);

    if (!mxtdata->debug_msg_data)
    {
        mutex_unlock(&mxtdata->debug_msg_lock);
        dev_err(dev, "No buffer!\n");
        return;
    }

    if (mxtdata->debug_msg_count < DEBUG_MSG_MAX)
    {
        memcpy(mxtdata->debug_msg_data +
               mxtdata->debug_msg_count * mxtdata->t5_msg_size,
               msg,
               mxtdata->t5_msg_size);
        mxtdata->debug_msg_count++;
    }
    else
    {
        dev_dbg(dev, "Discarding %u messages\n", mxtdata->debug_msg_count);
        mxtdata->debug_msg_count = 0;
    }

    mutex_unlock(&mxtdata->debug_msg_lock);

    sysfs_notify(&mxtdata->spidevice->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_sysfs_debug_msg_write(struct file *filp,
                                         struct kobject *kobj,
                                         struct bin_attribute *bin_attr,
                                         char *buf,
                                         loff_t off,
                                         size_t count)
{
    return -EIO;
}

static ssize_t mxt_sysfs_debug_msg_read(struct file *filp,
                                        struct kobject *kobj,
                                        struct bin_attribute *bin_attr,
                                        char *buf,
                                        loff_t off,
                                        size_t bytes)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    int count;
    size_t bytes_read;

    if (!mxtdata->debug_msg_data)
    {
        dev_err(dev, "No buffer!\n");
        return 0;
    }

    count = bytes / mxtdata->t5_msg_size;

    if (count > DEBUG_MSG_MAX)
    {
        count = DEBUG_MSG_MAX;
    }

    mutex_lock(&mxtdata->debug_msg_lock);

    if (count > mxtdata->debug_msg_count)
    {
        count = mxtdata->debug_msg_count;
    }

    bytes_read = count * mxtdata->t5_msg_size;

    memcpy(buf, mxtdata->debug_msg_data, bytes_read);
    mxtdata->debug_msg_count = 0;

    mutex_unlock(&mxtdata->debug_msg_lock);

    return bytes_read;
}

static int mxt_sysfs_debug_msg_init(struct mxt_data *mxtdata)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    sysfs_bin_attr_init(&mxtdata->debug_msg_attr);
    mxtdata->debug_msg_attr.attr.name = "debug_msg";
    mxtdata->debug_msg_attr.attr.mode = 0666;
    mxtdata->debug_msg_attr.read = mxt_sysfs_debug_msg_read;
    mxtdata->debug_msg_attr.write = mxt_sysfs_debug_msg_write;
    mxtdata->debug_msg_attr.size = mxtdata->t5_msg_size * DEBUG_MSG_MAX;

    if (sysfs_create_bin_file(&mxtdata->spidevice->dev.kobj, &mxtdata->debug_msg_attr) < 0)
    {
        dev_err(&mxtdata->spidevice->dev, "Failed to create %s\n", mxtdata->debug_msg_attr.attr.name);
        return -EINVAL;
    }
    return 0;
}

static void mxt_sysfs_debug_msg_remove(struct mxt_data *mxtdata)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (mxtdata->debug_msg_attr.attr.name)
    {
        sysfs_remove_bin_file(&mxtdata->spidevice->dev.kobj, &mxtdata->debug_msg_attr);
    }
}

static int mxt_check_mem_access_params(struct mxt_data *mxtdata,
                                       loff_t off,
                                       size_t *count)
{
    if (off >= mxtdata->mem_size)
    {
        return -EIO;
    }

    if (off + *count > mxtdata->mem_size)
    {
        *count = mxtdata->mem_size - off;
    }

    return 0;
}

static u8 mxt_read_chg(struct mxt_data *mxtdata)
{
    u8 ret_val = (u8)gpio_get_value(mxtdata->mxtplatform->gpio_chg_irq);
    return ret_val;
}

static int mxt_wait_for_chg(struct mxt_data *mxtdata)
{
    int timeout_counter = 2500; // 50 msec

    while (0 != mxt_read_chg(mxtdata) && timeout_counter > 0)
    {
        timeout_counter--;
        udelay(20);
    }

    return timeout_counter > 0 ? 0 : -1;
}

static int __mxt_read_reg(struct mxt_data *mxtdata, u16 start_register, u16 count, void *val)
{
    u8 attempt = 0;
    int ret_val;
    struct spi_message  spimsg[2];
    struct spi_transfer spitr;
    do
    {
        attempt++;
        if (attempt > 1)
        {
            if (attempt > 3)
            {
                dev_err(&mxtdata->spidevice->dev, "Too many Retries\n");
                return -EIO;
            }
            dev_warn(&mxtdata->spidevice->dev, "Retry %d after CRC fail\n", attempt-1);
            msleep(MXT_WAKEUP_TIME);
        }
        /* WRITE SPI_READ_REQ */
        spi_prepare_header(spi_tx_buf, SPI_READ_REQ, start_register, count);
        spi_message_init(&spimsg[0]);
        memset(&spitr, 0,  sizeof(spitr));
        spitr.tx_buf = spi_tx_buf;
        spitr.rx_buf = spi_rx_buf;
        spitr.len = SPI_APP_HEADER_LEN;
        spi_message_add_tail(&spitr, &spimsg[0]);
        ret_val = spi_sync(mxtdata->spidevice, &spimsg[0]);
        if (ret_val < 0)
        {
            dev_err(&mxtdata->spidevice->dev, "Error writing to spi (%d)", ret_val);
            return ret_val;
        }

        if (0 != mxt_wait_for_chg(mxtdata))
        {
            dev_err(&mxtdata->spidevice->dev, "Timeout on CHG");
        }

        /* READ SPI_READ_OK */
        spi_message_init(&spimsg[1]);
        memset(&spitr, 0,  sizeof(spitr));
        spitr.tx_buf = spi_tx_dummy_buf;
        spitr.rx_buf = spi_rx_buf;
        spitr.len = SPI_APP_HEADER_LEN + count;
        spi_message_add_tail(&spitr, &spimsg[1]);
        ret_val = spi_sync(mxtdata->spidevice, &spimsg[1]);
        if (ret_val < 0)
        {
            dev_err(&mxtdata->spidevice->dev, "Error reading from spi (%d)", ret_val);
            return ret_val;
        }
        if (SPI_READ_OK != spi_rx_buf[0])
        {
            dev_err(&mxtdata->spidevice->dev, "SPI_READ_OK != %.2X reading from spi", spi_rx_buf[0]);
            return -1;
        }
        if (spi_tx_buf[1] != spi_rx_buf[1] || spi_tx_buf[2] != spi_rx_buf[2])
        {
            dev_err(&mxtdata->spidevice->dev, "Unexpected address %d != %d reading from spi", spi_rx_buf[1] | (spi_rx_buf[2] << 8), start_register);
            return -1;
        }
        if (spi_tx_buf[3] != spi_rx_buf[3] || spi_tx_buf[4] != spi_rx_buf[4])
        {
            dev_err(&mxtdata->spidevice->dev, "Unexpected count %d != %d reading from spi", spi_rx_buf[3] | (spi_rx_buf[4] << 8), count);
            return -1;
        }
    }
    while (get_header_crc(spi_rx_buf) != spi_rx_buf[SPI_APP_HEADER_LEN-1]);

    memcpy(val, spi_rx_buf + SPI_APP_HEADER_LEN, count);
    return 0;
}

static int mxt_read_blks(struct mxt_data *mxtdata, u16 start, u16 count, u8 *buf)
{
    u16 offset = 0;
    int ret_val;
    u16 size;

    while (offset < count)
    {
        size = min(SPI_APP_DATA_MAX_LEN, count - offset);

        ret_val = __mxt_read_reg(mxtdata,
                                 start + offset,
                                 size,
                                 buf + offset);
        if (ret_val)
        {
            return ret_val;
        }

        offset += size;
    }

    return 0;
}

static int __mxt_write_reg(struct mxt_data *mxtdata, u16 reg, u16 count, const void *val)
{
    u8 attempt = 0;
    int ret_val;
    struct spi_message  spimsg[2];
    struct spi_transfer spitr;
    do
    {
        attempt++;
        if (attempt > 1)
        {
            if (attempt > 3)
            {
                dev_err(&mxtdata->spidevice->dev, "Too many Retries\n");
                return MXT_ERROR_IO;
            }
            dev_warn(&mxtdata->spidevice->dev, "Retry %d after CRC fail\n", attempt-1);
            msleep(MXT_WAKEUP_TIME);
        }
        /* WRITE SPI_WRITE_REQ */
        spi_prepare_header(spi_tx_buf, SPI_WRITE_REQ, address_iter, count);
        memcpy(spi_tx_buf + SPI_APP_HEADER_LEN, val, count);
        spi_message_init(&spimsg[0]);
        memset(&spitr, 0,  sizeof(spitr));
        spitr.tx_buf = spi_tx_buf;
        spitr.rx_buf = spi_rx_buf;
        spitr.len = SPI_APP_HEADER_LEN + count;
        spi_message_add_tail(&spitr, &spimsg[0]);
        ret_val = spi_sync(mxtdata->spidevice, &spimsg[0]);
        if (ret_val < 0)
        {
            dev_err(&mxtdata->spidevice->dev, "Error writing to spi (%d)", ret_val);
            return ret_val;
        }

        if (0 != mxt_wait_for_chg(mxtdata))
        {
            dev_err(&mxtdata->spidevice->dev, "Timeout on CHG");
        }

        /* READ SPI_WRITE_OK */
        spi_message_init(&spimsg[1]);
        memset(&spitr, 0,  sizeof(spitr));
        spitr.tx_buf = spi_tx_dummy_buf;
        spitr.rx_buf = spi_rx_buf;
        spitr.len = SPI_APP_HEADER_LEN;
        spi_message_add_tail(&spitr, &spimsg[1]);
        ret_val = spi_sync(mxtdata->spidevice, &spimsg[1]);
        if (ret_val < 0)
        {
            dev_err(&mxtdata->spidevice->dev, "Error reading from spi (%d)", ret_val);
            return ret_val;
        }
        if (SPI_WRITE_OK != spi_rx_buf[0])
        {
            dev_err(&mxtdata->spidevice->dev, "SPI_WRITE_OK != %.2X reading from spi", spi_rx_buf[0]);
            return -1;
        }
        if (spi_tx_buf[1] != spi_rx_buf[1] || spi_tx_buf[2] != spi_rx_buf[2])
        {
            dev_err(&mxtdata->spidevice->dev, "Unexpected address %d != %d reading from spi", spi_rx_buf[1] | (spi_rx_buf[2] << 8), address_iter);
            return -1;
        }
        if (spi_tx_buf[3] != spi_rx_buf[3] || spi_tx_buf[4] != spi_rx_buf[4])
        {
            dev_err(&mxtdata->spidevice->dev, "Unexpected count %d != %d reading from spi", spi_rx_buf[3] | (spi_rx_buf[4] << 8), count);
            return -1;
        }
    }
    while (get_header_crc(spi_rx_buf) != spi_rx_buf[SPI_APP_HEADER_LEN-1]);
    return 0;
}

static int mxt_write_blks(struct mxt_data *mxtdata, u16 start, u16 count, u8 *buf)
{
    u16 offset = 0;
    int ret_val;
    u16 size;

    while (offset < count)
    {
        size = min(SPI_APP_DATA_MAX_LEN, count - offset);

        ret_val = __mxt_write_reg(mxtdata,
                                  start + offset,
                                  size,
                                  buf + offset);
        if (ret_val)
        {
            return ret_val;
        }

        offset += size;
    }

    return 0;
}

static ssize_t mxt_sysfs_mem_access_read(struct file *filp,
                                         struct kobject *kobj,
                                         struct bin_attribute *bin_attr,
                                         char *buf,
                                         loff_t off,
                                         size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    int ret_val = 0;

    ret_val = mxt_check_mem_access_params(mxtdata, off, &count);
    if (ret_val < 0)
    {
        return ret_val;
    }

    if (count > 0)
    {
        ret_val = mxt_read_blks(mxtdata, off, count, buf);
    }

    return ret_val == 0 ? count : ret_val;
}

static ssize_t mxt_sysfs_mem_access_write(struct file *filp,
                                          struct kobject *kobj,
                                          struct bin_attribute *bin_attr,
                                          char *buf,
                                          loff_t off,
                                          size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    int ret_val = 0;

    ret_val = mxt_check_mem_access_params(mxtdata, off, &count);
    if (ret_val < 0)
    {
        return ret_val;
    }

    if (count > 0)
    {
        ret_val = mxt_write_blks(mxtdata, off, count, buf);
    }

    return ret_val == 0 ? count : ret_val;
}

static int mxt_wait_for_completion(struct mxt_data *mxtdata,
                                   struct completion *comp,
                                   unsigned int timeout_ms,
                                   const char *dbg_str)
{
    struct device *dev = &mxtdata->spidevice->dev;
    unsigned long timeout = msecs_to_jiffies(timeout_ms);
    long ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s %s >\n", __func__, dbg_str);

    ret_val = wait_for_completion_interruptible_timeout(comp, timeout);
    if (ret_val < 0)
    {
        return ret_val;
    }
    if (ret_val == 0)
    {
        dev_err(dev, "wait_for_completion timeout %s\n", dbg_str);
        return -ETIMEDOUT;
    }
    return 0;
}

static int mxt_bootloader_read(struct mxt_data *mxtdata,
                               u8 *val,
                               unsigned int count)
{
    int ret_val, retry;
    struct i2c_msg i2cmsg;

    i2cmsg.flags = I2C_M_RD; // I2C_M_TEN not set, 7 bit address
    i2cmsg.len = count;
    i2cmsg.buf = val;

    for (retry = 0; retry < 2; retry++)
    {
        if (retry > 0)
        {
            dev_err(&mxtdata->spidevice->dev, "%s: retry %d\n", __func__, retry);
            msleep(30);
        }
        ret_val = i2c_transfer(mxtdata->spidevice->adapter, &i2cmsg, 1);
        if (1 == ret_val)
        {
            ret_val = 0;
            break;
        }
        dev_err(&mxtdata->spidevice->dev, "%s: i2c_transfer ret_val %d\n", __func__, ret_val);
        ret_val = ret_val < 0 ? ret_val : -EIO;
    }
    return ret_val;
}

static int mxt_bootloader_write(struct mxt_data *mxtdata,
                                const u8 * const val,
                                unsigned int count)
{
    int ret_val, retry;
    struct i2c_msg i2cmsg;

    i2cmsg.flags = 0; // I2C_M_TEN not set, 7 bit address
    i2cmsg.len = count;
    i2cmsg.buf = (u8 *)val;

    for (retry = 0; retry < 2; retry++)
    {
        if (retry > 0)
        {
            dev_err(&mxtdata->spidevice->dev, "%s: retry %d\n", __func__, retry);
            msleep(30);
        }
        ret_val = i2c_transfer(mxtdata->spidevice->adapter, &i2cmsg, 1);
        if (1 == ret_val)
        {
            ret_val = 0;
            break;
        }
        dev_err(&mxtdata->spidevice->dev, "%s: i2c_transfer ret_val %d\n", __func__, ret_val);
        ret_val = ret_val < 0 ? ret_val : -EIO;
    }
    return ret_val;
}

static int mxt_probe_bootloader(struct mxt_data *mxtdata, bool alt_address)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    u8 buf[3];
    bool is_crc_failure, is_extended_id;

    dev_dbg(dev, "%s >\n", __func__);

    /* Check bootloader status and version information */
    ret_val = mxt_bootloader_read(mxtdata, buf, sizeof(buf));
    dev_info(dev, "%s, %d, buf[0] = %x, buf[1] = %x, buf[2] = %x, error = %d\n",  __func__, __LINE__, buf[0], buf[1], buf[2], ret_val);
    if (ret_val)
    {
        return ret_val;
    }

    is_crc_failure = (buf[0] & ~MXT_BOOT_STATUS_WAITINGS_MASK) == MXT_APP_CRC_FAIL;
    is_extended_id = buf[0] & MXT_BOOT_EXTENDED_ID;

    dev_info(dev, "Found bootloader ID:%u%s%u%s\n",
             is_extended_id ? (buf[1] & MXT_BOOT_ID_MASK) : buf[0],
             is_extended_id ? " version:" : "",
             is_extended_id ? buf[2] : 0,
             is_crc_failure ? ", APP_CRC_FAIL" : "");

    return 0;
}

static int mxt_send_bootloader_reset_cmd(struct mxt_data *mxtdata)
{
    u8 buf[2] = {0x01, 0x01};

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    return mxt_bootloader_write(mxtdata, buf, 2);
}

static int mxt_send_bootloader_unlock_cmd(struct mxt_data *mxtdata)
{
    u8 buf[2] = {MXT_BOOTL_UNLOCK_CMD_BYTE0, MXT_BOOTL_UNLOCK_CMD_BYTE1};

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    return mxt_bootloader_write(mxtdata, buf, 2);
}

static struct mxt_object * mxt_get_object(struct mxt_data *mxtdata, u8 type)
{
    struct mxt_object *object;
    int i;

    for (i = 0; i < mxtdata->mxtinfo->object_num; i++)
    {
        object = mxtdata->object_table + i;
        if (object->type == type)
        {
            return object;
        }
    }

    dev_warn(&mxtdata->spidevice->dev, "Invalid object type T%u\n", type);
    return NULL;
}

static void mxt_recv_t19_gpio_msg(struct mxt_data *mxtdata, u8 *message)
{
    const struct mxt_platform_data *mxtplatform = mxtdata->mxtplatform;
    int i;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    for (i = 0; i < mxtplatform->t19_gpio_num_keys; i++)
    {
        if (mxtplatform->t19_gpio_keymap[i] != KEY_RESERVED)
        {
            /* Active-low switch */
            input_report_key(mxtdata->inputdev, mxtplatform->t19_gpio_keymap[i], 0 == (message[1] & (1 << i)));
        }
    }
}

static void mxt_input_sync(struct mxt_data *mxtdata)
{
    //dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (mxtdata->inputdev)
    {
        if (mxtdata->mxtplatform->t19_gpio_num_keys)
        {
            input_mt_report_pointer_emulation(mxtdata->inputdev, mxtdata->mxtplatform->t19_gpio_num_keys);
        }
        input_sync(mxtdata->inputdev);
    }
}

static void mxt_recv_t6_command_processor_msg(struct mxt_data *mxtdata, u8 *msg)
{
    struct device *dev = &mxtdata->spidevice->dev;
    u8 status = msg[1];
    u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

    if (crc != mxtdata->config_crc)
    {
        mxtdata->config_crc = crc;
        dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
    }

    complete(&mxtdata->crc_completion);

    /* Detect reset */
    if (status & MXT_T6_MSG_STATUS_RESET_BIT)
    {
        complete(&mxtdata->reset_completion);
    }

    if (status != mxtdata->t6_status)
    {
        /* New status */
        dev_info(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
                 status,
                 status == 0 ? " OK" : "",
                 status & MXT_T6_MSG_STATUS_RESET_BIT ? " RESET" : "",
                 status & MXT_T6_MSG_STATUS_OFL_BIT ? " OFL" : "",
                 status & MXT_T6_MSG_STATUS_SIGERR_BIT ? " SIGERR" : "",
                 status & MXT_T6_MSG_STATUS_CAL_BIT ? " CAL" : "",
                 status & MXT_T6_MSG_STATUS_CFGERR_BIT ? " CFGERR" : "",
                 status & MXT_T6_MSG_STATUS_COMSERR_BIT ? " COMSERR" : "");

        mxtdata->t6_status = status;
    }
}

static void mxt_recv_t15_key_array_msg(struct mxt_data *mxtdata, u8 *msg)
{
    int key;
    bool curr_state, new_state;
    bool sync = false;
    unsigned long keystates = le32_to_cpu(msg[2]);

    for (key = 0; key < mxtdata->mxtplatform->t15_keyarray_num_keys; key++)
    {
        curr_state = test_bit(key, &mxtdata->t15_keyarray_keystatus);
        new_state = test_bit(key, &keystates);

        if (!curr_state && new_state)
        {
            dev_dbg(&mxtdata->spidevice->dev, "T15 key press: %u\n", key);
            __set_bit(key, &mxtdata->t15_keyarray_keystatus);
            input_event(mxtdata->inputdev, EV_KEY, mxtdata->mxtplatform->t15_keyarray_keymap[key], 1);
            sync = true;
        }
        else if (curr_state && !new_state)
        {
            dev_dbg(&mxtdata->spidevice->dev, "T15 key release: %u\n", key);
            __clear_bit(key, &mxtdata->t15_keyarray_keystatus);
            input_event(mxtdata->inputdev, EV_KEY, mxtdata->mxtplatform->t15_keyarray_keymap[key], 0);
            sync = true;
        }
    }

    if (sync)
    {
        input_sync(mxtdata->inputdev);
    }
}

static void mxt_recv_t93_touch_sequence_msg(struct mxt_data *mxtdata, u8 *msg)
{
    dev_dbg(&mxtdata->spidevice->dev, "T93 double tap %d\n", msg[1]);

    input_report_key(mxtdata->inputdev, KEY_WAKEUP, 1);
    input_sync(mxtdata->inputdev);
    input_report_key(mxtdata->inputdev, KEY_WAKEUP, 0);
    input_sync(mxtdata->inputdev);
}

#ifdef DEBUG
static const char *get_t100_touch_type_str(u8 touch_type)
{
    switch (touch_type)
    {
        case MXT_T100_TOUCH_TYPE_FINGER:
            return "Finger";
        case MXT_T100_TOUCH_TYPE_PASSIVE_STYLUS:
            return "Passive Stylus";
        case MXT_T100_TOUCH_TYPE_ACTIVE_STYLUS:
            return "Active Stylus";
        case MXT_T100_TOUCH_TYPE_HOVERING_FINGER:
            return "Hovering Finger";
        case MXT_T100_TOUCH_TYPE_GLOVE:
            return "Glove";
        case MXT_T100_TOUCH_TYPE_LARGE_TOUCH:
            return "Large Touch";
        case MXT_T100_TOUCH_TYPE_EDGE_TOUCH:
            return "Edge Touch";
        default:
            break;
    }
    return "???";
}
#endif

static void mxt_recv_t100_multiple_touch_msg(struct mxt_data *mxtdata, u8 *message)
{
    struct device *dev = &mxtdata->spidevice->dev;
    struct input_dev *inputdev = mxtdata->inputdev;
    int touch_id_slot;
    u8 tchstatus_byte;
    u8 touch_mxt_type = 0;
    u16 x, y;
    int touch_distance = 0;
    int touch_tool_type = 0;
    u8 touch_contact_major_axis = 0;
    u8 touch_pressure = 0;
    u8 touch_orientation = 0; // 0 -> along Y, 1 -> along X
    bool is_active = false;
    bool is_hover = false;

    // First Report ID is Screen Status Messages
    // Second Report ID is Reserved
    // Subsequent Report IDs are Touch Status Messages
    touch_id_slot = message[0] - mxtdata->t100_msg_reportid_min - 2;
    if (touch_id_slot < 0)
    {
        return;
    }

    tchstatus_byte = message[1];
    x = message[2] | message[3] << 8; // little endian 16
    y = message[4] | message[5] << 8; // little endian 16

    if (tchstatus_byte & MXT_T100_MSG_TCHSTATUS_DETECT_BIT)
    {
        touch_mxt_type = (tchstatus_byte & MXT_T100_MSG_TCHSTATUS_TYPE_MASK) >> MXT_T100_MSG_TCHSTATUS_TYPE_OFFSET;

        switch (touch_mxt_type)
        {
            case MXT_T100_TOUCH_TYPE_HOVERING_FINGER:
                touch_tool_type = MT_TOOL_FINGER;
                touch_distance = MXT_TOUCH_DISTANCE_HOVERING;
                is_hover = true;
                is_active = true;
                break;

            case MXT_T100_TOUCH_TYPE_FINGER:
            case MXT_T100_TOUCH_TYPE_GLOVE:
                touch_tool_type = MT_TOOL_FINGER;
                touch_distance = MXT_TOUCH_DISTANCE_ACTIVE_TOUCH;
                is_hover = false;
                is_active = true;

                if (mxtdata->t100_aux_area_idx)
                {
                    touch_contact_major_axis = message[mxtdata->t100_aux_area_idx];
                }
                if (mxtdata->t100_aux_ampl_idx)
                {
                    touch_pressure = message[mxtdata->t100_aux_ampl_idx];
                }
                if (mxtdata->t100_aux_vect_idx)
                {
                    touch_orientation = message[mxtdata->t100_aux_vect_idx];
                }
                break;

            case MXT_T100_TOUCH_TYPE_PASSIVE_STYLUS:
                touch_tool_type = MT_TOOL_PEN;
                touch_distance = MXT_TOUCH_DISTANCE_ACTIVE_TOUCH;
                is_hover = false;
                is_active = true;

                /*
                 * Passive stylus is reported with size zero so
                 * hardcode.
                 */
                touch_contact_major_axis = MXT_TOUCH_MAJOR_DEFAULT;

                if (mxtdata->t100_aux_ampl_idx)
                {
                    touch_pressure = message[mxtdata->t100_aux_ampl_idx];
                }
                break;

            case MXT_T100_TOUCH_TYPE_ACTIVE_STYLUS:
            {
                u8 auxdata0_byte = message[MXT_T100_MSG_AUXDATA_OFFSET];

                /* Report input buttons */
                input_report_key(inputdev, BTN_STYLUS, auxdata0_byte & MXT_T100_MSG_AUXDATA0_T107_BUTTON0_BIT);
                input_report_key(inputdev, BTN_STYLUS2, auxdata0_byte & MXT_T100_MSG_AUXDATA0_T107_BUTTON1_BIT);

                /* stylus in range, but position unavailable */
                if (0 == (auxdata0_byte & MXT_T100_MSG_AUXDATA0_T107_HOVER_BIT))
                {
                    break;
                }

                touch_tool_type = MT_TOOL_PEN;
                touch_distance = MXT_TOUCH_DISTANCE_ACTIVE_TOUCH;
                is_active = true;
                touch_contact_major_axis = MXT_TOUCH_MAJOR_DEFAULT;

                if (0 == (auxdata0_byte & MXT_T100_MSG_AUXDATA0_T107_TIPSWITCH_BIT))
                {
                    is_hover = true;
                    touch_distance = MXT_TOUCH_DISTANCE_HOVERING;
                }
                else if (mxtdata->t100_stylus_aux_pressure_idx)
                {
                    touch_pressure = message[mxtdata->t100_stylus_aux_pressure_idx];
                }
                break;
            }
            case MXT_T100_TOUCH_TYPE_LARGE_TOUCH:
                dev_dbg(dev, "Ignored Suppressed Large Touch\n");
                break;

            case MXT_T100_TOUCH_TYPE_EDGE_TOUCH:
                dev_err(dev, "T100 Edge Touch Not Managed Yet\n");
                break;

            default:
                dev_err(dev, "T100 Unexpected touch_mxt_type %d\n", touch_mxt_type);
                return;
        }
    }

    /*
     * Values reported should be non-zero if tool is touching the
     * device
     */
    if (0 == touch_pressure && !is_hover)
    {
        touch_pressure = MXT_TOUCH_PRESSURE_DEFAULT;
    }

    /*
     * https://www.kernel.org/doc/Documentation/input/multi-touch-protocol.txt
     */
    input_mt_slot(inputdev, touch_id_slot);

    if (is_active)
    {
        dev_dbg(dev,
                "[%u] %s (%u, %u) a:%d p:%d o:%d\n",
                touch_id_slot,
                get_t100_touch_type_str(touch_mxt_type),
                x, y, touch_contact_major_axis, touch_pressure, touch_orientation);

        input_mt_report_slot_state(inputdev, touch_tool_type, true);
        input_report_abs(inputdev, ABS_MT_POSITION_X, x);
        input_report_abs(inputdev, ABS_MT_POSITION_Y, y);
        input_report_abs(inputdev, ABS_MT_TOUCH_MAJOR, touch_contact_major_axis);
        input_report_abs(inputdev, ABS_MT_PRESSURE, touch_pressure);
        input_report_abs(inputdev, ABS_MT_DISTANCE, touch_distance);
        input_report_abs(inputdev, ABS_MT_ORIENTATION, touch_orientation);
    }
    else
    {
        dev_dbg(dev, "[%u] Release\n", touch_id_slot);

        /* close out slot */
        input_mt_report_slot_state(inputdev, 0, false);
    }

    mxtdata->update_input = true;
}

static int mxt_get_obj_type_from_report_id(struct mxt_data *mxtdata, u8 report_id)
{
    int ret_val = 0;
    if (report_id >= mxtdata->t100_msg_reportid_min &&
        report_id <= mxtdata->t100_msg_reportid_max)
    {
        ret_val = 100;
    }
    else if (report_id == mxtdata->t6_msg_reportid)
    {
        ret_val = 6;
    }
    else
    {
        /* Valid Report IDs start counting from 1 */
        u8 curr_rep_id = 1;
        int i;
        for (i = 0; i < mxtdata->mxtinfo->object_num; i++)
        {
            struct mxt_object *object = mxtdata->object_table + i;
            if (object->num_report_ids)
            {
                u8 max_rep_id;
                u8 min_rep_id = curr_rep_id;
                curr_rep_id += object->num_report_ids * mxt_get_obj_num_instances(object);
                max_rep_id = curr_rep_id - 1;
                if (report_id >= min_rep_id && report_id <= max_rep_id)
                {
                    ret_val = object->type;
                    break;
                }
            }
        }
    }
    return ret_val;
}

static int mxt_proc_message(struct mxt_data *mxtdata, u8 *message)
{
    u8 report_id = message[0];
    int object_number;
    bool dump = mxtdata->debug_enabled;
    bool msg_discarded = false;

    if (MXT_T5_REPORTID_VAL_NOMSG == report_id)
    {
        return 0;
    }

    object_number = mxt_get_obj_type_from_report_id(mxtdata, report_id);

    if (0 == object_number)
    {
        msg_discarded = true;
        dev_err(&mxtdata->spidevice->dev, "-> Rid %d unhandled\n", report_id);
        dump = true;
    }
    else
    {
        if (6 == object_number)
        {
            mxt_recv_t6_command_processor_msg(mxtdata, message);
        }
        else if (!mxtdata->inputdev)
        {
            /*
             * Do not report events if input device is
             * not yet registered
             */
            msg_discarded = true;
        }
        else if (93 == object_number)
        {
            mxt_recv_t93_touch_sequence_msg(mxtdata, message);
        }
        else if (mxtdata->suspended)
        {
            /*
             * Do not report events if input device is
             * returning from suspend
             */
            msg_discarded = true;
        }
        else if (100 == object_number)
        {
            mxt_recv_t100_multiple_touch_msg(mxtdata, message);
        }
        else if (15 == object_number)
        {
            mxt_recv_t15_key_array_msg(mxtdata, message);
        }
        else if (19 == object_number)
        {
            mxt_recv_t19_gpio_msg(mxtdata, message);
            mxtdata->update_input = true;
        }
        else if (92 == object_number)
        {
            dev_dbg(&mxtdata->spidevice->dev, "T92 long stroke LSTR=%d STRTYPE=%d\n",
                message[1] & 0x80 ? 1 : 0,
                message[1] & 0x0F);
        }
        else
        {
            msg_discarded = true;
            dev_warn(&mxtdata->spidevice->dev, "T%d unhandled\n", object_number);
            dump = true;
        }
    }

    if (dump)
    {
        dev_dbg(&mxtdata->spidevice->dev, "%c%d: %*ph <\n", msg_discarded ? 't':'T', object_number, mxtdata->t5_msg_size, message);
    }
    else
    {
        dev_dbg(&mxtdata->spidevice->dev, "msg %c%d <\n", msg_discarded ? 't':'T', object_number);
    }

    if (mxtdata->debug_v2_enabled)
    {
        mxt_debug_msg_add(mxtdata, message);
    }

    return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *mxtdata, u8 count)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    int i;
    u8 num_valid = 0;

    dev_dbg(dev, "%s %d >\n", __func__, count);

    /* Safety check for msg_buf */
    if (count > mxtdata->max_reportid)
    {
        return -EINVAL;
    }

    /* Process remaining messages if necessary */
    ret_val = mxt_read_blks(mxtdata,
                            mxtdata->t5_cfg_address,
                            mxtdata->t5_msg_size * count,
                            mxtdata->msg_buf);
    if (ret_val)
    {
        dev_err(dev, "Failed to read %u messages (%d)\n", count, ret_val);
        return ret_val;
    }

    for (i = 0;  i < count; i++)
    {
        ret_val = mxt_proc_message(mxtdata, mxtdata->msg_buf + mxtdata->t5_msg_size * i);

        if (ret_val == 1)
        {
            num_valid++;
        }
    }

    dev_dbg(dev, "%s %d <\n", __func__, num_valid);

    /* return number of messages read */
    return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    u8 count, num_left;

    //dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    /* Read T44 and T5 together */
    ret_val = mxt_read_blks(mxtdata,
                            mxtdata->t44_cfg_address,
                            1 + mxtdata->t5_msg_size,
                            mxtdata->msg_buf);
    if (ret_val)
    {
        dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret_val);
        return IRQ_NONE;
    }

    /* First byte is T44, the number of messages in the buffer */
    count = mxtdata->msg_buf[0];

    /*
     * This condition may be caused by the CHG line being configured in
     * Mode 0. It results in unnecessary operations but it is benign.
     */
    if (0 == count)
    {
        //dev_dbg(&mxtdata->spidevice->dev, "? T44 no msgs\n");
        return IRQ_NONE;
    }

    if (count > mxtdata->max_reportid)
    {
        dev_warn(dev, "T44 count %d exceeded max report id\n", count);
        count = mxtdata->max_reportid;
    }

    /* Process first message */
    ret_val = mxt_proc_message(mxtdata, mxtdata->msg_buf + 1);
    if (ret_val < 0)
    {
        dev_warn(dev, "Unexpected invalid message\n");
        return IRQ_NONE;
    }

    num_left = count - 1;

    /* Process remaining messages if necessary */
    if (num_left)
    {
        ret_val = mxt_read_and_process_messages(mxtdata, num_left);
        if ((ret_val >= 0) && (ret_val != num_left))
        {
            dev_warn(dev, "Unexpected invalid message\n");
        }
    }

    if (mxtdata->update_input)
    {
        mxt_input_sync(mxtdata);
        mxtdata->update_input = false;
    }

    return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int count, read;
    u8 tries = 2;

    dev_dbg(dev, "%s >\n", __func__);

    count = mxtdata->max_reportid;

    /* Read messages until we force an invalid */
    do
    {
        read = mxt_read_and_process_messages(mxtdata, count);
        if (read < count)
        {
            return 0;
        }
        tries--;
    }
    while (tries > 0);

    if (mxtdata->update_input)
    {
        mxt_input_sync(mxtdata);
        mxtdata->update_input = false;
    }

    dev_err(dev, "CHG pin isn't cleared\n");
    return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *mxtdata)
{
    int total_handled, num_handled;
    u8 count = mxtdata->last_message_count;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (count < 1 || count > mxtdata->max_reportid)
    {
        count = 1;
    }

    /* include final invalid message */
    total_handled = mxt_read_and_process_messages(mxtdata, count + 1);
    if (total_handled < 0)
    {
        return IRQ_NONE;
    }

    /* if there were not invalid messages, then we are not done */
    if (total_handled > count)
    {
        /* keep reading two msgs until one is invalid or reportid limit */
        do
        {
            num_handled = mxt_read_and_process_messages(mxtdata, 2);
            if (num_handled < 0)
            {
                return IRQ_NONE;
            }

            total_handled += num_handled;

            if (num_handled < 2)
            {
                break;
            }
        }
        while (total_handled < mxtdata->t100_num_touchids);
    }

    mxtdata->last_message_count = total_handled;

    if (mxtdata->update_input)
    {
        mxt_input_sync(mxtdata);
        mxtdata->update_input = false;
    }

    return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
    struct mxt_data *mxtdata = dev_id;

    //dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    complete(&mxtdata->chg_completion);

    if (!mxtdata->object_table)
    {
        return IRQ_HANDLED;
    }

    if (mxtdata->t44_cfg_address)
    {
        return mxt_process_messages_t44(mxtdata);
    }

    return mxt_process_messages(mxtdata);
}

static int mxt_send_t6_command_processor(struct mxt_data *mxtdata, u16 cmd_offset, u8 cmd_value, bool wait)
{
    u16 reg;
    u8 command_register;
    int timeout_counter = 100;
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    reg = mxtdata->t6_cfg_address + cmd_offset;

    ret_val = mxt_write_blks(mxtdata, reg, 1, cmd_value);
    if (ret_val)
    {
        return ret_val;
    }

    if (!wait)
    {
        return 0;
    }

    do
    {
        msleep(20);
        ret_val = mxt_read_blks(mxtdata, reg, 1, &command_register);
        if (ret_val)
        {
            return ret_val;
        }
        timeout_counter--;
    }
    while (command_register != 0 && timeout_counter > 0);

    if (timeout_counter <= 0)
    {
        dev_err(&mxtdata->spidevice->dev, "T6 cmd failed\n");
        return -EIO;
    }

    return 0;
}

static int mxt_soft_reset(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val = 0;

    dev_info(dev, "%s >\n", __func__);

    disable_irq(mxtdata->chg_irq);

    reinit_completion(&mxtdata->reset_completion);

    ret_val = mxt_send_t6_command_processor(mxtdata, MXT_T6_CFG_RESET_OFFSET, MXT_T6_CFG_RESET_VAL_TO_APP, false/*wait*/);
    if (ret_val)
    {
        return ret_val;
    }

    /* Ignore CHG line for 100ms after reset */
    msleep(100);

    enable_irq(mxtdata->chg_irq);

    ret_val = mxt_wait_for_completion(mxtdata, &mxtdata->reset_completion, MXT_RESET_TIMEOUT, "RST");
    if (ret_val)
    {
        return ret_val;
    }
    dev_info(dev, "%s <\n", __func__);
    return 0;
}

static void mxt_update_crc(struct mxt_data *mxtdata, u8 t6_cmd_offset, u8 t6_cmd_value)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    /*
     * On failure, CRC is set to 0 and config will always be downloaded
     */
    mxtdata->config_crc = 0;
    reinit_completion(&mxtdata->crc_completion);

    (void)mxt_send_t6_command_processor(mxtdata, t6_cmd_offset, t6_cmd_value, true/*wait*/);

    /*
     * Wait for crc message
     */
    (void)mxt_wait_for_completion(mxtdata, &mxtdata->crc_completion, MXT_CRC_TIMEOUT, "CRC");
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
    static const unsigned int crcpoly = 0x80001B;
    u32 result;
    u32 data_word;

    data_word = (secondbyte << 8) | firstbyte;
    result = ((*crc << 1) ^ data_word);

    if (result & 0x1000000)
    {
        result ^= crcpoly;
    }

    *crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
    u32 crc = 0;
    u8 *ptr = base + start_off;
    u8 *last_val = base + end_off - 1;

    if (end_off < start_off)
    {
        return -EINVAL;
    }

    while (ptr < last_val)
    {
        mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
        ptr += 2;
    }

    /* if len is odd, fill the last byte with 0 */
    if (ptr == last_val)
    {
        mxt_calc_crc24(&crc, *ptr, 0);
    }

    /* Mask to 24-bit */
    crc &= 0x00FFFFFF;

    return crc;
}

static int mxt_check_retrigen(struct mxt_data *mxtdata)
{
    int ret_val;
    int val;

    if (irq_get_trigger_type(mxtdata->chg_irq) & IRQF_TRIGGER_LOW)
    {
        return 0;
    }

    if (mxtdata->t18_cfg_address)
    {
        ret_val = mxt_read_blks(mxtdata,
                                mxtdata->t18_cfg_address + MXT_T18_CFG_CTRL_OFFSET,
                                1, &val);
        if (ret_val)
        {
            return ret_val;
        }

        if (val & MXT_T18_CFG_CTRL_RETRIGEN_BIT)
        {
            return 0; // RETRIGEN enabled
        }
    }

    dev_warn(&mxtdata->spidevice->dev, "Enabling RETRIGEN workaround\n");
    mxtdata->use_retrigen_workaround = true;
    return 0;
}

static int mxt_prepare_cfg_mem(struct mxt_data *mxtdata, struct mxt_cfg *cfg)
{
    struct device *dev = &mxtdata->spidevice->dev;
    struct mxt_object *object;
    unsigned int type, instance, size, byte_offset;
    int offset;
    int ret_val;
    int i;
    u16 reg;
    u8 val;

    dev_dbg(dev, "%s >\n", __func__);

    while (cfg->raw_pos < cfg->raw_size)
    {
        /* Read type, instance, size */
        ret_val = sscanf(cfg->raw + cfg->raw_pos, "%x %x %x%n", &type, &instance, &size, &offset);
        if (ret_val == 0)
        {
            /* EOF */
            break;
        }
        else if (ret_val != 3)
        {
            dev_err(dev, "Bad format: failed to parse object\n");
            return -EINVAL;
        }
        cfg->raw_pos += offset;

        object = mxt_get_object(mxtdata, type);
        if (!object)
        {
            dev_warn(dev, "Skipping T%d not found\n", type);
            for (i = 0; i < size; i++)
            {
                ret_val = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n", &val, &offset);
                if (ret_val != 1)
                {
                    dev_err(dev, "Bad format in T%d at %d\n", type, i);
                    return -EINVAL;
                }
                cfg->raw_pos += offset;
            }
            continue;
        }

        if (size > mxt_get_obj_size(object))
        {
            /*
             * Either we are in fallback mode due to wrong
             * config or config from a later fw version,
             * or the file is corrupt or hand-edited.
             */
            dev_warn(dev, "Discarding %zu byte(s) in T%u\n", size - mxt_get_obj_size(object), type);
        }
        else if (mxt_get_obj_size(object) > size)
        {
            /*
             * If firmware is upgraded, new bytes may be added to
             * end of objects. It is generally forward compatible
             * to zero these bytes - previous behaviour will be
             * retained. However this does invalidate the CRC and
             * will force fallback mode until the configuration is
             * updated. We warn here but do nothing else - the
             * malloc has zeroed the entire configuration.
             */
            dev_warn(dev, "Zeroing %zu byte(s) in T%d\n", mxt_get_obj_size(object) - size, type);
        }

        if (instance >= mxt_get_obj_num_instances(object))
        {
            dev_err(dev, "Object instances exceeded!\n");
            return -EINVAL;
        }

        reg = object->start_address + mxt_get_obj_size(object) * instance;

        for (i = 0; i < size; i++)
        {
            ret_val = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n", &val, &offset);
            if (ret_val != 1)
            {
                dev_err(dev, "Bad format in T%d at %d\n", type, i);
                return -EINVAL;
            }
            cfg->raw_pos += offset;

            if (i > mxt_get_obj_size(object))
            {
                continue;
            }

            byte_offset = reg + i - cfg->start_ofs;

            if (byte_offset >= 0 && byte_offset < cfg->mem_size)
            {
                *(cfg->mem + byte_offset) = val;
            }
            else
            {
                dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n", reg, object->type, byte_offset);
                return -EINVAL;
            }
        }
    }

    return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *mxtdata);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *mxtdata, const struct firmware *fw)
{
    struct device *dev = &mxtdata->spidevice->dev;
    struct mxt_cfg cfg;
    int ret_val;
    int offset;
    int i;
    u32 info_crc, config_crc, calculated_crc;
    u16 crc_start = 0;

    dev_dbg(dev, "%s >\n", __func__);

    /* Make zero terminated copy of the OBP_RAW file */
    cfg.raw = kzalloc(fw->size + 1, GFP_KERNEL);
    if (!cfg.raw)
    {
        return -ENOMEM;
    }

    memcpy(cfg.raw, fw->data, fw->size);
    cfg.raw[fw->size] = '\0';
    cfg.raw_size = fw->size;

    mxt_update_crc(mxtdata, MXT_T6_CFG_REPORTALL_OFFSET, MXT_T6_CFG_REPORTALL_VAL_EXEC);

    if (0 != strncmp(cfg.raw, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC)))
    {
        dev_err(dev, "Config file not '%s'\n", MXT_CFG_MAGIC);
        ret_val = -EINVAL;
        goto release_raw;
    }

    cfg.raw_pos = strlen(MXT_CFG_MAGIC);

    /* Load information block and check */
    for (i = 0; i < sizeof(struct mxt_info); i++)
    {
        ret_val = sscanf(cfg.raw + cfg.raw_pos, "%hhx%n", (unsigned char *)&cfg.mxtinfo + i, &offset);
        if (ret_val != 1)
        {
            dev_err(dev, "CFG bad format\n");
            ret_val = -EINVAL;
            goto release_raw;
        }

        cfg.raw_pos += offset;
    }

    if (cfg.mxtinfo.family_id != mxtdata->mxtinfo->family_id)
    {
        dev_err(dev, "Family ID mismatch!\n");
        ret_val = -EINVAL;
        goto release_raw;
    }

    if (cfg.mxtinfo.variant_id != mxtdata->mxtinfo->variant_id)
    {
        dev_err(dev, "Variant ID mismatch!\n");
        ret_val = -EINVAL;
        goto release_raw;
    }

    /* Read CRCs */
    ret_val = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &info_crc, &offset);
    if (ret_val != 1)
    {
        dev_err(dev, "Bad format: failed to parse Info CRC\n");
        ret_val = -EINVAL;
        goto release_raw;
    }
    cfg.raw_pos += offset;

    ret_val = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &config_crc, &offset);
    if (ret_val != 1)
    {
        dev_err(dev, "Bad format: failed to parse Config CRC\n");
        ret_val = -EINVAL;
        goto release_raw;
    }
    cfg.raw_pos += offset;

    /*
     * The Info Block CRC is calculated over mxt_info and the object
     * table. If it does not match then we are trying to load the
     * configuration from a different chip or firmware version, so
     * the configuration CRC is invalid anyway.
     */
    if (info_crc == mxtdata->info_crc)
    {
        if (config_crc == 0 || mxtdata->config_crc == 0)
        {
            dev_info(dev, "CRC zero, attempting to apply config\n");
        }
        else if (config_crc == mxtdata->config_crc)
        {
            dev_dbg(dev, "Config CRC 0x%06X: *match*\n", mxtdata->config_crc);
            return 0;
        }
        else
        {
            dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n", mxtdata->config_crc, config_crc);
        }
    }
    else
    {
        dev_warn(dev, "Warning: Info CRC error - device=0x%06X file=0x%06X\n", mxtdata->info_crc, info_crc);
    }

    /* Malloc memory to store configuration */
    cfg.start_ofs = MXT_OBJECT_START + mxtdata->mxtinfo->object_num * sizeof(struct mxt_object) + MXT_INFO_CHECKSUM_SIZE;
    cfg.mem_size = mxtdata->mem_size - cfg.start_ofs;
    cfg.mem = kzalloc(cfg.mem_size, GFP_KERNEL);
    if (!cfg.mem)
    {
        ret_val = -ENOMEM;
        goto release_raw;
    }

    ret_val = mxt_prepare_cfg_mem(mxtdata, &cfg);
    if (ret_val)
    {
        goto release_mem;
    }

    /* Calculate crc of the received configs (not the raw config file) */
    if (mxtdata->t71_cfg_address)
    {
        crc_start = mxtdata->t71_cfg_address;
    }
    else if (mxtdata->t7_cfg_address)
    {
        crc_start = mxtdata->t7_cfg_address;
    }
    else
    {
        dev_warn(dev, "Could not find CRC start\n");
    }

    if (crc_start > cfg.start_ofs)
    {
        calculated_crc = mxt_calculate_crc(cfg.mem,
                                           crc_start - cfg.start_ofs,
                                           cfg.mem_size);

        if (config_crc > 0 && config_crc != calculated_crc)
        {
            dev_warn(dev, "Config CRC in file inconsistent, calculated=%06X, file=%06X\n", calculated_crc, config_crc);
        }
    }

    ret_val = mxt_write_blks(mxtdata, cfg.start_ofs, cfg.mem_size, cfg.mem);
    if (ret_val)
    {
        goto release_mem;
    }

    mxt_update_crc(mxtdata, MXT_T6_CFG_BACKUPNV_OFFSET, MXT_T6_CFG_BACKUPNV_VAL_TO_NVM);

    ret_val = mxt_check_retrigen(mxtdata);
    if (ret_val)
    {
        goto release_mem;
    }

    ret_val = mxt_soft_reset(mxtdata);
    if (ret_val)
    {
        goto release_mem;
    }

    dev_info(dev, "Config successfully updated\n");

    /* T7 config may have changed */
    mxt_init_t7_power_cfg(mxtdata);

release_mem:
    kfree(cfg.mem);
release_raw:
    kfree(cfg.raw);

    return ret_val;
}

static int mxt_acquire_irq(struct mxt_data *mxtdata)
{
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->chg_irq)
    {
        mxtdata->chg_irq = gpio_to_irq(mxtdata->mxtplatform->gpio_chg_irq);
        ret_val = request_threaded_irq(mxtdata->chg_irq,
                                       NULL,
                                       mxt_interrupt,
                                       IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
                                       mxtdata->spidevice->name,
                                       mxtdata);
        if (ret_val)
        {
            dev_err(&mxtdata->spidevice->dev, "request_threaded_irq (%d)\n", ret_val);
            return ret_val;
        }

        /* Presence of mxtdata->chg_irq means IRQ initialised */
        dev_info(&mxtdata->spidevice->dev, "gpio_to_irq %lu -> %d\n", mxtdata->mxtplatform->gpio_chg_irq, mxtdata->chg_irq);
    }
    else
    {
        enable_irq(mxtdata->chg_irq);
    }

    if (mxtdata->object_table && mxtdata->use_retrigen_workaround)
    {
        ret_val = mxt_process_messages_until_invalid(mxtdata);
        if (ret_val)
        {
            return ret_val;
        }
    }

    dev_info(&mxtdata->spidevice->dev, "%s <\n", __func__);

    return 0;
}

static int mxt_input_open(struct input_dev *inputdev);
static void mxt_input_close(struct input_dev *inputdev);

static void mxt_unregister_input_device(struct mxt_data *mxtdata)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (mxtdata->inputdev)
    {
#ifdef INPUT_DEVICE_ALWAYS_OPEN
        mxt_input_close(mxtdata->inputdev);
#endif //INPUT_DEVICE_ALWAYS_OPEN

        input_unregister_device(mxtdata->inputdev);
    }
}

static void mxt_free_object_table(struct mxt_data *mxtdata)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    mxtdata->object_table = NULL;
    mxtdata->mxtinfo = NULL;
    kfree(mxtdata->raw_info_block);
    mxtdata->raw_info_block = NULL;
    kfree(mxtdata->msg_buf);
    mxtdata->msg_buf = NULL;

    mxtdata->t5_cfg_address = 0;
    mxtdata->t6_cfg_address = 0;
    mxtdata->t7_cfg_address = 0;
    mxtdata->t15_cfg_address = 0;
    mxtdata->t18_cfg_address = 0;
    mxtdata->t44_cfg_address = 0;
    mxtdata->t71_cfg_address = 0;
    mxtdata->t93_cfg_address = 0;
    mxtdata->t100_cfg_address = 0;
    mxtdata->t107_cfg_address = 0;

    mxtdata->t5_msg_size = 0;

    mxtdata->t6_msg_reportid = 0;
    mxtdata->t100_msg_reportid_min = 0;
    mxtdata->t100_msg_reportid_max = 0;

    mxtdata->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *mxtdata, struct mxt_object *object_table)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int i;
    u8 curr_rep_id = 1; /* Valid Report IDs start counting from 1 */
    u16 curr_end_address = 0;

    dev_info(&spidevice->dev, "%s, object_num = %d\n", __func__, mxtdata->mxtinfo->object_num);

    mxtdata->mem_size = 0;
    for (i = 0; i < mxtdata->mxtinfo->object_num; i++)
    {
        struct mxt_object *object = object_table + i;
        u8 min_rep_id, max_rep_id;

        // object start_address from little endian to local CPU endianness **IN PLACE**
        // IMPORTANT: this is only for the first loop throuch the object table
        le16_to_cpus(&object->start_address);

        if (object->num_report_ids)
        {
            min_rep_id = curr_rep_id;
            curr_rep_id += object->num_report_ids * mxt_get_obj_num_instances(object);
            max_rep_id = curr_rep_id - 1;
        }
        else
        {
            min_rep_id = 0;
            max_rep_id = 0;
        }

        dev_dbg(&mxtdata->spidevice->dev,
                "T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
                object->type, object->start_address,
                mxt_get_obj_size(object), mxt_get_obj_num_instances(object),
                min_rep_id, max_rep_id);

        switch (object->type)
        {
            case MXT_GEN_MESSAGEPROCESSOR_T5:
                mxtdata->t5_cfg_address = object->start_address;
                if (mxtdata->mxtinfo->family_id == 0x80 &&
                    mxtdata->mxtinfo->version < 0x20)
                {
                    /*
                     * On mXT224 firmware versions prior to V2.0
                     * read and discard unused CRC byte otherwise
                     * DMA reads are misaligned.
                     */
                    mxtdata->t5_msg_size = mxt_get_obj_size(object);
                }
                else
                {
                    /* CRC not enabled, so skip last byte */
                    mxtdata->t5_msg_size = mxt_get_obj_size(object) - 1;
                }
                break;
            case MXT_GEN_COMMANDPROCESSOR_T6:
                mxtdata->t6_cfg_address = object->start_address;
                mxtdata->t6_msg_reportid = min_rep_id;
                break;
            case MXT_GEN_POWERCONFIG_T7:
                mxtdata->t7_cfg_address = object->start_address;
                break;
            case MXT_SPT_COMMSCONFIG_T18:
                mxtdata->t18_cfg_address = object->start_address;
                break;
            case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
                mxtdata->t71_cfg_address = object->start_address;
                break;
            case MXT_TOUCH_KEYARRAY_T15:
                mxtdata->t15_cfg_address = object->start_address;
                break;
            case MXT_SPT_MESSAGECOUNT_T44:
                mxtdata->t44_cfg_address = object->start_address;
                break;
            case MXT_PROCI_TOUCHSEQUENCELOGGER_T93:
                mxtdata->t93_cfg_address = object->start_address;
                break;
            case MXT_TOUCH_MULTITOUCHSCREEN_T100:
                mxtdata->t100_cfg_address = object->start_address;
                mxtdata->t100_msg_reportid_min = min_rep_id;
                mxtdata->t100_msg_reportid_max = max_rep_id;
                // First Report ID is Screen Status Messages
                // Second Report ID is Reserved
                // Subsequent Report IDs are Touch Status Messages
                mxtdata->t100_num_touchids = object->num_report_ids - 2;
                break;
            case MXT_PROCI_ACTIVESTYLUS_T107:
                mxtdata->t107_cfg_address = object->start_address;
                break;
        }

        curr_end_address = object->start_address + mxt_get_obj_size(object) * mxt_get_obj_num_instances(object) - 1;
        if (curr_end_address >= mxtdata->mem_size)
        {
            mxtdata->mem_size = curr_end_address + 1;
        }
    }

    /* Store maximum reportid */
    mxtdata->max_reportid = curr_rep_id;

    /* If T44 exists, T5 position has to be directly after */
    if (mxtdata->t44_cfg_address && (mxtdata->t5_cfg_address != mxtdata->t44_cfg_address + 1))
    {
        dev_err(&spidevice->dev, "Invalid T44 position\n");
        return -EINVAL;
    }

    mxtdata->msg_buf = kcalloc(mxtdata->max_reportid, mxtdata->t5_msg_size, GFP_KERNEL);
    if (NULL == mxtdata->msg_buf)
    {
        return -ENOMEM;
    }

    return 0;
}

static int mxt_read_info_block(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int ret_val;
    size_t size;
    void *mxtinfo, *buf;
    u8 num_objects;
    u32 calculated_crc;
    u8 *crc_ptr;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    /* If info block already allocated, free it */
    if (mxtdata->raw_info_block != NULL)
    {
        mxt_free_object_table(mxtdata);
    }

    /* Read 7-byte ID information block starting at address 0 */
    size = sizeof(struct mxt_info);
    mxtinfo = kzalloc(size, GFP_KERNEL);
    if (NULL == mxtinfo)
    {
        return -ENOMEM;
    }

    /* Read information block, starting at address 0 */
    ret_val = mxt_read_blks(mxtdata, 0, size, mxtinfo);
    if (ret_val)
    {
        kfree(mxtinfo);
        return ret_val;
    }

    /* Resize buffer to give space for the object table and checksum */
    num_objects = ((struct mxt_info *)mxtinfo)->object_num;
    size += (num_objects * sizeof(struct mxt_object)) + MXT_INFO_CHECKSUM_SIZE;

    buf = krealloc(mxtinfo, size, GFP_KERNEL);
    if (NULL == buf)
    {
        return -ENOMEM;
    }

    /* Read rest of info block */
    ret_val = mxt_read_blks(mxtdata,
                            MXT_OBJECT_START,
                            size - MXT_OBJECT_START,
                            buf + MXT_OBJECT_START);
    if (ret_val)
    {
        goto err_free_buf;
    }

    /* Extract & calculate checksum */
    crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
    mxtdata->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

    calculated_crc = mxt_calculate_crc(buf, 0, size - MXT_INFO_CHECKSUM_SIZE);

    /*
     * CRC mismatch can be caused by data corruption due to comms
     * issue or else device is not using Object Based Protocol (eg i2c-hid)
     */
    if ((mxtdata->info_crc == 0) || (mxtdata->info_crc != calculated_crc))
    {
        dev_err(&spidevice->dev,
                "Info Block CRC error calculated=0x%06X read=0x%06X\n",
                calculated_crc, mxtdata->info_crc);
        ret_val = -EIO;
        goto err_free_buf;
    }

    mxtdata->raw_info_block = buf;
    mxtdata->mxtinfo = (struct mxt_info *)buf;

    dev_info(&spidevice->dev,
             "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
             mxtdata->mxtinfo->family_id, mxtdata->mxtinfo->variant_id,
             mxtdata->mxtinfo->version >> 4, mxtdata->mxtinfo->version & 0xf,
             mxtdata->mxtinfo->build, mxtdata->mxtinfo->object_num);

    /* Parse object table information */
    ret_val = mxt_parse_object_table(mxtdata, buf + MXT_OBJECT_START);
    if (ret_val)
    {
        dev_err(&spidevice->dev, "Error %d parsing object table\n", ret_val);
        mxt_free_object_table(mxtdata);
        return ret_val;
    }

    mxtdata->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

    return 0;

err_free_buf:
    kfree(buf);
    return ret_val;
}

static void mxt_regulator_enable(struct mxt_data *mxtdata)
{
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->reg_vdd || !mxtdata->reg_avdd)
    {
        return;
    }

    gpio_set_value(mxtdata->mxtplatform->gpio_reset, GPIO_RESET_YES_LOW);

    ret_val = regulator_enable(mxtdata->reg_vdd);
    if (ret_val)
    {
        return;
    }

    ret_val = regulator_enable(mxtdata->reg_avdd);
    if (ret_val)
    {
        return;
    }

    /*
     * According to maXTouch power sequencing specification, RESET line
     * must be kept low until some time after regulators come up to
     * voltage
     */
    msleep(MXT_REGULATOR_DELAY);
    gpio_set_value(mxtdata->mxtplatform->gpio_reset, GPIO_RESET_NO_HIGH);
    msleep(MXT_CHG_DELAY);

retry_wait:
    reinit_completion(&mxtdata->chg_completion);
    mxtdata->in_bootloader = true;
    ret_val = mxt_wait_for_completion(mxtdata, &mxtdata->chg_completion, MXT_POWERON_DELAY, "CHG");
    if (ret_val == -EINTR)
    {
        goto retry_wait;
    }
    mxtdata->in_bootloader = false;
}

static void mxt_regulator_disable(struct mxt_data *mxtdata)
{
    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->reg_vdd || !mxtdata->reg_avdd)
    {
        return;
    }

    regulator_disable(mxtdata->reg_vdd);
    regulator_disable(mxtdata->reg_avdd);
}

static int mxt_probe_regulators(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;

    dev_dbg(dev, "%s >\n", __func__);

    /* Must have reset GPIO to use regulator support */
    if (!gpio_is_valid(mxtdata->mxtplatform->gpio_reset))
    {
        ret_val = -EINVAL;
        goto fail;
    }
    if (!gpio_is_valid(mxtdata->mxtplatform->gpio_chg_irq))
    {
        ret_val = -EINVAL;
        goto fail;
    }

    mxtdata->reg_vdd = regulator_get(dev, "vdd");
    if (IS_ERR(mxtdata->reg_vdd))
    {
        ret_val = PTR_ERR(mxtdata->reg_vdd);
        dev_err(dev, "Error %d getting vdd regulator\n", ret_val);
        goto fail;
    }

    mxtdata->reg_avdd = regulator_get(dev, "avdd");
    if (IS_ERR(mxtdata->reg_avdd))
    {
        ret_val = PTR_ERR(mxtdata->reg_avdd);
        dev_err(dev, "Error %d getting avdd regulator\n", ret_val);
        goto fail_release;
    }

    mxt_regulator_enable(mxtdata);

    dev_dbg(dev, "Initialised regulators\n");
    return 0;

fail_release:
    regulator_put(mxtdata->reg_vdd);
    regulator_put(mxtdata->reg_avdd);

fail:
    mxtdata->reg_vdd = NULL;
    mxtdata->reg_avdd = NULL;
    gpio_free(mxtdata->mxtplatform->gpio_reset);
    gpio_free(mxtdata->mxtplatform->gpio_chg_irq);
    return ret_val;
}

static int mxt_input_device_set_up_active_stylus(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int ret_val;
    struct mxt_object *object;
    u8 styaux_byte;
    int t100_msg_aux_byte_idx;
    u8 ctrl_byte;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    object = mxt_get_object(mxtdata, MXT_PROCI_ACTIVESTYLUS_T107);
    if (!object)
    {
        return 0;
    }

    ret_val = mxt_read_blks(mxtdata, object->start_address, 1, &ctrl_byte);
    if (ret_val)
    {
        return ret_val;
    }

    /* Check enable bit */
    if (0 == (ctrl_byte & 0x01))
    {
        return 0;
    }

    ret_val = mxt_read_blks(mxtdata,
                            object->start_address + MXT_T107_CFG_STYAUX_OFFSET,
                            1, &styaux_byte);
    if (ret_val)
    {
        return ret_val;
    }

    /* map aux bits - the first aux of t100 msg is fixed for t107 */
    t100_msg_aux_byte_idx = MXT_T100_MSG_AUXDATA_OFFSET+1;
    if (styaux_byte & MXT_T107_CFG_STYAUX_PRS_BIT)
    {
        mxtdata->t100_stylus_aux_pressure_idx = t100_msg_aux_byte_idx++;
    }
    if (styaux_byte & MXT_T107_CFG_STYAUX_XPEAK_BIT)
    {
        mxtdata->t100_stylus_aux_xpeak_idx = t100_msg_aux_byte_idx++;
    }
    if (styaux_byte & MXT_T107_CFG_STYAUX_YPEAK_BIT)
    {
        mxtdata->t100_stylus_aux_ypeak_idx = t100_msg_aux_byte_idx++;
    }

    input_set_capability(mxtdata->inputdev, EV_KEY, BTN_STYLUS);
    input_set_capability(mxtdata->inputdev, EV_KEY, BTN_STYLUS2);

    dev_dbg(&spidevice->dev,
            "T107 on T100 aux - pressure:%u xpeak:%u ypeak:%u\n",
            mxtdata->t100_stylus_aux_pressure_idx, mxtdata->t100_stylus_aux_xpeak_idx, mxtdata->t100_stylus_aux_ypeak_idx);

    return 0;
}

static int mxt_read_t100_multiple_touch_cfg(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int ret_val;
    struct mxt_object *object;
    u8 range_x[2], range_y[2];
    u8 cfg1_byte, tchaux_byte;
    u8 t100_msg_aux_byte_idx;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    object = mxt_get_object(mxtdata, MXT_TOUCH_MULTITOUCHSCREEN_T100);
    if (!object)
    {
        return -EINVAL;
    }

    /* read touchscreen dimensions */
    ret_val = mxt_read_blks(mxtdata,
                            object->start_address + MXT_T100_CFG_XRANGE_OFFSET,
                            2, range_x);
    if (ret_val)
    {
        return ret_val;
    }

    mxtdata->t100_max_x = range_x[0] | range_x[1] << 8; // little endian 16

    ret_val = mxt_read_blks(mxtdata,
                            object->start_address + MXT_T100_CFG_YRANGE_OFFSET,
                            2, range_y);
    if (ret_val)
    {
        return ret_val;
    }

    mxtdata->t100_max_y = range_y[0] | range_y[1] << 8; // little endian 16

    /* read orientation config */
    ret_val =  mxt_read_blks(mxtdata,
                             object->start_address + MXT_T100_CFG_CFG1_OFFSET,
                             1, &cfg1_byte);
    if (ret_val)
    {
        return ret_val;
    }

    mxtdata->t100_xy_switch = cfg1_byte & MXT_T100_CFG_CFG1_SWITCHXY_BIT;

    ret_val =  mxt_read_blks(mxtdata,
                             object->start_address + MXT_T100_CFG_TCHAUX_OFFSET,
                             1, &tchaux_byte);
    if (ret_val)
    {
        return ret_val;
    }

    /* map aux bits */
    t100_msg_aux_byte_idx = MXT_T100_MSG_AUXDATA_OFFSET;
    if (tchaux_byte & MXT_T100_CFG_TCHAUX_VECT_BIT)
    {
        mxtdata->t100_aux_vect_idx = t100_msg_aux_byte_idx++;
    }
    if (tchaux_byte & MXT_T100_CFG_TCHAUX_AMPL_BIT)
    {
        mxtdata->t100_aux_ampl_idx = t100_msg_aux_byte_idx++;
    }
    if (tchaux_byte & MXT_T100_CFG_TCHAUX_AREA_BIT)
    {
        mxtdata->t100_aux_area_idx = t100_msg_aux_byte_idx++;
    }

    dev_dbg(&spidevice->dev,
            "T100 aux - vect:%u ampl:%u area:%u\n",
            mxtdata->t100_aux_vect_idx, mxtdata->t100_aux_ampl_idx, mxtdata->t100_aux_area_idx);

    return 0;
}

static void mxt_input_device_set_up_as_touchpad(struct mxt_data *mxtdata)
{
    const struct mxt_platform_data *mxtplatform = mxtdata->mxtplatform;
    int i;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    mxtdata->inputdev->name = "Solomon Systech maXTouch Touchpad";

    /*
     * https://www.kernel.org/doc/Documentation/input/event-codes.txt
     * touchpads where the button is placed beneath the surface,
     * such that pressing down on the pad causes a button click
     */
    __set_bit(INPUT_PROP_BUTTONPAD, mxtdata->inputdev->propbit);

    input_abs_set_res(mxtdata->inputdev, ABS_X, MXT_PIXELS_PER_MM);
    input_abs_set_res(mxtdata->inputdev, ABS_Y, MXT_PIXELS_PER_MM);
    input_abs_set_res(mxtdata->inputdev, ABS_MT_POSITION_X, MXT_PIXELS_PER_MM);
    input_abs_set_res(mxtdata->inputdev, ABS_MT_POSITION_Y, MXT_PIXELS_PER_MM);

    for (i = 0; i < mxtplatform->t19_gpio_num_keys; i++)
    {
        if (mxtplatform->t19_gpio_keymap[i] != KEY_RESERVED)
        {
            input_set_capability(mxtdata->inputdev, EV_KEY, mxtplatform->t19_gpio_keymap[i]);
        }
    }
}

static int mxt_input_device_initialize(struct mxt_data *mxtdata)
{
    const struct mxt_platform_data *mxtplatform = mxtdata->mxtplatform;
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    unsigned int mt_flags = 0;
    int i;

    dev_dbg(dev, "%s >\n", __func__);

    if (mxtdata->t100_cfg_address)
    {
        ret_val = mxt_read_t100_multiple_touch_cfg(mxtdata);
        if (ret_val)
        {
            dev_warn(dev, "Failed to read T100 config\n");
        }
    }
    else
    {
        dev_err(dev, "NO multitouch\n");
        return -EINVAL;
    }

    /* Handle default values and orientation switch */
    if (mxtdata->t100_max_x == 0)
    {
        mxtdata->t100_max_x = 1023;
    }

    if (mxtdata->t100_max_y == 0)
    {
        mxtdata->t100_max_y = 1023;
    }

    if (mxtdata->t100_xy_switch)
    {
        swap(mxtdata->t100_max_x, mxtdata->t100_max_y);
    }

    dev_info(dev, "Touchscreen size X%uY%u\n", mxtdata->t100_max_x, mxtdata->t100_max_y);

    /* we allocate the device once while the device is registered and unregistered more times */
    if (!mxtdata->inputdev)
    {
        /* allocate memory for new managed input device (no need to free) */
        mxtdata->inputdev = devm_input_allocate_device(dev);
        if (!mxtdata->inputdev)
        {
            return -ENOMEM;
        }
    }

    if (mxtdata->mxtplatform->input_name)
    {
        mxtdata->inputdev->name = mxtdata->mxtplatform->input_name;
    }
    else
    {
        mxtdata->inputdev->name = "Solomon Systech maXTouch Touchscreen";
    }

    mxtdata->inputdev->phys = mxtdata->phys;
    mxtdata->inputdev->id.bustype = BUS_SPI;
    mxtdata->inputdev->dev.parent = dev;

#ifndef INPUT_DEVICE_ALWAYS_OPEN
    mxtdata->inputdev->open = mxt_input_open;
    mxtdata->inputdev->close = mxt_input_close;
#endif //INPUT_DEVICE_ALWAYS_OPEN

    /* 
     * https://www.kernel.org/doc/Documentation/input/input-programming.txt
     * https://www.kernel.org/doc/Documentation/input/event-codes.txt
     * https://www.kernel.org/doc/Documentation/input/multi-touch-protocol.txt
     */

    /* we support EV_ABS: absolute axis value changes */
    set_bit(EV_ABS/*event type*/, mxtdata->inputdev->evbit);

    /* we support EV_KEY: keyboards, buttons, or other key-like devices */
    input_set_capability(mxtdata->inputdev, EV_KEY/*event type*/, BTN_TOUCH/*event code*/);

    /* for single touch (no multi touch) */
    input_set_abs_params(mxtdata->inputdev, ABS_X, 0, mxtdata->t100_max_x, 0/*fuzz,noise*/, 0/*flat position*/);
    input_set_abs_params(mxtdata->inputdev, ABS_Y, 0, mxtdata->t100_max_y, 0, 0);

    if (mxtdata->t100_aux_ampl_idx)
    {
        input_set_abs_params(mxtdata->inputdev, ABS_PRESSURE, 0, 255, 0, 0);
    }

    /* if device has buttons we assume it is a touchpad */
    if (mxtplatform->t19_gpio_num_keys)
    {
        mxt_input_device_set_up_as_touchpad(mxtdata);
        mt_flags |= INPUT_MT_POINTER;
    }
    else
    {
        mt_flags |= INPUT_MT_DIRECT;
    }

    /* multi touch */
    ret_val = input_mt_init_slots(mxtdata->inputdev, mxtdata->t100_num_touchids, mt_flags);
    if (ret_val)
    {
        dev_err(dev, "Error %d initialising slots\n", ret_val);
        return ret_val;
    }

    input_set_abs_params(mxtdata->inputdev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
    input_set_abs_params(mxtdata->inputdev, ABS_MT_DISTANCE, MXT_TOUCH_DISTANCE_ACTIVE_TOUCH, MXT_TOUCH_DISTANCE_HOVERING, 0, 0);

    input_set_abs_params(mxtdata->inputdev, ABS_MT_POSITION_X, 0, mxtdata->t100_max_x, 0, 0);
    input_set_abs_params(mxtdata->inputdev, ABS_MT_POSITION_Y, 0, mxtdata->t100_max_y, 0, 0);

    if (mxtdata->t100_aux_area_idx)
    {
        input_set_abs_params(mxtdata->inputdev, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_AREA, 0, 0);
    }

    if (mxtdata->t100_aux_ampl_idx || mxtdata->t100_stylus_aux_pressure_idx)
    {
        input_set_abs_params(mxtdata->inputdev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    }

    if (mxtdata->t100_aux_vect_idx)
    {
        input_set_abs_params(mxtdata->inputdev, ABS_MT_ORIENTATION, 0, 255, 0, 0);
    }

    /* For T107 Active Stylus */
    if (mxtdata->t107_cfg_address)
    {
        ret_val = mxt_input_device_set_up_active_stylus(mxtdata);
        if (ret_val)
        {
            dev_err(dev, "Failed to read T107 config\n");
        }
    }

    /* For T15 Key Array */
    if (mxtdata->t15_cfg_address)
    {
        mxtdata->t15_keyarray_keystatus = 0;

        for (i = 0; i < mxtdata->mxtplatform->t15_keyarray_num_keys; i++)
        {
            input_set_capability(mxtdata->inputdev, EV_KEY, mxtdata->mxtplatform->t15_keyarray_keymap[i]);
        }
    }

    /* For double tap */
    if (mxtdata->t93_cfg_address)
    {
        input_set_capability(mxtdata->inputdev, EV_KEY, KEY_WAKEUP);
    }

    input_set_drvdata(mxtdata->inputdev, mxtdata);

    dev_dbg(dev, "input_register_device\n");
    ret_val = input_register_device(mxtdata->inputdev);
    if (ret_val)
    {
        dev_err(dev, "Error %d registering input device\n", ret_val);
        return ret_val;
    }

#ifdef INPUT_DEVICE_ALWAYS_OPEN
    mxt_input_open(mxtdata->inputdev);
#endif //INPUT_DEVICE_ALWAYS_OPEN

    dev_info(dev, "%s <\n", __func__);

    return 0;
}

static int mxt_set_t93_touchsequence_ena_dis_cfg(struct mxt_data *mxtdata, u16 cmd_offset, u8 status)
{
    u16 reg;
    u8 command_register;
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    reg = mxtdata->t93_cfg_address + cmd_offset;
    ret_val = mxt_read_blks(mxtdata, reg, 1, &command_register);
    if (ret_val)
    {
        return ret_val;
    }

    if (status == MXT_T93_ENABLE)
    {
        command_register |= (MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);
    }
    if (status == MXT_T93_DISABLE)
    {
        command_register &= ~(MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);
    }
    ret_val = mxt_write_blks(mxtdata, reg, 1, command_register);

    if (ret_val)
    {
        return ret_val;
    }

    return 0;
}

static int mxt_set_t100_multitouchscreen_cfg(struct mxt_data *mxtdata, u16 cmd_offset, u8 type)
{
    u16 reg;
    u8 command_register;
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    reg = mxtdata->t100_cfg_address + cmd_offset;
    command_register = type;

    ret_val = mxt_write_blks(mxtdata, reg, 1, command_register);

    if (ret_val)
    {
        return ret_val;
    }

    return 0;
}

static int mxt_sysfs_mem_access_init(struct mxt_data *mxtdata);
static void mxt_sysfs_mem_access_remove(struct mxt_data *mxtdata);

static int mxt_configure_objects(struct mxt_data *mxtdata, const struct firmware *cfg);

static void mxt_request_fw_config_nowait_cb(const struct firmware *cfg, void *ctx)
{
    (void)mxt_configure_objects(ctx, cfg);
    release_firmware(cfg);
}

static int mxt_initialize(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int recovery_attempts = 0;
    int ret_val;

    dev_dbg(&spidevice->dev, "%s >\n", __func__);

    while (1)
    {
        ret_val = mxt_read_info_block(mxtdata);
        if (!ret_val)
        {
            break;
        }

        /* Check bootloader state */
        ret_val = mxt_probe_bootloader(mxtdata, false);
        if (ret_val)
        {
            dev_info(&spidevice->dev, "Trying alternate bootloader address\n");
            ret_val = mxt_probe_bootloader(mxtdata, true);
            if (ret_val)
            {
                /* Chip is not in appmode or bootloader mode */
                return ret_val;
            }
        }

        /* OK, we are in bootloader, see if we can recover */
        if (++recovery_attempts > 1)
        {
            dev_err(&spidevice->dev, "Could not recover from bootloader mode\n");
            /*
             * We can reflash from this state, so do not
             * abort initialization.
             */
            mxtdata->in_bootloader = true;
            mxtdata->force_update_fw = true;
            return 0;
        }

        /* Attempt to exit bootloader into app mode */
        mxt_send_bootloader_reset_cmd(mxtdata);
        msleep(MXT_FW_RESET_TIME);
    }

    ret_val = mxt_check_retrigen(mxtdata);
    if (ret_val)
    {
        goto err_free_object_table;
    }

    ret_val = mxt_acquire_irq(mxtdata);
    if (ret_val)
    {
        goto err_free_object_table;
    }

    ret_val = mxt_sysfs_mem_access_init(mxtdata);
    if (ret_val)
    {
        goto err_free_object_table;
    }

    ret_val = mxt_sysfs_debug_msg_init(mxtdata);
    if (ret_val)
    {
        goto err_free_object_table;
    }

    if (mxtdata->cfg_name)
    {
        ret_val = request_firmware_nowait(THIS_MODULE,
                                          true,
                                          mxtdata->cfg_name,
                                          &mxtdata->spidevice->dev,
                                          GFP_KERNEL,
                                          mxtdata,
                                          mxt_request_fw_config_nowait_cb);
        if (ret_val)
        {
            dev_err(&spidevice->dev, "Failed to invoke firmware loader: %d\n", ret_val);
            goto err_free_object_table;
        }
    }
    else
    {
        ret_val = mxt_configure_objects(mxtdata, NULL);
        if (ret_val)
        {
            goto err_free_object_table;
        }
    }

    dev_info(&spidevice->dev, "%s <\n", __func__);

    return 0;

err_free_object_table:
    mxt_free_object_table(mxtdata);
    return ret_val;
}

static int mxt_set_t7_power_cfg(struct mxt_data *mxtdata, u8 sleep)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    struct t7_powerconfig *new_config;
    struct t7_powerconfig deepsleep = { .active = 0, .idle = 0 };

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (sleep == MXT_T7_POWER_CFG_DEEPSLEEP)
    {
        new_config = &deepsleep;
    }
    else
    {
        new_config = &mxtdata->t7_powercfg;
    }

    ret_val = mxt_write_blks(mxtdata,
                             mxtdata->t7_cfg_address,
                             sizeof(mxtdata->t7_powercfg),
                             new_config);
    if (0 == ret_val)
    {
        dev_info(dev, "Set T7 ACTV:%d IDLE:%d\n", new_config->active, new_config->idle);
    }

    return ret_val;
}

static int mxt_init_t7_power_cfg(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;
    int retry;

    dev_dbg(dev, "%s >\n", __func__);

    for (retry = 0; retry < 2; retry++)
    {
        ret_val = mxt_read_blks(mxtdata,
                                mxtdata->t7_cfg_address,
                                sizeof(mxtdata->t7_powercfg),
                                &mxtdata->t7_powercfg);
        if (ret_val)
        {
            return ret_val;
        }
        if (0 == mxtdata->t7_powercfg.active || 0 == mxtdata->t7_powercfg.idle)
        {
            if (0 == retry)
            {
                dev_dbg(dev, "T7 cfg zero, resetting\n");
                mxt_soft_reset(mxtdata);
            }
            else
            {
                dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
                mxtdata->t7_powercfg.active = 20;
                mxtdata->t7_powercfg.idle = 100;
                ret_val = mxt_set_t7_power_cfg(mxtdata, MXT_T7_POWER_CFG_RUN);
            }
        }
        else
        {
            break;
        }
    }

    dev_dbg(dev, "Init T7: ACTV %d IDLE %d\n", mxtdata->t7_powercfg.active, mxtdata->t7_powercfg.idle);

    return ret_val;
}

static int mxt_configure_objects(struct mxt_data *mxtdata, const struct firmware *cfg)
{
    struct device *dev = &mxtdata->spidevice->dev;
    int ret_val;

    dev_dbg(dev, "%s %s >\n", __func__, cfg ? "cfg":"-");

    ret_val = mxt_init_t7_power_cfg(mxtdata);
    if (ret_val)
    {
        dev_err(dev, "Failed to initialize power cfg\n");
        goto err_free_object_table;
    }

    if (cfg)
    {
        ret_val = mxt_update_cfg(mxtdata, cfg);
        if (ret_val)
        {
            dev_warn(dev, "Error %d updating config\n", ret_val);
        }
    }

    ret_val = mxt_input_device_initialize(mxtdata);
    if (ret_val)
    {
        goto err_free_object_table;
    }

    return 0;

err_free_object_table:
    mxt_free_object_table(mxtdata);
    return ret_val;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_devattr_config_crc_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%06x\n", mxtdata->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_devattr_fw_version_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
                     mxtdata->mxtinfo->version >> 4, mxtdata->mxtinfo->version & 0xf,
                     mxtdata->mxtinfo->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_devattr_hw_version_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
                     mxtdata->mxtinfo->family_id, mxtdata->mxtinfo->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
                                 struct mxt_object *object, int instance,
                                 const u8 *val)
{
    int i;

    if (mxt_get_obj_num_instances(object) > 1)
        count += scnprintf(buf + count, PAGE_SIZE - count,
                           "Instance %u\n", instance);

    for (i = 0; i < mxt_get_obj_size(object); i++)
        count += scnprintf(buf + count, PAGE_SIZE - count,
                           "\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
    count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

    return count;
}

static ssize_t mxt_devattr_object_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    struct mxt_object *object;
    int count = 0;
    int i, j;
    int ret_val = 0;
    u8 *obuf;

    /* Pre-allocate buffer large enough to hold max sized object. */
    obuf = kmalloc(256, GFP_KERNEL);
    if (!obuf)
    {
        return -ENOMEM;
    }

    for (i = 0; i < mxtdata->mxtinfo->object_num; i++)
    {
        object = mxtdata->object_table + i;
        switch (object->type)
        {
            case MXT_GEN_COMMANDPROCESSOR_T6:
            case MXT_GEN_POWERCONFIG_T7:
            case MXT_GEN_ACQUISITIONCONFIG_T8:
            case MXT_TOUCH_KEYARRAY_T15:
            case MXT_SPT_COMMSCONFIG_T18:
            case MXT_SPT_GPIOPWM_T19:
            case MXT_PROCI_GRIPFACE_T20:
            case MXT_PROCG_NOISE_T22:
            case MXT_TOUCH_PROXIMITY_T23:
            case MXT_PROCI_ONETOUCH_T24:
            case MXT_SPT_SELFTEST_T25:
            case MXT_PROCI_TWOTOUCH_T27:
            case MXT_SPT_CTECONFIG_T28:
            case MXT_SPT_USERDATA_T38:
            case MXT_PROCI_GRIP_T40:
            case MXT_PROCI_PALM_T41:
            case MXT_PROCI_TOUCHSUPPRESSION_T42:
            case MXT_SPT_DIGITIZER_T43:
            case MXT_SPT_CTECONFIG_T46:
            case MXT_PROCI_STYLUS_T47:
            case MXT_TOUCH_PROXKEY_T52:
            case MXT_GEN_DATASOURCE_T53:
            case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:

                count += scnprintf(buf + count, PAGE_SIZE - count, "T%u:\n", object->type);

                for (j = 0; j < mxt_get_obj_num_instances(object); j++)
                {
                    u16 size = mxt_get_obj_size(object);
                    u16 addr = object->start_address + j * size;

                    ret_val = mxt_read_blks(mxtdata, addr, size, obuf);
                    if (ret_val)
                    {
                        goto done;
                    }

                    count = mxt_show_instance(buf, count, object, j, obuf);
                }
                break;
            default:
                break;
        }
    }

done:
    kfree(obuf);
    return ret_val ?: count;
}

static int mxt_check_bootloader(struct mxt_data *mxtdata, unsigned int expected_next_state)
{
    struct device *dev = &mxtdata->spidevice->dev;
    u8 status_byte;
    int ret_val;

recheck:
    if (0 != mxt_wait_for_chg(mxtdata))
    {
        dev_err(dev, "BOOTL Timeout on CHG expected_next_state=%.2x\n", expected_next_state);
    }
    ret_val = mxt_bootloader_read(mxtdata, &status_byte, 1);
    if (0 != ret_val)
    {
        dev_err(dev, "Bootloader read FAIL\n");
        return ret_val;
    }
    dev_dbg(dev, "Bootloader status_byte %02X\n", status_byte);
    if ( (MXT_BOOT_STATUS_WAITING_FRAME_DATA == expected_next_state) ||
         (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state) )
    {
        status_byte &= ~MXT_BOOT_STATUS_WAITINGS_MASK;
    }
    else if (MXT_BOOT_STATUS_FRAME_CRC_PASS == expected_next_state)
    {
        if (MXT_BOOT_STATUS_FRAME_CRC_FAIL == status_byte)
        {
            return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
        }
        if (MXT_BOOT_STATUS_FRAME_CRC_CHECK == status_byte)
        {
            goto recheck;
        }
    }
    else
    {
        return -EINVAL;
    }

    if (status_byte != expected_next_state)
    {
        dev_info(dev, "Invalid bootloader mode state %02X", status_byte);
        if ( (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state) &&
             (MXT_BOOT_STATUS_WAITING_FRAME_DATA == status_byte) )
        {
            return MXT_ERROR_BOOTLOADER_UNLOCKED;
        }
        if (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state)
        {
            mxt_send_bootloader_reset_cmd(mxtdata);
        }
        return -EINVAL;
    }

    return 0;
}

static int mxt_check_firmware_format(struct device *dev, const struct firmware *fw)
{
    unsigned int pos = 0;
    char c;

    while (pos < fw->size)
    {
        c = *(fw->data + pos);

        if (c < '0' || (c > '9' && c < 'A') || c > 'F')
        {
            return 0;
        }

        pos++;
    }

    dev_err(dev, "Aborting: firmware file must be in binary format\n");
    dev_err(dev, "xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw\n");

    return -EINVAL;
}

static int mxt_enter_bootloader(struct mxt_data *mxtdata)
{
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (mxtdata->suspended)
    {
        if (mxtdata->mxtplatform->suspend_mode == MXT_SUSPEND_REGULATOR)
        {
            mxt_regulator_enable(mxtdata);
        }

        mxtdata->suspended = false;
    }

    if (!mxtdata->in_bootloader)
    {
        disable_irq(mxtdata->chg_irq);

        /* Change to the bootloader mode */
        ret_val = mxt_send_t6_command_processor(mxtdata, MXT_T6_CFG_RESET_OFFSET, MXT_T6_CFG_RESET_VAL_TO_BOOTL, false/*wait*/);
        if (ret_val)
        {
            return ret_val;
        }

        msleep(MXT_RESET_TIME);

        mxtdata->in_bootloader = true;
        mxt_sysfs_debug_msg_remove(mxtdata);
        mxt_sysfs_mem_access_remove(mxtdata);
        mxt_unregister_input_device(mxtdata);
        mxt_free_object_table(mxtdata);
    }

    dev_info(&mxtdata->spidevice->dev, "Entered bootloader\n");

    return 0;
}

static int mxt_send_frames(struct mxt_data *mxtdata)
{
    struct device *dev = &mxtdata->spidevice->dev;
    struct mxt_flash *mxtflash = mxtdata->mxtflash;
    int ret_val;

    dev_dbg(dev, "%s >\n", __func__);

    if (NULL == mxtflash)
    {
        dev_err(dev, "%s mxt_flash = null\n", __func__);
        return EINVAL;
    }

    ret_val = mxt_check_bootloader(mxtdata, MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD);
    if (0 == ret_val)
    {
        dev_info(dev, "Unlocking bootloader\n");
        ret_val = mxt_send_bootloader_unlock_cmd(mxtdata);
        if (0 != ret_val)
        {
            dev_err(dev, "Failure to unlock bootloader\n");
            return ret_val;
        }
        dev_info(dev, "Bootloader unlocked\n");
    }
    else if (MXT_ERROR_BOOTLOADER_UNLOCKED == ret_val)
    {
        dev_info(dev, "Bootloader already unlocked\n");
    }
    else
    {
        dev_err(dev, "Unexpected state transition\n");
        return -EINVAL;
    }

    dev_info(dev, "Flashing device...\n");

    mxtflash->frame_retry = 0;
    mxtflash->frame_count = 1;
    mxtflash->fw_pos = 0;
    while ((mxtflash->fw_pos+2) < mxtflash->fw->size)
    {
        if (0 == mxtflash->frame_retry)
        {
            mxtflash->pframe = (u8*)(mxtflash->fw->data + mxtflash->fw_pos);
            mxtflash->frame_size = *mxtflash->pframe << 8 | *(mxtflash->pframe + 1);
            dev_dbg(dev, "Frame %d: size %d\n", mxtflash->frame_count, mxtflash->frame_size);

            /* Allow for CRC bytes at end of frame */
            mxtflash->frame_size += 2;
        }

        if (0 != mxt_check_bootloader(mxtdata, MXT_BOOT_STATUS_WAITING_FRAME_DATA))
        {
            dev_err(dev, "Unexpected bootloader state\n");
            return -EINVAL;
        }

        /* Write one frame to device */
        ret_val = mxt_bootloader_write(mxtdata, mxtflash->pframe, mxtflash->frame_size);
        if (0 != ret_val)
        {
            return ret_val;
        }

        // Check CRC
        dev_dbg(dev, "Checking CRC\n");
        ret_val = mxt_check_bootloader(mxtdata, MXT_BOOT_STATUS_FRAME_CRC_PASS);
        if (MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL == ret_val)
        {
            if (mxtflash->frame_retry > 0)
            {
                dev_err(dev, "Failure sending frame %d - aborting\n", mxtflash->frame_count);
                return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
            }
            mxtflash->frame_retry++;
            dev_err(dev, "Frame %d: CRC fail, retry %d\n", mxtflash->frame_count, mxtflash->frame_retry);
        }
        else if (0 != ret_val)
        {
            dev_err(dev, "Unexpected bootloader state\n");
            return ret_val;
        }
        else
        {
            dev_dbg(dev, "CRC pass\n");
            mxtflash->frame_retry = 0;
            mxtflash->frame_count++;
            mxtflash->fw_pos += mxtflash->frame_size;
        }
    }
    dev_info(dev, "Sent %d frames, %lld bytes\n", mxtflash->frame_count, mxtflash->fw_pos);

    return 0;
}

static int mxt_load_fw(struct device *dev)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    mxtdata->mxtflash = devm_kzalloc(dev, sizeof(struct mxt_flash), GFP_KERNEL);
    if (!mxtdata->mxtflash)
    {
        return -ENOMEM;
    }

    mxtdata->mxtflash->mxtdata = mxtdata;

    ret_val = request_firmware(&mxtdata->mxtflash->fw, mxtdata->fw_name, dev);
    if (ret_val)
    {
        dev_err(dev, "request_firmware %d %s\n", ret_val, mxtdata->fw_name);
        goto free;
    }

    /* Check for incorrect enc file */
    ret_val = mxt_check_firmware_format(dev, mxtdata->mxtflash->fw);
    if (ret_val)
    {
        goto release_firmware;
    }

    if (!mxtdata->in_bootloader)
    {
        ret_val = mxt_enter_bootloader(mxtdata);
        if (ret_val)
        {
            goto release_firmware;
        }
    }

    ret_val = mxt_send_frames(mxtdata);
    if (ret_val)
    {
        goto release_firmware;
    }

    mxtdata->in_bootloader = false;

release_firmware:
    release_firmware(mxtdata->mxtflash->fw);
free:
    devm_kfree(dev, mxtdata->mxtflash);
    return ret_val;
}

static int mxt_update_file_name(struct device *dev,
                                char **out_file_name,
                                const char *in_file_name,
                                size_t in_str_len)
{
    char *file_name_tmp;

    /* Simple sanity check */
    if (in_str_len > 64)
    {
        dev_warn(dev, "File name too long\n");
        return -EINVAL;
    }

    file_name_tmp = krealloc(*out_file_name, in_str_len + 1, GFP_KERNEL);
    if (!file_name_tmp)
    {
        return -ENOMEM;
    }

    *out_file_name = file_name_tmp;
    memcpy(*out_file_name, in_file_name, in_str_len);

    /* Echo into the sysfs entry may append newline at the end of buf */
    if (in_file_name[in_str_len - 1] == '\n')
    {
        (*out_file_name)[in_str_len - 1] = '\0';
    }
    else
    {
        (*out_file_name)[in_str_len] = '\0';
    }

    return 0;
}

static ssize_t mxt_devattr_update_fw_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    int ret_val;

    ret_val = mxt_update_file_name(dev, &mxtdata->fw_name, buf, count);
    if (ret_val)
    {
        return ret_val;
    }

    ret_val = mxt_load_fw(dev);
    if (ret_val)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret_val);
        count = ret_val;
    }
    else
    {
        dev_info(dev, "The firmware update succeeded\n");

        msleep(MXT_RESET_TIME);
        mxtdata->suspended = false;

        ret_val = mxt_initialize(mxtdata);
        if (ret_val)
        {
            return ret_val;
        }
    }

    return count;
}

static ssize_t mxt_devattr_update_cfg_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf,
                                            size_t count)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    const struct mxt_platform_data *mxtplatform = mxtdata->mxtplatform;
    const struct firmware *cfg;
    int ret_val;

    ret_val = mxt_update_file_name(dev, &mxtdata->cfg_name, buf, count);
    if (ret_val)
    {
        return ret_val;
    }

    ret_val = request_firmware(&cfg, mxtdata->cfg_name, dev);
    if (ret_val)
    {
        dev_err(dev, "request_firmware %d %s\n", ret_val, mxtdata->cfg_name);
        goto out;
    }

    mxtdata->updating_config = true;

    mxt_unregister_input_device(mxtdata);

    if (mxtdata->suspended)
    {
        if (mxtplatform->suspend_mode == MXT_SUSPEND_REGULATOR)
        {
            enable_irq(mxtdata->chg_irq);
            mxt_regulator_enable(mxtdata);
        }
        else if (mxtplatform->suspend_mode == MXT_SUSPEND_DEEP_SLEEP)
        {
            mxt_set_t7_power_cfg(mxtdata, MXT_T7_POWER_CFG_RUN);
            (void)mxt_acquire_irq(mxtdata);
        }

        mxtdata->suspended = false;
    }

    ret_val = mxt_configure_objects(mxtdata, cfg);
    if (!ret_val)
    {
        // all good
        ret_val = count;
    }

    release_firmware(cfg);
out:
    mxtdata->updating_config = false;
    return ret_val;
}

static ssize_t mxt_devattr_debug_enable_show(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    char c;

    c = mxtdata->debug_enabled ? '1' : '0';
    return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_devattr_debug_notify_show(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
    return sprintf(buf, "0\n");
}

static ssize_t mxt_devattr_debug_v2_enable_store(struct device *dev,
                                                 struct device_attribute *attr,
                                                 const char *buf,
                                                 size_t count)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    u8 i;
    ssize_t ret_val;

    if (kstrtou8(buf, 0, &i) == 0 && i < 2)
    {
        if (i == 1)
        {
            mxt_debug_msg_enable(mxtdata);
        }
        else
        {
            mxt_debug_msg_disable(mxtdata);
        }

        ret_val = count;
    }
    else
    {
        dev_dbg(dev, "debug_enabled write error\n");
        ret_val = -EINVAL;
    }

    return ret_val;
}

static ssize_t mxt_devattr_debug_enable_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf,
                                              size_t count)
{
    struct mxt_data *mxtdata = dev_get_drvdata(dev);
    u8 i;
    ssize_t ret_val;

    if (kstrtou8(buf, 0, &i) == 0 && i < 2)
    {
        mxtdata->debug_enabled = (i == 1);

        dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
        ret_val = count;
    }
    else
    {
        dev_dbg(dev, "debug_enabled write error\n");
        ret_val = -EINVAL;
    }

    return ret_val;
}

static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_devattr_update_fw_store);

static struct attribute *mxt_fw_attrs[] =
{
    &dev_attr_update_fw.attr,
    NULL
};

static const struct attribute_group mxt_fw_attr_group =
{
    .attrs = mxt_fw_attrs,
};

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_devattr_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_devattr_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_devattr_object_show, NULL);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_devattr_update_cfg_store);
static DEVICE_ATTR(config_crc, S_IRUGO, mxt_devattr_config_crc_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_devattr_debug_enable_show, mxt_devattr_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL, mxt_devattr_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_devattr_debug_notify_show, NULL);

static struct attribute *mxt_attrs[] =
{
    &dev_attr_fw_version.attr,
    &dev_attr_hw_version.attr,
    &dev_attr_object.attr,
    &dev_attr_update_cfg.attr,
    &dev_attr_config_crc.attr,
    &dev_attr_debug_enable.attr,
    &dev_attr_debug_v2_enable.attr,
    &dev_attr_debug_notify.attr,
    NULL
};

static const struct attribute_group mxt_attr_group =
{
    .attrs = mxt_attrs,
};

static int mxt_sysfs_mem_access_init(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    ret_val = sysfs_create_group(&spidevice->dev.kobj, &mxt_attr_group);
    if (ret_val)
    {
        dev_err(&spidevice->dev, "Failure %d creating sysfs group\n", ret_val);
        return ret_val;
    }

    sysfs_bin_attr_init(&mxtdata->mem_access_attr);
    mxtdata->mem_access_attr.attr.name = "mem_access";
    mxtdata->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
    mxtdata->mem_access_attr.read = mxt_sysfs_mem_access_read;
    mxtdata->mem_access_attr.write = mxt_sysfs_mem_access_write;
    mxtdata->mem_access_attr.size = mxtdata->mem_size;

    ret_val = sysfs_create_bin_file(&spidevice->dev.kobj,
                                    &mxtdata->mem_access_attr);
    if (ret_val)
    {
        dev_err(&spidevice->dev, "Failed to create %s\n", mxtdata->mem_access_attr.attr.name);
        sysfs_remove_group(&spidevice->dev.kobj, &mxt_attr_group);
    }
    return ret_val;
}

static void mxt_sysfs_mem_access_remove(struct mxt_data *mxtdata)
{
    struct spi_device *spidevice = mxtdata->spidevice;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (mxtdata->mem_access_attr.attr.name)
    {
        sysfs_remove_bin_file(&spidevice->dev.kobj,
                              &mxtdata->mem_access_attr);
    }
    sysfs_remove_group(&spidevice->dev.kobj, &mxt_attr_group);
}

static void mxt_reset_slots(struct mxt_data *mxtdata)
{
    int id;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->inputdev)
    {
        return;
    }

    for (id = 0; id < mxtdata->t100_num_touchids; id++)
    {
        input_mt_slot(mxtdata->inputdev, id);
        input_mt_report_slot_state(mxtdata->inputdev, 0, false);
    }

    mxt_input_sync(mxtdata);
}

static int mxt_start(struct mxt_data *mxtdata)
{
    int ret_val;
    struct spi_device *spidevice = mxtdata->spidevice;

    dev_info(&spidevice->dev, "%s, suspend_mode %d, double_tap_enable %d >\n", __func__, mxtdata->mxtplatform->suspend_mode, mxtdata->double_tap_enable);

    if (!mxtdata->suspended || mxtdata->in_bootloader)
    {
        dev_info(&spidevice->dev, "%s, suspended %d, in_bootloader = %d <\n", __func__, mxtdata->suspended, mxtdata->in_bootloader);
        return 0;
    }

    switch (mxtdata->mxtplatform->suspend_mode)
    {
        case MXT_SUSPEND_REGULATOR:
            enable_irq(mxtdata->chg_irq);
            mxt_regulator_enable(mxtdata);
            break;

        case MXT_SUSPEND_DEEP_SLEEP:
        default:
            /*
             * Discard any touch messages still in message buffer
             * from before chip went to sleep
             */
            (void)mxt_process_messages_until_invalid(mxtdata);

            ret_val = mxt_set_t7_power_cfg(mxtdata, MXT_T7_POWER_CFG_RUN);
            if (ret_val)
            {
                return ret_val;
            }
            mxt_set_t100_multitouchscreen_cfg(mxtdata, 0, (MXT_T100_CFG_CTRL_ENABLE_BIT | MXT_T100_CFG_CTRL_RPTEN_BIT | MXT_T100_CFG_CTRL_DISSCRMSG_BIT | MXT_T100_CFG_CTRL_SCANEN_BIT));

            /* Recalibrate since chip has been in deep sleep */
            ret_val = mxt_send_t6_command_processor(mxtdata, MXT_T6_CFG_CALIBRATE_OFFSET, MXT_T6_CFG_CALIBRATE_VAL_EXEC, false/*wait*/);
            if (ret_val)
            {
                return ret_val;
            }
            if (mxtdata->t93_cfg_address)
            {
                mxt_set_t93_touchsequence_ena_dis_cfg(mxtdata, 0, MXT_T93_DISABLE);
            }
            if(mxtdata->double_tap_enable == 0)
            {
                ret_val = mxt_acquire_irq(mxtdata);
                if (ret_val)
                {
                    return ret_val;
                }
            }
            else
            {
                disable_irq_wake(mxtdata->chg_irq);
            }
            break;
    }

    mxtdata->suspended = false;

    dev_info(&spidevice->dev, "%s <\n",__func__);

    return 0;
}

static int mxt_stop(struct mxt_data *mxtdata)
{
    int ret_val;
    struct spi_device *spidevice = mxtdata->spidevice;

    dev_info(&spidevice->dev, "%s, suspend mode %d, double_tap_enable %d >\n", __func__, mxtdata->mxtplatform->suspend_mode, mxtdata->double_tap_enable);

    if (mxtdata->suspended || mxtdata->in_bootloader || mxtdata->updating_config)
    {
        dev_info(&spidevice->dev, "%s, suspended %d, in_bootloader %d, updating_config %d <\n",__func__, mxtdata->suspended, mxtdata->in_bootloader, mxtdata->updating_config);
        return 0;
    }

    switch (mxtdata->mxtplatform->suspend_mode)
    {
        case MXT_SUSPEND_REGULATOR:
            disable_irq(mxtdata->chg_irq);
            mxt_regulator_disable(mxtdata);
            mxt_reset_slots(mxtdata);
            break;

        case MXT_SUSPEND_DEEP_SLEEP:
        default:
            disable_irq(mxtdata->chg_irq);

            if(mxtdata->double_tap_enable == 1)
            {
                mxt_set_t93_touchsequence_ena_dis_cfg(mxtdata, 0, MXT_T93_ENABLE);
                mxt_set_t7_power_cfg(mxtdata, MXT_T7_POWER_CFG_GESTURE);
                //mxt_wake_irq_enable(mxtdata);
            }
            else
            {
                ret_val = mxt_set_t7_power_cfg(mxtdata, MXT_T7_POWER_CFG_DEEPSLEEP);
                if (ret_val)
                {
                    return ret_val;
                }
            }
            mxt_set_t100_multitouchscreen_cfg(mxtdata, 0, (MXT_T100_CFG_CTRL_ENABLE_BIT | MXT_T100_CFG_CTRL_DISSCRMSG_BIT | MXT_T100_CFG_CTRL_SCANEN_BIT));
            mxt_reset_slots(mxtdata);
            if(mxtdata->double_tap_enable == 1)
            {
                enable_irq_wake(mxtdata->chg_irq);
                enable_irq(mxtdata->chg_irq);
            }
            break;
    }

    mxtdata->suspended = true;

    dev_info(&spidevice->dev, "%s <\n",__func__);

    return 0;
}

static int mxt_input_open(struct input_dev *inputdev)
{
    struct mxt_data *mxtdata = input_get_drvdata(inputdev);
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    ret_val = mxt_start(mxtdata);

    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "%s failed rc=%d\n", __func__, ret_val);
    }

    return ret_val;
}

static void mxt_input_close(struct input_dev *inputdev)
{
    struct mxt_data *mxtdata = input_get_drvdata(inputdev);
    int ret_val;

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    ret_val = mxt_stop(mxtdata);

    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "%s failed rc=%d\n", __func__, ret_val);
    }
}

#ifdef CONFIG_OF // Open Firmware (Device Tree)
static const struct mxt_platform_data *mxt_platform_data_get_from_device_tree(struct spi_device *spidevice)
{
    struct mxt_platform_data *mxtplatform;
    struct device_node *devnode = spidevice->dev.of_node;
    u32 *keymap;
    int proplen, ret_val;

    if (!devnode)
    {
        return ERR_PTR(-ENOENT); /* no device tree device */
    }

    mxtplatform = devm_kzalloc(&spidevice->dev, sizeof(struct mxt_platform_data), GFP_KERNEL);
    if (!mxtplatform)
    {
        return ERR_PTR(-ENOMEM);
    }

    mxtplatform->gpio_reset = of_get_named_gpio_flags(devnode, "atmel,reset-gpio", 0, NULL);
    mxtplatform->gpio_chg_irq = of_get_named_gpio_flags(devnode, "atmel,irq-gpio", 0, NULL);

    ret_val = of_property_read_string(devnode, "atmel,cfg_name", &mxtplatform->cfg_name);
    if (ret_val)
    {
        dev_err(&spidevice->dev,  "Couldn't read atmel,cfg_name: %d\n", ret_val);
    }

    ret_val = of_property_read_string(devnode, "atmel,fw_name", &mxtplatform->fw_name);
    if (ret_val)
    {
        dev_err(&spidevice->dev,  "Couldn't read atmel,fw_name: %d\n", ret_val);
    }

    of_property_read_string(devnode, "atmel,input_name", &mxtplatform->input_name);

    if (of_find_property(devnode, "linux,gpio-keymap", &proplen))
    {
        mxtplatform->t19_gpio_num_keys = proplen / sizeof(u32);

        keymap = devm_kzalloc(&spidevice->dev,
                              mxtplatform->t19_gpio_num_keys * sizeof(keymap[0]),
                              GFP_KERNEL);
        if (!keymap)
        {
            return ERR_PTR(-ENOMEM);
        }

        ret_val = of_property_read_u32_array(devnode, "linux,gpio-keymap", keymap, mxtplatform->t19_gpio_num_keys);
        if (ret_val)
        {
            dev_warn(&spidevice->dev, "Couldn't read linux,gpio-keymap: %d\n", ret_val);
        }

        mxtplatform->t19_gpio_keymap = keymap;
    }

    of_property_read_u32(devnode, "atmel,suspend-mode", &mxtplatform->suspend_mode);

    dev_dbg(&spidevice->dev, "%s <\n", __func__);

    return mxtplatform;
}
#else // CONFIG_OF
static const struct mxt_platform_data *mxt_platform_data_get_from_device_tree(struct spi_device *spidevice)
{
    return ERR_PTR(-ENOENT);
}
#endif // CONFIG_OF

#ifdef CONFIG_ACPI // Advanced Configuration and Power Interface
struct mxt_acpi_platform_data
{
    const char *hid;
    struct mxt_platform_data mxtplatform;
};

static unsigned int samus_touchpad_buttons[] =
{
    KEY_RESERVED,
    KEY_RESERVED,
    KEY_RESERVED,
    BTN_LEFT
};

static struct mxt_acpi_platform_data samus_platform_data[] =
{
    {
        /* Touchpad */
        .hid = "ATML0000",
        .mxtplatform = {
            .t19_gpio_num_keys = ARRAY_SIZE(samus_touchpad_buttons),
            .t19_gpio_keymap = samus_touchpad_buttons,
        },
    },
    {
        /* Touchscreen */
        .hid    = "ATML0001",
    },
    { }
};

static unsigned int chromebook_tp_buttons[] =
{
    KEY_RESERVED,
    KEY_RESERVED,
    KEY_RESERVED,
    KEY_RESERVED,
    KEY_RESERVED,
    BTN_LEFT
};

static struct mxt_acpi_platform_data chromebook_platform_data[] =
{
    {
        /* Touchpad */
        .hid    = "ATML0000",
        .mxtplatform  = {
            .t19_gpio_num_keys   = ARRAY_SIZE(chromebook_tp_buttons),
            .t19_gpio_keymap = chromebook_tp_buttons,
        },
    },
    {
        /* Touchscreen */
        .hid    = "ATML0001",
    },
    { }
};

static const struct dmi_system_id mxt_dmi_table[] =
{
    {
        /* 2015 Google Pixel */
        .ident = "Chromebook Pixel 2",
        .matches = {
            DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
            DMI_MATCH(DMI_PRODUCT_NAME, "Samus"),
        },
        .driver_data = samus_platform_data,
    },
    {
        /* Other Google Chromebooks */
        .ident = "Chromebook",
        .matches = {
            DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
        },
        .driver_data = chromebook_platform_data,
    },
    { }
};

static const struct mxt_platform_data *mxt_platform_data_get_from_acpi(struct spi_device *spidevice)
{
    struct acpi_device *adev;
    const struct dmi_system_id *system_id;
    const struct mxt_acpi_platform_data *acpi_pdata;

    adev = ACPI_COMPANION(&spidevice->dev);
    if (!adev)
    {
        return ERR_PTR(-ENOENT);
    }

    system_id = dmi_first_match(mxt_dmi_table);
    if (!system_id)
    {
        return ERR_PTR(-ENOENT);
    }

    acpi_pdata = system_id->driver_data;
    if (!acpi_pdata)
    {
        return ERR_PTR(-ENOENT);
    }

    while (acpi_pdata->hid)
    {
        if (0 == strcmp(acpi_device_hid(adev), acpi_pdata->hid))
        {
            dev_dbg(&spidevice->dev, "%s <\n", __func__);
            return &acpi_pdata->mxtplatform;
        }

        acpi_pdata++;
    }

    return ERR_PTR(-ENOENT);
}
#else // CONFIG_ACPI
static const struct mxt_platform_data *mxt_platform_data_get_from_acpi(struct spi_device *spidevice)
{
    return ERR_PTR(-ENOENT);
}
#endif // CONFIG_ACPI

static struct mxt_platform_data *mxt_platform_data_get_default(struct spi_device *spidevice)
{
    struct mxt_platform_data *mxtplatform = devm_kzalloc(&spidevice->dev, sizeof(struct mxt_platform_data), GFP_KERNEL);
    if (!mxtplatform)
    {
        return ERR_PTR(-ENOMEM);
    }

    /* Set default parameters */

    dev_dbg(&spidevice->dev, "%s <\n", __func__);

    return mxtplatform;
}

static const struct mxt_platform_data * mxt_platform_data_get(struct spi_device *spidevice)
{
    const struct mxt_platform_data *mxtplatform;

    dev_dbg(&spidevice->dev, "%s >\n", __func__);

    mxtplatform = dev_get_platdata(&spidevice->dev);
    if (mxtplatform)
    {
        return mxtplatform;
    }

    mxtplatform = mxt_platform_data_get_from_device_tree(spidevice);
    if (!IS_ERR(mxtplatform) || PTR_ERR(mxtplatform) != -ENOENT)
    {
        return mxtplatform;
    }

    mxtplatform = mxt_platform_data_get_from_acpi(spidevice);
    if (!IS_ERR(mxtplatform) || PTR_ERR(mxtplatform) != -ENOENT)
    {
        return mxtplatform;
    }

    mxtplatform = mxt_platform_data_get_default(spidevice);
    if (!IS_ERR(mxtplatform))
    {
        return mxtplatform;
    }

    dev_err(&spidevice->dev, "No platform data specified\n");
    return ERR_PTR(-EINVAL);
}

static int mxt_gpio_setup(struct mxt_data *mxtdata)
{
    int ret_val;

    ret_val = gpio_request(mxtdata->mxtplatform->gpio_chg_irq, "irq-gpio");
    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "gpio_request %lu (%d)", mxtdata->mxtplatform->gpio_chg_irq, ret_val);
        return ret_val;
    }
    ret_val = gpio_direction_input(mxtdata->mxtplatform->gpio_chg_irq);
    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "gpio_direction_input %lu (%d)", mxtdata->mxtplatform->gpio_chg_irq, ret_val);
        return ret_val;
    }
    dev_dbg(&mxtdata->spidevice->dev, "gpio_chg_irq %lu IN %d\n", mxtdata->mxtplatform->gpio_chg_irq, mxt_read_chg(mxtdata));

    ret_val = gpio_request(mxtdata->mxtplatform->gpio_reset, "reset-gpio");
    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "gpio_request %lu (%d)", mxtdata->mxtplatform->gpio_reset, ret_val);
        return ret_val;
    }
    ret_val = gpio_direction_output(mxtdata->mxtplatform->gpio_reset, GPIO_RESET_NO_HIGH);
    if (ret_val)
    {
        dev_err(&mxtdata->spidevice->dev, "gpio_direction_output %lu (%d)", mxtdata->mxtplatform->gpio_reset, ret_val);
        return ret_val;
    }
    dev_dbg(&mxtdata->spidevice->dev, "gpio_reset %lu OUT %d\n", mxtdata->mxtplatform->gpio_reset, GPIO_RESET_NO_HIGH);

    return 0;
}

static int mxt_probe(struct spi_device *spidevice)
{
    struct mxt_data *mxtdata;
    const struct mxt_platform_data *mxtplatform;
    int ret_val;

    dev_dbg(&spidevice->dev, "%s >>>\n", __func__);

    if ( (spidevice->bits_per_word && spidevice->bits_per_word != 8) ||
         !(spidevice->mode & SPI_CPHA) ||
         !(spidevice->mode & SPI_CPOL) )
    {
        dev_err(&spidevice->dev, "unexpected spi device setup\n");
        return -EINVAL;
    }

    mxtplatform = mxt_platform_data_get(spidevice);
    if (IS_ERR(mxtplatform))
    {
        return PTR_ERR(mxtplatform);
    }

    mxtdata = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
    if (!mxtdata)
    {
        return -ENOMEM;
    }
    snprintf(mxtdata->phys, sizeof(mxtdata->phys), "spi-%d/input0", spidevice->master->bus_num);
    dev_info(&spidevice->dev, "%s %s\n", __func__, mxtdata->phys);

    mxtdata->spidevice = spidevice;
    mxtdata->mxtplatform = mxtplatform;
    spi_set_drvdata(spidevice, mxtdata);

    if (mxtdata->mxtplatform->cfg_name)
    {
        mxt_update_file_name(&mxtdata->spidevice->dev,
                             &mxtdata->cfg_name,
                             mxtdata->mxtplatform->cfg_name,
                             strlen(mxtdata->mxtplatform->cfg_name));
    }

    if (mxtdata->mxtplatform->fw_name)
    {
        mxt_update_file_name(&mxtdata->spidevice->dev,
                             &mxtdata->fw_name,
                             mxtdata->mxtplatform->fw_name,
                             strlen(mxtdata->mxtplatform->fw_name));
    }

    dev_info(&spidevice->dev, "%s mxtdata->fw_name: %s, mxtdata->cfg_name: %s \n", __func__, mxtdata->fw_name, mxtdata->cfg_name);

    init_completion(&mxtdata->chg_completion);
    init_completion(&mxtdata->reset_completion);
    init_completion(&mxtdata->crc_completion);
    mutex_init(&mxtdata->debug_msg_lock);

    ret_val = mxt_gpio_setup(mxtdata);
    if (ret_val)
    {
        goto err_free_irq;
    }

    //Enable regulator even if suspend mode = deepsleep
    if (mxtplatform->suspend_mode == MXT_SUSPEND_REGULATOR || mxtplatform->suspend_mode == MXT_SUSPEND_DEEP_SLEEP)
    {
        ret_val = mxt_acquire_irq(mxtdata);
        if (ret_val)
        {
            goto err_free_mem;
        }

        ret_val = mxt_probe_regulators(mxtdata);
        if (ret_val)
        {
            goto err_free_irq;
        }

        disable_irq(mxtdata->chg_irq);
    }

    ret_val = sysfs_create_group(&spidevice->dev.kobj, &mxt_fw_attr_group);
    if (ret_val)
    {
        dev_err(&spidevice->dev, "Failure %d creating fw sysfs group\n", ret_val);
        return ret_val;
    }

    ret_val = mxt_initialize(mxtdata);
    if (ret_val)
    {
        goto err_free_irq;
    }

    dev_info(&spidevice->dev, "%s <\n", __func__);
    return 0;

err_free_irq:
    if (mxtdata->chg_irq)
    {
        free_irq(mxtdata->chg_irq, mxtdata);
    }

    gpio_free(mxtdata->mxtplatform->gpio_reset);
    gpio_free(mxtdata->mxtplatform->gpio_chg_irq);
    if(mxtdata->reg_vdd)
    {
        regulator_put(mxtdata->reg_vdd);
    }
    if(mxtdata->reg_avdd)
    {
        regulator_put(mxtdata->reg_avdd);
    }
err_free_mem:
    kfree(mxtdata);
    dev_info(&spidevice->dev, "%s error\n", __func__);
    return ret_val;
}

static int mxt_remove(struct spi_device *spidevice)
{
    struct mxt_data *mxtdata = spi_get_drvdata(spidevice);
    dev_info(&spidevice->dev, "%s >\n", __func__);

    sysfs_remove_group(&spidevice->dev.kobj, &mxt_fw_attr_group);
    mxt_sysfs_debug_msg_remove(mxtdata);
    mxt_sysfs_mem_access_remove(mxtdata);

    if (mxtdata->chg_irq)
    {
        free_irq(mxtdata->chg_irq, mxtdata);
    }

    gpio_free(mxtdata->mxtplatform->gpio_reset);
    gpio_free(mxtdata->mxtplatform->gpio_chg_irq);

    if(mxtdata->reg_avdd)
    {
        regulator_put(mxtdata->reg_avdd);
    }
    if(mxtdata->reg_vdd)
    {
        regulator_put(mxtdata->reg_vdd);
    }
    mxt_unregister_input_device(mxtdata);
    mxt_free_object_table(mxtdata);
    kfree(mxtdata);

    return 0;
}

static int __maybe_unused mxt_suspend(struct device *dev)
{
    struct spi_device *spidevice = to_spi_device(dev);
    struct mxt_data *mxtdata = spi_get_drvdata(spidevice);

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->inputdev)
    {
        return 0;
    }

    mutex_lock(&mxtdata->inputdev->mutex);

    if (mxtdata->inputdev->users)
    {
        (void)mxt_stop(mxtdata);
    }

    mutex_unlock(&mxtdata->inputdev->mutex);

    return 0;
}

static int __maybe_unused mxt_resume(struct device *dev)
{
    struct spi_device *spidevice = to_spi_device(dev);
    struct mxt_data *mxtdata = spi_get_drvdata(spidevice);

    dev_dbg(&mxtdata->spidevice->dev, "%s >\n", __func__);

    if (!mxtdata->inputdev)
    {
        return 0;
    }

    mutex_lock(&mxtdata->inputdev->mutex);

    if (mxtdata->inputdev->users)
    {
        (void)mxt_start(mxtdata);
    }

    mutex_unlock(&mxtdata->inputdev->mutex);

    return 0;
}

static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

#ifdef CONFIG_OF // Open Firmware (Device Tree)
static const struct of_device_id mxt_of_match[] =
{
    { .compatible = "atmel,maxtouch", },
    {},
};
MODULE_DEVICE_TABLE(of, mxt_of_match);
#endif // CONFIG_OF

#ifdef CONFIG_ACPI // Advanced Configuration and Power Interface
static const struct acpi_device_id mxt_acpi_id[] =
{
    { "ATML0000", 0 },  /* Touchpad */
    { "ATML0001", 0 },  /* Touchscreen */
    { }
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_id);
#endif // CONFIG_ACPI

static const struct spi_device_id mxt_id[] =
{
    { "qt602240_ts", 0 },
    { "atmel_mxt_ts", 0 },
    { "atmel_mxt_tp", 0 },
    { "maxtouch", 0 },
    { "mXT224", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, mxt_id);

static struct spi_driver mxt_driver =
{
    .driver = {
        .name   = "atmel_mxt_ts",
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(mxt_of_match),
        .acpi_match_table = ACPI_PTR(mxt_acpi_id),
        .pm = &mxt_pm_ops,
    },
    .probe      = mxt_probe,
    .remove     = mxt_remove,
    .id_table   = mxt_id,
};

module_spi_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Solomon Systech maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
