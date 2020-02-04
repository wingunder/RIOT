/*
 * Copyright (C) 2020 Pieter du Preez <pdupreez@gmail.com>
 *               2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#define ENABLE_DEBUG        (1)
#include "debug.h"

#include "assert.h"
#include "periph/common_bus.h"

#include <errno.h>
#include <byteorder.h>

/* flag to enable address auto incrementation on read or write */
#define COMMON_BUS_SPI_FLAG_AINC  (0x40)
#define COMMON_BUS_I2C_FLAG_AINC  (0x80)

#define COMMON_BUS_I2C_DEV_OFFSET 0
#define COMMON_BUS_SPI_DEV_OFFSET (I2C_NUMOF)
#define COMMON_BUS_DEV_NUMOF      (I2C_NUMOF + SPI_NUMOF)

typedef uint8_t common_bus_cmd_funct_t(size_t bus_handle, common_bus_rw_args_t *args);

typedef struct {
    common_bus_rw_args_t   args; /** command arguments */
    common_bus_cmd_funct_t *func; /** command function pointer */
} common_bus_cmd_t;

typedef struct {
    size_t cnt;
    common_bus_cmd_t *list[COMMON_BUS_CMD_LIST_NUMOF];
} _common_bus_cmd_list_t;

#if defined MODULE_PERIPH_SPI || defined MODULE_PERIPH_I2C
static _common_bus_cmd_list_t _common_bus_cmd_list[COMMON_BUS_DEV_NUMOF];
static common_bus_setup_t     _common_bus_setups[COMMON_BUS_DEV_NUMOF];
#endif

#ifdef MODULE_PERIPH_SPI

#include <periph/gpio.h>

/* flag to set when reading from the device */
#define COMMON_BUS_SPI_FLAG_READ  (0x80)

static inline void _check_spi_bus_handle(int bus_handle)
{
    assert(bus_handle >= (int)COMMON_BUS_SPI_DEV_OFFSET &&
           bus_handle < (int)(COMMON_BUS_SPI_DEV_OFFSET + SPI_NUMOF));
}

static uint8_t _common_bus_spi_read_regs(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] spi_read_regs bus_handle=%d\n", bus_handle);
    _check_spi_bus_handle(bus_handle);
    common_bus_spi_t *ptr = &_common_bus_setups[bus_handle].bus.spi;

    //uint8_t reg = args->read_regs.reg;
    DEBUG("[common_bus] read_regs bus=%d reg=0x%02x len=%d\n", ptr->dev, args->reg, args->len);
    uint8_t res = spi_transfer_reg(ptr->dev, ptr->cs, (COMMON_BUS_SPI_FLAG_READ | args->reg), 0);
    DEBUG("[common_bus] read_regs res=0x%02x\n", res);

//    uint8_t reg = COMMON_BUS_SPI_FLAG_READ | args->read_regs.reg;
//    if (args->read_regs.len > 1) {
//        reg |= COMMON_BUS_SPI_FLAG_AINC;
//    }
//    spi_transfer_bytes(bus->spi.dev, bus->spi.cs, true, &reg, NULL, 1);
//    spi_transfer_bytes(bus->spi.dev, bus->spi.cs, false,
//                       NULL, args->read_regs.data, args->read_regs.len);
//    DEBUG("[common_bus] read_regs data[0]=0x%02x\n", (uint8_t)(*args->read_regs.data));
    return COMMON_BUS_OK;
}

static uint8_t _common_bus_spi_read_bytes(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] spi_read_regs bus_handle=%d\n", bus_handle);
    _check_spi_bus_handle(bus_handle);
    common_bus_spi_t *ptr = &_common_bus_setups[bus_handle].bus.spi;

    spi_transfer_bytes(ptr->dev, ptr->cs, false, NULL, args->data, args->len);
    return COMMON_BUS_OK;
}

static uint8_t _common_bus_spi_write_regs(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] spi_write_regs bus_handle=%d\n", bus_handle);
    _check_spi_bus_handle(bus_handle);
    common_bus_spi_t *ptr = &_common_bus_setups[bus_handle].bus.spi;

    uint8_t reg8 = args->reg;
    if (args->len > 1) {
        args->reg |= COMMON_BUS_SPI_FLAG_AINC;
    }
    spi_transfer_bytes(ptr->dev, ptr->cs, true, &reg8, NULL, 1);
    spi_transfer_bytes(ptr->dev, ptr->cs, false, args->data, NULL, args->len);
    return COMMON_BUS_OK;
}

static uint8_t _common_bus_spi_write_bytes(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] spi_write_bytes bus_handle=%d\n", bus_handle);
    _check_spi_bus_handle(bus_handle);
    common_bus_spi_t *ptr = &_common_bus_setups[bus_handle].bus.spi;

    spi_transfer_bytes(ptr->dev, ptr->cs, false, args->data, NULL, args->len);
    return COMMON_BUS_OK;
}

static int _common_bus_spi_run(size_t bus_handle)
{
    DEBUG("[common_bus] spi_run acquire bus_handle=%d\n", bus_handle);
    _check_spi_bus_handle(bus_handle);
    common_bus_spi_t *ptr = &_common_bus_setups[bus_handle].bus.spi;
    int ret = spi_acquire(ptr->dev, ptr->cs, ptr->mode, ptr->clk);
    if (ret == SPI_OK) {
        int cmd_ret;
        _common_bus_cmd_list_t *dev_cmd_list = &_common_bus_cmd_list[bus_handle];
        for (size_t i=0; i<dev_cmd_list->cnt; i++) {
            common_bus_cmd_t *cmd = dev_cmd_list->list[i];
            cmd_ret = cmd->func(bus_handle, &cmd->args);
            if (cmd_ret != COMMON_BUS_OK) {
                break;
            }
        }
        spi_release(ptr->dev);
        DEBUG("[common_bus] spi_run release bus=%d\n", ptr->dev);
        ret = COMMON_BUS_OK;
    }
    else if (ret == SPI_NOMODE) {
        DEBUG("[common_bus] spi_run no mode bus=%d ret=%d\n", ptr->dev, ret);
        ret = COMMON_BUS_ERR_SPI_NOMODE;
    }
    else if (ret == SPI_NOCLK) {
        DEBUG("[common_bus] spi_run no clock bus=%d ret=%d\n", ptr->dev, ret);
        ret = COMMON_BUS_ERR_SPI_NOCLK;
    }
    else {
        DEBUG("[common_bus] spi_run failed bus=%d ret=%d\n", ptr->dev, ret);
        ret = COMMON_BUS_ERR_UNKNOWN;
    }
    return ret;
}

int common_bus_spi_init(size_t dev_num, int8_t cs_port, uint8_t cs_pin,
                        spi_mode_t mode, spi_clk_t clk)
{
    int bus_handle = COMMON_BUS_SPI_DEV_OFFSET + dev_num;
    DEBUG("[common_bus] spi_init dev_num=%d bus_handle=%d\n", dev_num, bus_handle);
    assert(dev_num < SPI_NUMOF);

    common_bus_setup_t *ptr = &_common_bus_setups[bus_handle];
    ptr->bus.spi.dev = SPI_DEV(dev_num);
    ptr->bus.spi.cs_port = cs_port;
    ptr->bus.spi.cs_pin = cs_pin;
    ptr->bus.spi.mode = mode;
    ptr->bus.spi.clk = clk;
    ptr->type = COMMON_BUS_SPI;
    ptr->f.run = _common_bus_spi_run;
    ptr->f.read_regs = _common_bus_spi_read_regs;
    ptr->f.read_bytes = _common_bus_spi_read_bytes;
    ptr->f.write_regs = _common_bus_spi_write_regs;
    ptr->f.write_bytes = _common_bus_spi_write_bytes;

    spi_init(ptr->bus.spi.dev);
    if (cs_port == -1) {
        DEBUG("[common_bus] init using hardware chip select\n");
        ptr->bus.spi.cs = SPI_HWCS(cs_pin);
    }
    else {
        DEBUG("[common_bus] init using chip select port=%d pin=%d\n",
              cs_port, cs_pin);
        ptr->bus.spi.cs = GPIO_PIN(cs_port, cs_pin);
    }

    int ret = spi_init_cs(ptr->bus.spi.dev, ptr->bus.spi.cs);
    if (ret == SPI_OK) {
        int ret = spi_acquire(ptr->bus.spi.dev, ptr->bus.spi.cs,
                              ptr->bus.spi.mode, ptr->bus.spi.clk);
        if (ret == SPI_OK) {
            spi_release(ptr->bus.spi.dev);
            return bus_handle;
        }
    }
    return COMMON_BUS_ERR_INIT_FAILED;
}

#endif

#ifdef MODULE_PERIPH_I2C

static inline void _check_i2c_bus_handle(int bus_handle)
{
    assert(bus_handle >= (int)COMMON_BUS_I2C_DEV_OFFSET &&
           bus_handle < (int)(COMMON_BUS_I2C_DEV_OFFSET + I2C_NUMOF));
}

static uint8_t _common_bus_i2c_read_regs(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] i2c_read_regs bus_handle=%d\n", bus_handle);
    _check_i2c_bus_handle(bus_handle);
    if (args->flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }

    uint8_t reg = args->reg;
    if (args->len > 1) {
        reg |= COMMON_BUS_I2C_FLAG_AINC;
    }

    uint16_t reg_end;
    /* Handle endianness of register if 16 bit */
    if (args->flags & I2C_REG16) {
        /* Make sure register is in big-endian on I2C bus */
        reg_end = htons(reg);
    }
    else {
        reg_end = reg;
    }

    /* First set ADDR and register with no stop */
    common_bus_i2c_t *ptr = &_common_bus_setups[bus_handle].bus.i2c;
    int ret = i2c_write_bytes(ptr->dev, ptr->addr,
                              &reg_end, (args->flags & I2C_REG16) ? 2 : 1,
                              args->flags | I2C_NOSTOP);
    if (ret < 0) {
        return ret;
    }
    /* Then get the data from device */
    return i2c_read_bytes(ptr->dev, ptr->addr, args->data, args->len, args->flags);
}

static uint8_t _common_bus_i2c_read_bytes(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] i2c_read_bytes bus_handle=%d\n", bus_handle);
    _check_i2c_bus_handle(bus_handle);
    common_bus_i2c_t *ptr = &_common_bus_setups[bus_handle].bus.i2c;
    return i2c_read_bytes(ptr->dev, ptr->addr, args->data, args->len, args->flags);
}

static uint8_t _common_bus_i2c_write_regs(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] i2c_write_regs bus_handle=%d\n", bus_handle);
    _check_i2c_bus_handle(bus_handle);
    uint8_t reg = args->reg;
    if (args->len > 1) {
        reg |= COMMON_BUS_I2C_FLAG_AINC;
    }
    common_bus_i2c_t *ptr = &_common_bus_setups[bus_handle].bus.i2c;
    return i2c_write_regs(ptr->dev, ptr->addr, reg, args->data, args->len, args->flags);
}

static uint8_t _common_bus_i2c_write_bytes(size_t bus_handle, common_bus_rw_args_t *args)
{
    DEBUG("[common_bus] i2c_write_bytes bus_handle=%d\n", bus_handle);
    _check_i2c_bus_handle(bus_handle);
    common_bus_i2c_t *ptr = &_common_bus_setups[bus_handle].bus.i2c;
    return i2c_write_bytes(ptr->dev, ptr->addr, args->data, args->len, args->flags);
}

static int _common_bus_i2c_run(size_t bus_handle)
{
    DEBUG("[common_bus] i2c_run bus_handle=%d\n", bus_handle);
    _check_i2c_bus_handle(bus_handle);
    common_bus_i2c_t *ptr = &_common_bus_setups[bus_handle].bus.i2c;
    int ret = i2c_acquire(ptr->dev);
    if (ret == 0) {
        int cmd_ret;
        _common_bus_cmd_list_t *dev_cmd_list = &_common_bus_cmd_list[bus_handle];
        for (size_t i=0; i<dev_cmd_list->cnt; i++) {
            common_bus_cmd_t *cmd = dev_cmd_list->list[i];
            cmd_ret = cmd->func(bus_handle, &cmd->args);
            if (cmd_ret != COMMON_BUS_OK) {
                break;
            }
        }
        i2c_release(ptr->dev);
        DEBUG("[common_bus] i2c_run release bus=%d\n", ptr->dev);
        ret = COMMON_BUS_OK;
    }
    else {
        ret = COMMON_BUS_ERR_ACQUIRE;
    }
    return ret;
}

int common_bus_i2c_init(size_t dev_num, int8_t addr)
{
    int bus_handle = COMMON_BUS_I2C_DEV_OFFSET + dev_num;
    DEBUG("[common_bus] i2c_init dev_num=%d bus_handle=%d\n", dev_num, bus_handle);
    assert(dev_num < I2C_NUMOF);

    common_bus_setup_t *ptr = &_common_bus_setups[bus_handle];
    ptr->bus.i2c.dev = I2C_DEV(dev_num);
    ptr->bus.i2c.addr = addr;
    ptr->type = COMMON_BUS_I2C;
    ptr->f.run = _common_bus_i2c_run;
    ptr->f.read_regs = _common_bus_i2c_read_regs;
    ptr->f.read_bytes = _common_bus_i2c_read_bytes;
    ptr->f.write_regs = _common_bus_i2c_write_regs;
    ptr->f.write_bytes = _common_bus_i2c_write_bytes;
    return bus_handle;
}

#endif

static inline void _check_bus_handle(int bus_handle)
{
    assert(bus_handle < (int)COMMON_BUS_DEV_NUMOF);
}

static common_bus_cmd_t *_common_bus_cmd_get_cmd_ptr(int bus_handle)
{
    _check_bus_handle(bus_handle);
    _common_bus_cmd_list_t *dev_cmd_list = &_common_bus_cmd_list[bus_handle];
    assert(dev_cmd_list->cnt < COMMON_BUS_CMD_LIST_NUMOF);
    common_bus_cmd_t *cmd = dev_cmd_list->list[dev_cmd_list->cnt];
    dev_cmd_list->cnt++;
    return cmd;
}

void common_bus_add_read_regs(size_t bus_handle, uint16_t reg, uint8_t *data,
                              size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_read_regs bus_handle=%d\n", bus_handle);
    common_bus_cmd_t *cmd = _common_bus_cmd_get_cmd_ptr(bus_handle);
    cmd->func = _common_bus_setups[bus_handle].f.read_regs;
    cmd->args.reg = reg;
    cmd->args.data = data;
    cmd->args.len = len;
    cmd->args.flags = flags;
}

void common_bus_add_read_bytes(size_t bus_handle, uint8_t *data,
                               size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_read_bytes bus_handle=%d\n", bus_handle);
    common_bus_cmd_t *cmd = _common_bus_cmd_get_cmd_ptr(bus_handle);
    cmd->func = _common_bus_setups[bus_handle].f.read_bytes;
    cmd->args.data = data;
    cmd->args.len = len;
    cmd->args.flags = flags;
}

void common_bus_add_write_regs(size_t bus_handle, uint16_t reg, uint8_t *data,
                               size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_write_regs bus_handle=%d\n", bus_handle);
    common_bus_cmd_t *cmd = _common_bus_cmd_get_cmd_ptr(bus_handle);
    cmd->func = _common_bus_setups[bus_handle].f.write_regs;
    cmd->args.reg = reg;
    cmd->args.data = data;
    cmd->args.len = len;
    cmd->args.flags = flags;
}

void common_bus_add_write_bytes(size_t bus_handle, uint8_t *data,
                                size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_write_bytes bus_handle=%d\n", bus_handle);
    common_bus_cmd_t *cmd = _common_bus_cmd_get_cmd_ptr(bus_handle);
    cmd->func = _common_bus_setups[bus_handle].f.write_bytes;
    cmd->args.data = data;
    cmd->args.len = len;
    cmd->args.flags = flags;
}

int common_bus_run(size_t bus_handle)
{
    DEBUG("[common_bus] run bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].f.run(bus_handle);
}

void common_bus_clear(size_t bus_handle)
{
    DEBUG("[common_bus] clear bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    _common_bus_cmd_list[bus_handle].cnt = 0;
}

#ifdef CONFIG_PERIPH_COMMON_DEBUG

common_bus_type_t common_bus_get_type(size_t bus_handle)
{
    DEBUG("[common_bus] get_type bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].type;
}

spi_t common_bus_get_spi_dev(size_t bus_handle)
{
    DEBUG("[common_bus] get_spi_dev bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.spi.dev;
}

int8_t common_bus_get_spi_cs_port(size_t bus_handle)
{
    DEBUG("[common_bus] get_spi_cs_port bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.spi.cs_port;
}

uint8_t common_bus_get_spi_cs_pin(size_t bus_handle)
{
    DEBUG("[common_bus] get_spi_cs_pin bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.spi.cs_pin;
}

spi_mode_t common_bus_get_spi_mode(size_t bus_handle)
{
    DEBUG("[common_bus] get_spi_mode bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.spi.mode;
}

spi_clk_t common_bus_get_spi_clk(size_t bus_handle)
{
    DEBUG("[common_bus] get_spi_clk bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.spi.clk;
}


i2c_t common_bus_get_i2c_dev(size_t bus_handle)
{
    DEBUG("[common_bus] get_i2c_dev bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.i2c.dev;
}

uint8_t common_bus_get_i2c_addr(size_t bus_handle)
{
    DEBUG("[common_bus] get_i2c_addr bus_handle=%d\n", bus_handle);
    _check_bus_handle(bus_handle);
    return _common_bus_setups[bus_handle].bus.i2c.addr;
}

#endif
