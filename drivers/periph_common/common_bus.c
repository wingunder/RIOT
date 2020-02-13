/*
 * Copyright (C) 2020 Pieter du Preez <pdupreez@gmail.com>
 *               2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#define ENABLE_DEBUG        (0)
#include "debug.h"

#include "assert.h"
#include "periph/common_bus.h"

#include <errno.h>
#include <byteorder.h>

/* flag to set when reading from the device */
#define COMMON_BUS_SPI_FLAG_READ  (0x80)

/* flag to enable address auto incrementation on read or write */
#define COMMON_BUS_SPI_FLAG_AINC  (0x40)
#define COMMON_BUS_I2C_FLAG_AINC  (0x80)

#if defined MODULE_PERIPH_SPI || defined MODULE_PERIPH_I2C || defined MODULE_SOFT_SPI
static common_bus_context_t _common_bus_setups[COMMON_BUS_DEV_NUMOF];
#endif

#ifdef MODULE_PERIPH_SPI

#include <periph/gpio.h>

static inline void _spi_check_handle(int bus_handle)
{
    assert(bus_handle >= (int)COMMON_BUS_SPI_DEV_OFFSET &&
           bus_handle < (int)(COMMON_BUS_SPI_DEV_OFFSET + SPI_NUMOF));
}

static int _spi_read_regs(const common_bus_params_t *ptr, uint16_t reg,
                          uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    uint8_t reg8 = COMMON_BUS_SPI_FLAG_READ | reg;
    if (len > 1) {
        reg8 |= COMMON_BUS_SPI_FLAG_AINC;
    }
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, true, &reg8, NULL, 1);
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, false, NULL, data, len);
    return COMMON_BUS_OK;
}

static int _spi_read_bytes(const common_bus_params_t *ptr,
                           uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, false, NULL, data, len);
    return COMMON_BUS_OK;
}

static int _spi_write_regs(const common_bus_params_t *ptr, uint16_t reg,
                           const uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    uint8_t reg8 = reg;
    if (len > 1) {
        reg8 |= COMMON_BUS_SPI_FLAG_AINC;
    }
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, true, &reg8, NULL, 1);
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, false, data, NULL, len);
    return COMMON_BUS_OK;
}

static int _spi_write_bytes(const common_bus_params_t *ptr,
                            const uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    spi_transfer_bytes(ptr->spi.dev, ptr->spi.cs, false, data, NULL, len);
    return COMMON_BUS_OK;
}

static int _spi_acquire(const common_bus_params_t *ptr)
{
    return spi_acquire(ptr->spi.dev, ptr->spi.cs, ptr->spi.mode, ptr->spi.clk);
}

static void _spi_release(const common_bus_params_t *ptr)
{
    spi_release(ptr->spi.dev);
}

int common_bus_spi_init(size_t dev_num, int8_t cs_port, uint8_t cs_pin,
                        spi_mode_t mode, spi_clk_t clk)
{
    const int bus_handle = COMMON_BUS_SPI_DEV_OFFSET + dev_num;
    DEBUG("[common_bus] spi_init dev_num=%d bus_handle=%d\n", dev_num, bus_handle);
    assert(dev_num < SPI_NUMOF);

    common_bus_spi_t *ctx = &_common_bus_setups[bus_handle].bus.spi;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->mode = mode;
    ctx->clk = clk;

    spi_init(ctx->dev);
    if (cs_port == -1) {
        DEBUG("[common_bus] init using hardware chip select\n");
        ctx->cs = SPI_HWCS(ctx->cs_pin);
    }
    else {
        DEBUG("[common_bus] init using chip select port=%d pin=%d\n",
              ctx->cs_port, ctx->cs_pin);
        ctx->cs = GPIO_PIN(ctx->cs_port, ctx->cs_pin);
    }

    int ret = spi_init_cs(ctx->dev, ctx->cs);

#if (ENABLE_DEBUG != 0)
    if (ret == SPI_OK) {
        ret = spi_acquire(ctx->dev, ctx->cs, ctx->mode, ctx->clk);
        if (ret == SPI_OK) {
            spi_release(ctx->dev);
            return bus_handle;
        }
    }
#endif
    return ret;
}

#endif

#ifdef MODULE_SOFT_SPI

#include <periph/gpio.h>

static inline void _soft_spi_check_handle(int bus_handle)
{
    assert(bus_handle >= (int)COMMON_BUS_SOFT_SPI_DEV_OFFSET &&
           bus_handle < (int)(COMMON_BUS_SOFT_SPI_DEV_OFFSET + SOFT_SPI_NUMOF));
}

static int _soft_spi_read_regs(const common_bus_params_t *ptr, uint16_t reg,
                               uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    uint8_t reg8 = COMMON_BUS_SPI_FLAG_READ | reg;
    if (len > 1) {
        reg8 |= COMMON_BUS_SPI_FLAG_AINC;
    }
    soft_spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            true, &reg8, NULL, 1);
    soft_spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            false, NULL, data, len);
    return COMMON_BUS_OK;
}

static int _soft_spi_read_bytes(const common_bus_params_t *ptr,
                                uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    soft_spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            false, NULL, data, len);
    return COMMON_BUS_OK;
}

static int _soft_spi_write_regs(const common_bus_params_t *ptr, uint16_t reg,
                                const uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    uint8_t reg8 = reg;
    if (len > 1) {
        reg8 |= COMMON_BUS_SPI_FLAG_AINC;
    }
    soft_spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            true, &reg8, NULL, 1);
    soft_spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            false, data, NULL, len);
    return COMMON_BUS_OK;
}

static int _soft_spi_write_bytes(const common_bus_params_t *ptr,
                                 const uint8_t *data, size_t len, uint16_t flags)
{
    (void)flags;
    spi_transfer_bytes(ptr->soft_spi.dev, ptr->soft_spi.cs,
                       false, data, NULL, len);
    return COMMON_BUS_OK;
}

static int _soft_spi_acquire(const common_bus_params_t *ptr)
{
    return soft_spi_acquire(ptr->soft_spi.dev, ptr->soft_spi.cs,
                            ptr->soft_spi.mode, ptr->soft_spi.clk);
}

static void _soft_spi_release(const common_bus_params_t *ptr)
{
    spi_release(ptr->soft_spi.dev);
}

int common_bus_soft_spi_init(size_t dev_num, int8_t cs_port, uint8_t cs_pin,
                             soft_spi_mode_t mode, soft_spi_clk_t clk)
{
    const int bus_handle = COMMON_BUS_SPI_DEV_OFFSET + dev_num;
    DEBUG("[common_bus] spi_init dev_num=%d bus_handle=%d\n", dev_num, bus_handle);
    assert(dev_num < SPI_NUMOF);

    common_bus_soft_spi_t *ctx = &_common_bus_setups[bus_handle].bus.soft_spi;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->mode = mode;
    ctx->clk = clk;

    soft_spi_init(ctx->dev);
    if (cs_port == -1) {
        DEBUG("[common_bus] init using hardware chip select\n");
        ctx->cs = SPI_HWCS(ctx->cs_pin);
    }
    else {
        DEBUG("[common_bus] init using chip select port=%d pin=%d\n",
              ctx->cs_port, ctx->cs_pin);
        ctx->cs = GPIO_PIN(ctx->cs_port, ctx->cs_pin);
    }

    int ret = soft_spi_init_cs(ctx->dev, ctx->cs);

#if (ENABLE_DEBUG != 0)
    if (ret == SPI_OK) {
        ret = soft_spi_acquire(ctx->dev, ctx->cs, ctx->mode, ctx->clk);
        if (ret == SPI_OK) {
            soft_spi_release(ctx->dev);
            return bus_handle;
        }
    }
#endif
    return ret;
}

#endif

#ifdef MODULE_PERIPH_I2C

static inline void _i2c_check_handle(int bus_handle)
{
    assert(bus_handle >= (int)COMMON_BUS_I2C_DEV_OFFSET &&
           bus_handle < (int)(COMMON_BUS_I2C_DEV_OFFSET + I2C_NUMOF));
}

static int _i2c_read_regs(const common_bus_params_t *ptr, uint16_t reg,
                          uint8_t *data, size_t len, uint16_t flags)
{
    if (flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }

    if (len > 1) {
        reg |= COMMON_BUS_I2C_FLAG_AINC;
    }

    uint16_t reg_end;
    /* Handle endianness of register if 16 bit */
    if (flags & I2C_REG16) {
        /* Make sure register is in big-endian on I2C bus */
        reg_end = htons(reg);
    }
    else {
        reg_end = reg;
    }

    /* First set ADDR and register with no stop */
    int ret = i2c_write_bytes(ptr->i2c.dev, ptr->i2c.addr,
                              &reg_end, (flags & I2C_REG16) ? 2 : 1,
                              flags | I2C_NOSTOP);
    if (ret >= 0) {
        /* Now get the data from device */
        ret = i2c_read_bytes(ptr->i2c.dev, ptr->i2c.addr, data, len, flags);
    }
    return ret;
}

static int _i2c_read_bytes(const common_bus_params_t *ptr,
                           uint8_t *data, size_t len, uint16_t flags)
{
    return i2c_read_bytes(ptr->i2c.dev, ptr->i2c.addr, data, len, flags);
}

static int _i2c_write_regs(const common_bus_params_t *ptr, uint16_t reg,
                           const uint8_t *data, size_t len, uint16_t flags)
{
    if (len > 1) {
        reg |= COMMON_BUS_I2C_FLAG_AINC;
    }
    return i2c_write_regs(ptr->i2c.dev, ptr->i2c.addr, reg, data, len, flags);
}

static int _i2c_write_bytes(const common_bus_params_t *ptr,
                            const uint8_t *data, size_t len, uint16_t flags)
{
    return i2c_write_bytes(ptr->i2c.dev, ptr->i2c.addr, data, len, flags);
}

static int _i2c_acquire(const common_bus_params_t *ptr)
{
    return i2c_acquire(ptr->i2c.dev);
}

static void _i2c_release(const common_bus_params_t *ptr)
{
    i2c_release(ptr->i2c.dev);
}

int common_bus_i2c_init(size_t dev_num, int8_t addr)
{
    const int bus_handle = COMMON_BUS_I2C_DEV_OFFSET + dev_num;
    DEBUG("[common_bus] i2c_init dev_num=%d bus_handle=%d\n", dev_num, bus_handle);
    assert(dev_num < I2C_NUMOF);

    common_bus_i2c_t *ctx = &_common_bus_setups[bus_handle].bus.i2c;
    ctx->dev = I2C_DEV(dev_num);
    ctx->addr = addr;

    int ret = 0;
#if (ENABLE_DEBUG != 0)
    ret = i2c_acquire(ctx->dev);
    if (ret == 0) {
        i2c_release(ctx->dev);
        return bus_handle;
    }
#endif
    return ret;
}

#endif

static common_bus_context_t *_get_context_ptr(int bus_handle)
{
    assert(bus_handle < (int)COMMON_BUS_DEV_NUMOF);
    common_bus_context_t *ctx = &_common_bus_setups[bus_handle];
    ctx->f.check_handle(bus_handle);
    return ctx;
}

void common_bus_init(void)
{
    for (size_t i=0; i<I2C_NUMOF; i++) {
        common_bus_context_t *ctx = &_common_bus_setups[COMMON_BUS_I2C_DEV_OFFSET+i];
        ctx->bus.i2c.dev = I2C_DEV(i);
        ctx->bus.i2c.addr = 0x00;
        ctx->type = COMMON_BUS_I2C;
        ctx->f.check_handle = _i2c_check_handle;
        ctx->f.read_regs = _i2c_read_regs;
        ctx->f.read_bytes = _i2c_read_bytes;
        ctx->f.write_regs = _i2c_write_regs;
        ctx->f.write_bytes = _i2c_write_bytes;
        ctx->f.acquire = _i2c_acquire;
        ctx->f.release = _i2c_release;
    }
    for (size_t i=0; i<SPI_NUMOF; i++) {
        common_bus_context_t *ctx = &_common_bus_setups[COMMON_BUS_SPI_DEV_OFFSET+i];
        ctx->bus.spi.dev = SPI_DEV(i);
        ctx->bus.spi.cs_port = 0;
        ctx->bus.spi.cs_pin = 0;
        ctx->bus.spi.mode = 0;
        ctx->bus.spi.clk = 0;
        ctx->type = COMMON_BUS_SPI;
        ctx->f.check_handle = _spi_check_handle;
        ctx->f.read_regs = _spi_read_regs;
        ctx->f.read_bytes = _spi_read_bytes;
        ctx->f.write_regs = _spi_write_regs;
        ctx->f.write_bytes = _spi_write_bytes;
        ctx->f.acquire = _spi_acquire;
        ctx->f.release = _spi_release;
    }
    for (size_t i=0; i<SOFT_SPI_NUMOF; i++) {
        common_bus_context_t *ctx = &_common_bus_setups[COMMON_BUS_SOFT_SPI_DEV_OFFSET+i];
        ctx->bus.soft_spi.dev = SOFT_SPI_DEV(i);
        ctx->bus.soft_spi.cs_port = 0;
        ctx->bus.soft_spi.cs_pin = 0;
        ctx->bus.soft_spi.mode = 0;
        ctx->bus.soft_spi.clk = 0;
        ctx->type = COMMON_BUS_SOFT_SPI;
        ctx->f.check_handle = _soft_spi_check_handle;
        ctx->f.read_regs = _soft_spi_read_regs;
        ctx->f.read_bytes = _soft_spi_read_bytes;
        ctx->f.write_regs = _soft_spi_write_regs;
        ctx->f.write_bytes = _soft_spi_write_bytes;
        ctx->f.acquire = _soft_spi_acquire;
        ctx->f.release = _soft_spi_release;
    }
}

int common_bus_read_regs(int bus_handle, uint16_t reg,
                         uint8_t *data, size_t len, uint16_t flags)
{
    DEBUG("[common_bus] read_regs bus_handle=%d\n", bus_handle);
    common_bus_context_t *ctx = _get_context_ptr(bus_handle);
    int ret = ctx->f.acquire(&ctx->bus);
    if (ret == 0) {
        ret = ctx->f.read_regs(&ctx->bus, reg, data, len, flags);
        ctx->f.release(&ctx->bus);
    }
    return ret;
}

int common_bus_read_bytes(int bus_handle,
                          uint8_t *data, size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_read_bytes bus_handle=%d\n", bus_handle);
    common_bus_context_t *ctx = _get_context_ptr(bus_handle);
    int ret = ctx->f.acquire(&ctx->bus);
    if (ret == 0) {
        ret = ctx->f.read_bytes(&ctx->bus, data, len, flags);
        ctx->f.release(&ctx->bus);
    }
    return ret;
}

int common_bus_add_write_regs(int bus_handle, uint16_t reg,
                              uint8_t *data, size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_write_regs bus_handle=%d\n", bus_handle);
    common_bus_context_t *ctx = _get_context_ptr(bus_handle);
    int ret = ctx->f.acquire(&ctx->bus);
    if (ret == 0) {
        ret = ctx->f.write_regs(&ctx->bus, reg, data, len, flags);
        ctx->f.release(&ctx->bus);
    }
    return ret;
}

int common_bus_add_write_bytes(int bus_handle,
                               uint8_t *data, size_t len, uint16_t flags)
{
    DEBUG("[common_bus] add_write_bytes bus_handle=%d\n", bus_handle);
    common_bus_context_t *ctx = _get_context_ptr(bus_handle);
    int ret = ctx->f.acquire(&ctx->bus);
    if (ret == 0) {
        ret = ctx->f.write_bytes(&ctx->bus, data, len, flags);
        ctx->f.release(&ctx->bus);
    }
    return ret;
}

#ifdef CONFIG_PERIPH_COMMON_DEBUG

common_bus_type_t common_bus_get_type(int bus_handle)
{
    return _get_context_ptr(bus_handle)->type;
}

spi_t common_bus_get_spi_dev(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.spi.dev;
}

int8_t common_bus_get_spi_cs_port(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.spi.cs_port;
}

uint8_t common_bus_get_spi_cs_pin(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.spi.cs_pin;
}

spi_mode_t common_bus_get_spi_mode(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.spi.mode;
}

spi_clk_t common_bus_get_spi_clk(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.spi.clk;
}

soft_spi_t common_bus_get_soft_spi_dev(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.soft_spi.dev;
}

int8_t common_bus_get_soft_spi_cs_port(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.soft_spi.cs_port;
}

uint8_t common_bus_get_soft_spi_cs_pin(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.soft_spi.cs_pin;
}

soft_spi_mode_t common_bus_get_soft_spi_mode(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.soft_spi.mode;
}

soft_spi_clk_t common_bus_get_soft_spi_clk(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.soft_spi.clk;
}

i2c_t common_bus_get_i2c_dev(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.i2c.dev;
}

uint8_t common_bus_get_i2c_addr(int bus_handle)
{
    return _get_context_ptr(bus_handle)->bus.i2c.addr;
}

#endif
