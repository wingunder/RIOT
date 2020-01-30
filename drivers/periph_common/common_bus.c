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

#include <periph/common_bus.h>

/* flag to enable address auto incrementation on read or write */
#define COMMON_BUS_FLAG_AINC (0x40)

#ifdef MODULE_PERIPH_SPI

#include <periph/gpio.h>

/* flag to set when reading from the device */
#define COMMON_BUS_FLAG_READ (0x80)

static int _spi_init(const common_bus_params_t *bus)
{
    spi_init(bus->spi.dev);
    if (bus->spi.cs != GPIO_UNDEF && spi_init_cs(bus->spi.dev, bus->spi.cs) != SPI_OK) {
        return SPI_NOCS;
    }
    return SPI_OK;
}

static int _spi_add(const common_bus_params_t *bus,
                    const common_bus_reg_cmd_t *cmd)
{
    return spi_acquire(bus->spi.dev, bus->spi.cs, bus->spi.mode, bus->spi.clk);
}

static void _spi_run(const common_bus_params_t *bus)
{
    spi_release(bus->spi.dev);
    spi_transfer_regs(bus->spi.dev, bus->spi.cs,
                      (COMMON_BUS_FLAG_READ | (uint8_t)reg), NULL, out, 1);
    DEBUG("[_spi_read_reg] reg 0x%02x, val 0x%02x\n", reg, *out);
    spi_transfer_regs(bus->spi.dev, bus->spi.cs,
                      (COMMON_BUS_FLAG_READ | COMMON_BUS_FLAG_AINC | (uint8_t)reg), NULL, data, len);
}

static void _spi_clear(const common_bus_params_t *bus, uint16_t reg, uint8_t *out)
{
}

#endif

#ifdef MODULE_PERIPH_I2C

static int _i2c_init(const common_bus_params_t *bus)
{
    (void)(bus->i2c.dev);
    return 0;
}

static int _i2c_run(const common_bus_params_t *bus)
{
    return i2c_acquire(bus->i2c.dev);
}

static void _i2c_add(const common_bus_params_t *bus)
{
    i2c_release(bus->i2c.dev);
}

static int _i2c_run(const common_bus_params_t *bus)
{
    return i2c_read_reg(bus->i2c.dev, bus->i2c.addr, reg, out, 0);
    DEBUG("[_i2c_write_reg] write: reg 0x%02x, val 0x%02x\n", (int)reg, (int)data);
    return i2c_write_reg(bus->i2c.dev, bus->i2c.addr, reg, data, 0);
}

static void _i2c_clear(const common_bus_params_t *bus)
{

}

#endif

void common_bus_setup(common_bus_setup_t* setup)
{
    switch (setup->type) {
#ifdef MODULE_PERIPH_SPI
    case COMMON_BUS_SPI:
        setup->f.init = _spi_init;
        setup->f.add = _spi_acquire;
        setup->f.run = _spi_release;
        setup->f.clear = _spi_read_reg;
        break;
#endif
#ifdef MODULE_PERIPH_I2C
    case COMMON_BUS_I2C:
        setup->f.init = _i2c_init;
        setup->f.add = _i2c_acquire;
        setup->f.run = _i2c_release;
        setup->f.clear = _i2c_read_reg;
        break;
#endif
    default:
        setup->f.init = NULL;
        setup->f.add = NULL;
        setup->f.run = NULL;
        setup->f.clear = NULL;
    }
}
