/*
 * Copyright (C) 2020 Pieter du Preez <pdupreez@gmail.com>
 *               2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_periph_common_bus (I2C/SPI)
 * @ingroup     drivers_periph
 * @brief       Low-level I2C/SPI bus driver interface
 *
 * This is a kind of router for drivers that are able to support
 * either I2C or API bus interfaces.
 *
 * It is required to call the common_bus_setup function as start-up.
 * This function sets up function pointers for directing bus
 * communication to the desired bus.
 *
 * @{
 *
 * @file
 * @brief       Low-level I2C/SPI bus driver interface definition
 *
 * @author      Pieter du Preez <pdupreez@gmail.com>
 */

#ifndef PERIPH_COMMON_BUS_H
#define PERIPH_COMMON_BUS_H

//#ifdef MODULE_PERIPH_SPI
#include "spi.h"
//#endif

//#ifdef MODULE_PERIPH_I2C
#include "i2c.h"
//#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum number of commands available
 */
#define COMMON_BUS_CMD_LIST_NUMOF 8

/**
 * @brief   Common bus return codes
 */
typedef enum {
    COMMON_BUS_OK          =  0,  /**< everything was fine */
    COMMON_BUS_ERR_NODEV       = -1,  /**< unable to talk to device */
    COMMON_BUS_ERR_SPI_NOCS    = -2,  /**< invalid chip select line specified */
    COMMON_BUS_ERR_SPI_NOMODE  = -3,  /**< selected mode is not supported */
    COMMON_BUS_ERR_SPI_NOCLK   = -4,  /**< selected clock value is not supported */
    COMMON_BUS_ERR_NOBUS       = -10, /**< bus interface error */
    COMMON_BUS_ERR_UNKNOWN     = -11, /**< unknown error */
    COMMON_BUS_ERR_CMD_LIST_RANGE = -12 , /**< index to command list is out of range */
    COMMON_BUS_ERR_ACQUIRE = -13, /**< acquiring the bus failed */
    COMMON_BUS_ERR_INIT_FAILED = -14,
} common_bus_return_code_t;

/**
 * @brief   Supported bus types.
 */
typedef enum {
    COMMON_BUS_UNDEF = 0,   /** Undefined bus type */
    COMMON_BUS_I2C,         /** I2C bus type */
    COMMON_BUS_SPI,         /** SPI bus type */
} common_bus_type_t;

#ifdef MODULE_PERIPH_SPI
typedef struct  {
    spi_t dev;              /** The device */
    gpio_t cs;              /** Chip select */
    int8_t cs_port;              /** Chip select */
    uint8_t cs_pin;              /** Chip select */
    spi_mode_t mode;        /** The mode */
    spi_clk_t clk;          /** Clock speed */
} common_bus_spi_t;
#endif

#ifdef MODULE_PERIPH_I2C
typedef struct {
    i2c_t dev;              /** The device */
    uint8_t addr;           /** The address */
} common_bus_i2c_t;
#endif

/**
 * @brief   A union of bus parameter types
 */
typedef union
{
#ifdef MODULE_PERIPH_SPI
    common_bus_spi_t spi;          /** SPI parameters */
#endif
#ifdef MODULE_PERIPH_I2C
    common_bus_i2c_t i2c;          /** I2C parameters */
#endif
    unsigned int dev;       /** The device place holder */
} common_bus_params_t;

typedef struct {
    uint16_t reg;
    uint8_t  *data;
    size_t   len;
    uint16_t flags;
} common_bus_rw_args_t;

/**
 * @brief   Run register commands function typedef
 *
 * @param[in] bus       bus parameters
 */
typedef uint8_t common_bus_rw_func_t(size_t bus_handle, common_bus_rw_args_t *args);

typedef int common_bus_run_t(size_t bus_handle);

/**
 * @brief   Function pointer structure for pivoting to a specified bus.
 */
typedef struct {
    common_bus_rw_func_t *read_regs;
    common_bus_rw_func_t *write_regs;
    common_bus_rw_func_t *read_bytes;
    common_bus_rw_func_t *write_bytes;
    common_bus_run_t     *run; /** run register commands function */
} common_bus_function_t;

/**
 * @brief   The setup struct contains the bus type, the bus
 *          and the common bus function pointers.
 */
typedef struct
{
    common_bus_type_t type;  /** The transport type */
    common_bus_params_t bus; /** The bus parameters */
    common_bus_function_t f; /** The function pointers */
} common_bus_setup_t;

/**
 * @brief   Function that must be called at start-up.
 */
int common_bus_spi_init(size_t bus_handle, int8_t cs_port, uint8_t cs_pin,
                        spi_mode_t mode, spi_clk_t clk);

int common_bus_i2c_init(size_t dev_num, int8_t addr);

void common_bus_add_read_regs(size_t bus_handle, uint16_t reg, uint8_t *data,
                              size_t len, uint16_t flags);

void common_bus_add_read_bytes(size_t bus_handle, uint8_t *data,
                               size_t len, uint16_t flags);

void common_bus_add_write_regs(size_t bus_handle, uint16_t reg, uint8_t *data,
                               size_t len, uint16_t flags);

void common_bus_add_write_bytes(size_t bus_handle, uint8_t *data,
                                size_t len, uint16_t flags);

int common_bus_run(size_t bus_handle);

void common_bus_clear(size_t bus_handle);

#ifdef CONFIG_PERIPH_COMMON_DEBUG

common_bus_type_t common_bus_get_type(size_t bus_handle);

spi_t common_bus_get_spi_dev(size_t bus_handle);
int8_t common_bus_get_spi_cs_port(size_t bus_handle);
uint8_t common_bus_get_spi_cs_pin(size_t bus_handle);
spi_mode_t common_bus_get_spi_mode(size_t bus_handle);
spi_clk_t common_bus_get_spi_clk(size_t bus_handle);

i2c_t common_bus_get_i2c_dev(size_t bus_handle);
uint8_t common_bus_get_i2c_addr(size_t bus_handle);

#endif

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_COMMON_BUS_H */
/** @} */
