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

#ifdef MODULE_PERIPH_SPI
#include "spi.h"
#endif

#ifdef MODULE_PERIPH_I2C
#include "i2c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Common bus return codes
 */
enum {
    COMMON_BUS_OK    =  0,  /**< everything was fine */
    COMMON_BUS_NOBUS = -1,  /**< bus interface error */
    COMMON_BUS_NODEV = -2,  /**< unable to talk to device */
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
    spi_mode_t mode;        /** The mode */
    spi_clk_t clk;          /** Clock speed */
} spi_bus_t;
#endif

#ifdef MODULE_PERIPH_I2C
typedef struct {
    i2c_t dev;              /** The device */
    uint8_t addr;           /** The address */
} i2c_bus_t;
#endif

typedef enum {
    COMMON_BUS_REG_CMD_READ = 0,  /** A read command */
    COMMON_BUS_REG_CMD_WRITE = 1, /** A write command */
} common_bus_reg_cmd_type_t;

typedef struct {
    common_bus_reg_cmd_type_t type; /** command type */
    uint8_t *reg;                  /** register */
    uint8_t reg_len;               /** register length*/
    uint8_t *res;                  /** result */
    uint8_t res_len;               /** result length*/
} common_bus_reg_cmd_t;

/**
 * @brief   A union of bus parameter types
 */
typedef union
{
#ifdef MODULE_PERIPH_SPI
    spi_bus_t spi;          /** SPI parameters */
#endif
#ifdef MODULE_PERIPH_I2C
    i2c_bus_t i2c;          /** I2C parameters */
#endif
    unsigned int dev;       /** The device place holder */
} common_bus_params_t;

/**
 * @brief   Bus initialization function typedef
 *
 * @param[in] bus       bus parameters
 */
typedef int common_bus_init_t(const common_bus_params_t *bus);

/**
 * @brief   Add register command function typedef
 *
 * @param[in] bus       bus parameters
 * @param[in] cmd       command parameters
 */
typedef int common_bus_add_reg_cmd_t(const common_bus_params_t *bus,
                                     const common_bus_reg_cmd_t *cmd);

/**
 * @brief   Run register commands function typedef
 *
 * @param[in] bus       bus parameters
 */
typedef int common_bus_run_reg_cmds_t(const common_bus_params_t *bus);

/**
 * @brief   Clear register command list function typedef
 *
 * @param[in] bus       bus parameters
 */
typedef void common_bus_clear_cmds_t(const common_bus_params_t *bus);

/**
 * @brief   Function pointer structure for pivoting to a specified bus.
 */
typedef struct {
    common_bus_init_t *init;        /** init function */
    common_bus_add_reg_cmd_t *add;  /** add register command function */
    common_bus_run_reg_cmds_t *run; /** run register commands function */
    common_bus_clear_cmds_t *clear; /** clear register commands function */
} common_bus_function_t;

/**
 * @brief   The transport struct contains the bus type, the bus
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
void common_bus_setup(common_bus_setup_t* setup);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_COMMON_BUS_H */
/** @} */
