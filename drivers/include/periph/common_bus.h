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

#ifdef MODULE_SOFT_SPI
#include "soft_spi.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum number of common bus devices
 */
#define COMMON_BUS_DEV_NUMOF           (I2C_NUMOF + SPI_NUMOF + SOFT_SPI_NUMOF)
/**
 * @brief   I2C device offset
 */
#define COMMON_BUS_I2C_DEV_OFFSET      0
/**
 * @brief   SPI device offset
 */
#define COMMON_BUS_SPI_DEV_OFFSET      (I2C_NUMOF)
/**
 * @brief   SOFT_SPI device offset
 */
#define COMMON_BUS_SOFT_SPI_DEV_OFFSET (I2C_NUMOF + SPI_NUMOF)

/**
 * @brief   Common bus return codes
 */
typedef enum {
    COMMON_BUS_OK           =  0,   /**< success */
} common_bus_return_code_t;

/**
 * @brief   Supported bus types.
 */
typedef enum {
    COMMON_BUS_UNDEF = 0,   /**< Undefined bus type */
    COMMON_BUS_I2C,         /**< I2C bus type */
    COMMON_BUS_SPI,         /**< SPI bus type */
    COMMON_BUS_SOFT_SPI,    /**< Soft SPI bus type */
} common_bus_type_t;

#ifdef MODULE_PERIPH_SPI
typedef struct  {
    spi_t dev;              /**< The device */
    gpio_t cs;              /**< Chip select */
    int8_t cs_port;         /**< Chip select */
    uint8_t cs_pin;         /**< Chip select */
    spi_mode_t mode;        /**< The mode */
    spi_clk_t clk;          /**< Clock speed */
} common_bus_spi_t;
#endif

#ifdef MODULE_SOFT_SPI
typedef struct  {
    soft_spi_t dev;         /**< The device */
    gpio_t cs;              /**< Chip select */
    int8_t cs_port;         /**< Chip select */
    uint8_t cs_pin;         /**< Chip select */
    soft_spi_mode_t mode;   /**< The mode */
    soft_spi_clk_t clk;     /**< Clock speed */
} common_bus_soft_spi_t;
#endif

#ifdef MODULE_PERIPH_I2C
typedef struct {
    i2c_t dev;              /**< The device */
    uint8_t addr;           /**< The address */
} common_bus_i2c_t;
#endif

/**
 * @brief   A union of bus parameter types
 */
typedef union
{
#ifdef MODULE_PERIPH_SPI
    common_bus_spi_t spi;           /**< SPI parameters */
#endif
#ifdef MODULE_PERIPH_I2C
    common_bus_i2c_t i2c;           /**< I2C parameters */
#endif
#ifdef MODULE_SOFT_SPI
    common_bus_soft_spi_t soft_spi; /**< Soft SPI parameters */
#endif
    unsigned int dummy;             /**< Allow compilation, when no peripherals were enabled. */
} common_bus_params_t;

/**
 * @brief   Common bus read regs function typedef
 *
 * @param[in] ptr          parameter pointer
 * @param[in] reg          register
 * @param[out] data        data read
 * @param[in] len          requested data length
 * @param[in] flags        flags (used in I2C mode)
 */
typedef int common_bus_read_regs_t(const common_bus_params_t *ptr, uint16_t reg,
                                   uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Common bus read bytes function typedef
 *
 * @param[in] ptr          parameter pointer
 * @param[out] data        data read
 * @param[in] len          requested data length
 * @param[in] flags        flags (used in I2C mode)
 */
typedef int common_bus_read_bytes_t(const common_bus_params_t *ptr,
                                    uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Common bus write regs function typedef
 *
 * @param[in] ptr          parameter pointer
 * @param[in] reg          register
 * @param[in] data         data to write
 * @param[in] len          requested data length
 * @param[in] flags        flags (used in I2C mode)
 */
typedef int common_bus_write_regs_t(const common_bus_params_t *ptr, uint16_t reg,
                                    const uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Common bus write bytes function typedef
 *
 * @param[in] ptr          parameter pointer
 * @param[in] data         data to write
 * @param[in] len          requested data length
 * @param[in] flags        flags (used in I2C mode)
 */
typedef int common_bus_write_bytes_t(const common_bus_params_t *ptr,
                                     const uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Common bus acquire function typedef
 *
 * @param[in] ptr          parameter pointer
 */
typedef int common_bus_acquire_t(const common_bus_params_t *ptr);

/**
 * @brief   Common bus release function typedef
 *
 * @param[in] ptr          parameter pointer
 */
typedef void common_bus_release_t(const common_bus_params_t *ptr);

/**
 * @brief   Function pointer structure for pivoting to a specified bus.
 */
typedef struct {
    common_bus_read_regs_t    *read_regs;    /**< read registers funtion pointer */
    common_bus_write_regs_t   *write_regs;   /**< write registers funtion pointer */
    common_bus_read_bytes_t   *read_bytes;   /**< read bytes funtion pointer */
    common_bus_write_bytes_t  *write_bytes;  /**< write bytes funtion pointer */
    common_bus_acquire_t      *acquire;      /**< acquire bus funtion pointer */
    common_bus_release_t      *release;      /**< release bus funtion pointer */
} common_bus_function_t;

/**
 * @brief   The context struct contains the bus type, the bus
 *          and the common bus function pointers.
 */
typedef struct
{
    bool initialized;       /**< A flag for checking initialization status */
    common_bus_type_t type;  /**< The transport type */
    common_bus_params_t bus; /**< The bus parameters */
    common_bus_function_t f; /**< The function pointers */
} common_bus_context_t;

/**
 * @brief   Common bus init function
 */
void common_bus_init(void);

/**
 * @brief   SPI init function
 * @param[in] dev_num      device number
 * @param[in] cs_port      chip select port
 * @param[in] cs_pin       chip select pin
 * @param[in] mode         SPI mode
 * @param[in] clk          SPI clock speed
 * @return                 the bus handle or error
 */
int common_bus_spi_init(size_t dev_num, int8_t cs_port, uint8_t cs_pin,
                        spi_mode_t mode, spi_clk_t clk);

/**
 * @brief   Soft SPI init function
 * @param[in] dev_num      device number
 * @param[in] cs_port      chip select port
 * @param[in] cs_pin       chip select pin
 * @param[in] mode         SPI mode
 * @param[in] clk          SPI clock speed
 * @return                 the bus handle or error
 */
int common_bus_soft_spi_init(size_t dev_num, int8_t cs_port, uint8_t cs_pin,
                             soft_spi_mode_t mode, soft_spi_clk_t clk);

/**
 * @brief   I2C init function
 * @param[in] dev_num      device number
 * @param[in] addr         I2C address
 * @return                 error code
 */
int common_bus_i2c_init(size_t dev_num, int8_t addr);

/**
 * @brief   Read registers command
 * @param[in] bus_handle   bus handle
 * @param[in] reg          register
 * @param[out] data        data pointer
 * @param[in] len          data len
 * @param[in] flags        flags
 * @return                 error code
 */
int common_bus_read_regs(int bus_handle, uint16_t reg,
                         uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Read bytes command
 * @param[in] bus_handle   bus handle
 * @param[out] data        data pointer
 * @param[in] len          data len
 * @param[in] flags        flags
 * @return                 error code
 */
int common_bus_read_bytes(int bus_handle,
                          uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Add write registers command
 * @param[in] bus_handle   bus handle
 * @param[in] reg          register
 * @param[in] data         data pointer
 * @param[in] len          data len
 * @param[in] flags        flags
 * @return                 error code
 */
int common_bus_write_regs(int bus_handle, uint16_t reg,
                          uint8_t *data, size_t len, uint16_t flags);

/**
 * @brief   Write bytes command
 * @param[in] bus_handle   bus handle
 * @param[out] data        data pointer
 * @param[in] len          data len
 * @param[in] flags        flags
 * @return                 error code
 */
int common_bus_write_bytes(int bus_handle,
                           uint8_t *data, size_t len, uint16_t flags);

#ifdef CONFIG_PERIPH_COMMON_DEBUG

/**
 * @brief   Get type command
 * @param[in] bus_handle   bus handle
 * @return                 common bus type
 */
common_bus_type_t common_bus_get_type(int bus_handle);

/**
 * @brief   Get SPI device number
 * @param[in] dev_num      device number
 * @return                 SPI device number
 */
spi_t common_bus_get_spi_dev(int bus_handle);

/**
 * @brief   Get SPI device number
 * @param[in] bus_handle   bus handle
 * @return  SPI SPI chip select port
 */
int8_t common_bus_get_spi_cs_port(int bus_handle);

/**
 * @brief   Get SPI device number
 * @param[in] bus_handle   bus handle
 * @return  SPI SPI chip select pin
 */
uint8_t common_bus_get_spi_cs_pin(int bus_handle);

spi_mode_t common_bus_get_spi_mode(int bus_handle);
spi_clk_t common_bus_get_spi_clk(int bus_handle);

/**
 * @brief   Get SOFT_SPI device number
 * @param[in] bus_handle   bus handle
 * @return  SOFT_SPI device number
 */
soft_spi_t common_bus_get_soft_spi_dev(int bus_handle);

/**
 * @brief   Get SOFT_SPI device number
 * @param[in] bus_handle   bus handle
 * @return  SOFT_SPI SOFT_SPI chip select port
 */
int8_t common_bus_get_soft_spi_cs_port(int bus_handle);

/**
 * @brief   Get SOFT_SPI device number
 * @param[in] bus_handle   bus handle
 * @return  SOFT_SPI SOFT_SPI chip select pin
 */
uint8_t common_bus_get_soft_spi_cs_pin(int bus_handle);

soft_spi_mode_t common_bus_get_soft_spi_mode(int bus_handle);
soft_spi_clk_t common_bus_get_soft_spi_clk(int bus_handle);

i2c_t common_bus_get_i2c_dev(int bus_handle);
uint8_t common_bus_get_i2c_addr(int bus_handle);

#endif

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_COMMON_BUS_H */
/** @} */
