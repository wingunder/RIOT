/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the low-level common bus
 *              (I2C or SPI) peripheral driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kevin Weiss <kevin.weiss@haw-hamburg.de>
 * @author      Pieter du Preez <pdupreez@gmail.com>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <periph_conf.h>
#include <periph/common_bus.h>
#include <shell.h>

#define INVALID_ARGS    puts("Error: Invalid number of arguments");

#define BUFSIZE         (128U)

#define CONVERT_ERROR   (-32768)

#define ARG_ERROR       (-1)

/* common_bus_buf is global to reduce stack memory consumtion */
//static uint8_t common_bus_buf[BUFSIZE];

#define SETUPS_NUMOF (I2C_NUMOF + SPI_NUMOF)

/**
 * @brief   GPIO parameters
 */
typedef struct  {
    uint8_t port;           /** The port number */
    uint8_t pin;            /** The pin number */
} setup_gpio_param_t;

/**
 * @brief   SPI parameters
 */
typedef struct  {
    unsigned int dev;       /** The device number */
    setup_gpio_param_t cs;  /** Chip select GPIO parameters */
    spi_mode_t mode;        /** The mode */
    spi_clk_t clk;          /** Clock speed */
} setup_spi_param_t;

/**
 * @brief   I2C parameters
 */
typedef struct {
    unsigned int dev;       /** The device number */
    uint8_t addr;           /** The address */
} setup_i2c_param_t;

/**
 * @brief   A union of bus parameters
 */
typedef union
{
    setup_spi_param_t spi;  /** SPI parameters */
    setup_i2c_param_t i2c;  /** I2C parameters */
    unsigned int dev_num;   /** The device number */
} setup_params_t;

/**
 * @brief   A bus setup item type
 */
typedef union
{
    setup_params_t params;  /** The device parameters */
    common_bus_type_t type; /** The device type */
} setup_item_t;

static setup_item_t setups[SETUPS_NUMOF];

static size_t setup_index = 0;

static inline void _print_spi_mode(spi_clk_t clk)
{
    switch (clk) {
    case SPI_CLK_100KHZ:
        printf("clk: SPI_CLK_100KHZ (%d)", clk);
        break;
    case SPI_CLK_400KHZ:
        printf("clk: SPI_CLK_400KHZ (%d)", clk);
        break;
    case SPI_CLK_1MHZ:
        printf("clk: SPI_CLK_1MHZ (%d)", clk);
        break;
    case SPI_CLK_5MHZ:
        printf("clk: SPI_CLK_5MHZ (%d)", clk);
        break;
    case SPI_CLK_10MHZ:
        printf("clk: SPI_CLK_10MHZ (%d)", clk);
        break;
    default:
        break;
    }
}

static inline void _print_bus_params(int i)
{
    switch (setups[i].type) {
    case COMMON_BUS_I2C:
        printf("addr: 0x%02x", setups[i].params.i2c.addr);
        break;
    case COMMON_BUS_SPI:
        printf("cs_port: %d cs_pin: %d mode: %d",
               setups[i].params.spi.cs.port, setups[i].params.spi.cs.pin,
               setups[i].params.spi.mode);
        _print_spi_mode(setups[i].params.spi.clk);
        break;
    default:
        break;
    }
}

static inline void _print_bus_type(int i)
{
    switch (setups[i].type) {
    case COMMON_BUS_I2C:
        printf("I2C");
        break;
    case COMMON_BUS_SPI:
        printf("SPI");
        break;
    default:
        break;
    }
}

static inline void _print_bus_type_index(int i)
{
    switch (setups[i].type) {
    case COMMON_BUS_I2C:
        printf("I2C[%d]", setups[i].params.i2c.dev);
        break;
    case COMMON_BUS_SPI:
        printf("SPI[%d]", setups[i].params.spi.dev);
        break;
    default:
        break;
    }
}

static inline void _print_bus(int i)
{
    _print_bus_type_index(i);
    printf(": ");
    _print_bus_params(i);
}

static inline void _print_all_buses(void)
{
    for (size_t i=0; i<SETUPS_NUMOF; i++) {
        _print_bus(i);
        printf("\n");
    }
}

static inline void _print_common_bus_read(int i, uint16_t *reg, uint8_t *buf, int len)
{
    _print_bus_type_index(i);
    printf(" read %i byte(s) ", len);
    if (reg != NULL) {
        printf("from reg 0x%02x ", *reg);
    }
    printf(": [");
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            printf(", ");
        }
        printf("0x%02x", buf[i]);
    }
    printf("]\n");
}

static inline void _print_common_bus_status(void)
{
    printf("Status: ");
    for (size_t i=0; i<SETUPS_NUMOF; i++) {
        _print_bus_type(i);
        if (i == setup_index) {
            printf("[%i]", setups[i].params.dev_num);
        }
        else {
            printf("(%i)", setups[i].params.dev_num);
        }
    }
    printf("\n");
}

static inline int _get_num(const char *str)
{
    errno = 0;
    char *temp;
    long val = strtol(str, &temp, 0);

    if (temp == str || *temp != '\0' ||
        ((val == LONG_MIN || val == LONG_MAX) && errno == ERANGE)) {
        val = CONVERT_ERROR;
    }
    return (int)val;
}

static int _check_param(int argc, char **argv, int c_min, int c_max, char *use)
{
    int dev;

    if (argc - 1 < c_min || argc - 1 > c_max) {
        printf("Usage: %s %s\n", argv[0], use);
        INVALID_ARGS;
        return ARG_ERROR;
    }

    dev = _get_num(argv[1]);
    if (dev < 0 || dev >= (int)I2C_NUMOF) {
        printf("Error: No device, only %d supported\n", (int)I2C_NUMOF);
        return ARG_ERROR;
    }
    return dev;
}

//static int _print_common_bus_error(int res)
//{
//    if (res == -EOPNOTSUPP) {
//        printf("Error: EOPNOTSUPP [%d]\n", -res);
//        return 1;
//    }
//    else if (res == -EINVAL) {
//        printf("Error: EINVAL [%d]\n", -res);
//        return 1;
//    }
//    else if (res == -EAGAIN) {
//        printf("Error: EAGAIN [%d]\n", -res);
//        return 1;
//    }
//    else if (res == -ENXIO) {
//        printf("Error: ENXIO [%d]\n", -res);
//        return 1;
//    }
//    else if (res == -EIO) {
//        printf("Error: EIO [%d]\n", -res);
//        return 1;
//    }
//    else if (res == -ETIMEDOUT) {
//        printf("Error: ETIMEDOUT [%d]\n", -res);
//        return 1;
//    }
//    else if (res == COMMON_BUS_OK) {
//        printf("Success: COMMON_BUS_OK [%d]\n", res);
//        return 0;
//    }
//    printf("Error: Unknown error [%d]\n", res);
//    return 1;
//}
//
//int cmd_acquire(int argc, char **argv)
//{
//    int res;
//    int dev;
//
//    dev = _check_param(argc, argv, 1, 1, "DEV");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    printf("Command: acquire(%i)\n", dev);
//    res = i2c_acquire(dev);
//    if (res == COMMON_BUS_OK) {
//        printf("Success: i2c_%i acquired\n", dev);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_release(int argc, char **argv)
//{
//    int dev;
//
//    dev = _check_param(argc, argv, 1, 1, "DEV");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    printf("Command: i2c_release(%i)\n", dev);
//    i2c_release(dev);
//
//    printf("Success: i2c_%i released\n", dev);
//    return 0;
//}
//
//int cmd_read_reg(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint16_t reg;
//    uint8_t flags = 0;
//    uint8_t data;
//    int dev;
//
//    dev = _check_param(argc, argv, 4, 4, "DEV ADDR REG FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    reg = _get_num(argv[3]);
//    flags = _get_num(argv[4]);
//
//    printf("Command: i2c_read_reg(%i, 0x%02x, 0x%02x, 0x%02x)\n", dev, addr,
//           reg, flags);
//    res = i2c_read_reg(dev, addr, reg, &data, flags);
//
//    if (res == COMMON_BUS_OK) {
//        _print_common_bus_read(dev, &reg, &data, 1);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_read_regs(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint16_t reg;
//    uint8_t flags = 0;
//    int len;
//    int dev;
//
//    dev = _check_param(argc, argv, 5, 5, "DEV ADDR REG LEN FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    reg = _get_num(argv[3]);
//    len = _get_num(argv[4]);
//    flags = _get_num(argv[5]);
//
//    if (len < 1 || len > (int)BUFSIZE) {
//        puts("Error: invalid LENGTH parameter given");
//        return 1;
//    }
//    else {
//        printf("Command: i2c_read_regs(%i, 0x%02x, 0x%02x, %i, 0x%02x)\n", dev,
//            addr, reg, len, flags);
//        res = i2c_read_regs(dev, addr, reg, common_bus_buf, len, flags);
//    }
//
//    if (res == COMMON_BUS_OK) {
//        _print_common_bus_read(dev, &reg, common_bus_buf, len);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_read_byte(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint8_t flags = 0;
//    uint8_t data;
//    int dev;
//
//    dev = _check_param(argc, argv, 3, 3, "DEV ADDR FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    flags = _get_num(argv[3]);
//
//    printf("Command: i2c_read_byte(%i, 0x%02x, 0x%02x)\n", dev, addr, flags);
//    res = i2c_read_byte(dev, addr, &data, flags);
//
//    if (res == COMMON_BUS_OK) {
//        _print_common_bus_read(dev, NULL, &data, 1);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_read_bytes(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint8_t flags = 0;
//    int len;
//    int dev;
//
//    dev = _check_param(argc, argv, 4, 4, "DEV ADDR LENGTH FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    len = _get_num(argv[3]);
//    flags = _get_num(argv[4]);
//
//    if (len < 1 || len > (int)BUFSIZE) {
//        puts("Error: invalid LENGTH parameter given");
//        return 1;
//    }
//    else {
//        printf("Command: i2c_read_bytes(%i, 0x%02x, %i, 0x%02x)\n", dev,
//         addr, len, flags);
//        res = i2c_read_bytes(dev, addr, common_bus_buf, len, flags);
//    }
//
//    if (res == COMMON_BUS_OK) {
//        _print_common_bus_read(dev, NULL, common_bus_buf, len);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_write_byte(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint8_t flags = 0;
//    uint8_t data;
//    int dev;
//
//    dev = _check_param(argc, argv, 4, 4, "DEV ADDR BYTE FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    data = _get_num(argv[3]);
//    flags = _get_num(argv[4]);
//
//    printf("Command: i2c_write_byte(%i, 0x%02x, 0x%02x, [0x%02x", dev, addr,
//        flags, data);
//    puts("])");
//    res = i2c_write_byte(dev, addr, data, flags);
//
//    if (res == COMMON_BUS_OK) {
//        printf("Success: i2c_%i wrote 1 byte to the bus\n", dev);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_write_bytes(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint8_t flags = 0;
//    int len = argc - 4;
//    int dev;
//
//    dev = _check_param(argc, argv, 4, 3 + BUFSIZE,
//                       "DEV ADDR FLAG BYTE0 [BYTE1 [BYTE_n [...]]]");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    flags = _get_num(argv[3]);
//    for (int i = 0; i < len; i++) {
//        common_bus_buf[i] = _get_num(argv[i + 4]);
//    }
//
//    printf("Command: i2c_write_bytes(%i, 0x%02x, 0x%02x, [", dev, addr,
//        flags);
//    for (int i = 0; i < len; i++) {
//        if (i != 0) {
//            printf(", ");
//        }
//        printf("0x%02x", common_bus_buf[i]);
//    }
//    puts("])");
//    res = i2c_write_bytes(dev, addr, common_bus_buf, len, flags);
//
//    if (res == COMMON_BUS_OK) {
//        printf("Success: i2c_%i wrote %i bytes\n", dev, len);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_write_reg(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint16_t reg;
//    uint8_t flags = 0;
//    uint8_t data;
//    int dev;
//
//    dev = _check_param(argc, argv, 5, 5, "DEV ADDR REG BYTE FLAG");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    reg = _get_num(argv[3]);
//    data = _get_num(argv[4]);
//    flags = _get_num(argv[5]);
//
//    printf("Command: i2c_write_reg(%i, 0x%02x, 0x%02x, 0x%02x, [0x%02x", dev,
//        addr, reg, flags, data);
//    puts("])");
//    res = i2c_write_reg(dev, addr, reg, data, flags);
//
//    if (res == COMMON_BUS_OK) {
//        printf("Success: i2c_%i wrote 1 byte\n", dev);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}
//
//int cmd_write_regs(int argc, char **argv)
//{
//    int res;
//    uint16_t addr;
//    uint16_t reg;
//    uint8_t flags = 0;
//    int len = argc - 5;
//    int dev;
//
//    dev = _check_param(argc, argv, 5, 4 + BUFSIZE,
//                       "DEV ADDR REG FLAG BYTE0 [BYTE1 ...]");
//    if (dev == ARG_ERROR) {
//        return 1;
//    }
//
//    addr = _get_num(argv[2]);
//    reg = _get_num(argv[3]);
//    flags = _get_num(argv[4]);
//    for (int i = 0; i < len; i++) {
//        common_bus_buf[i] = _get_num(argv[i + 5]);
//    }
//
//    printf("Command: i2c_write_regs(%i, 0x%02x, 0x%02x, 0x%02x, [", dev,
//        addr, reg, flags);
//    for (int i = 0; i < len; i++) {
//        if (i != 0) {
//            printf(", ");
//        }
//        printf("0x%02x", common_bus_buf[i]);
//    }
//    puts("])");
//    res = i2c_write_regs(dev, addr, reg, common_bus_buf, len, flags);
//
//    if (res == COMMON_BUS_OK) {
//        printf("Success: i2c_%i wrote %i bytes to reg 0x%02x\n",
//            dev, len, reg);
//        return 0;
//    }
//    return _print_common_bus_error(res);
//}

int cmd_get_devs(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    printf("Success: Amount of i2c devices: [%d]\n", I2C_NUMOF);
    printf("Success: Amount of spi devices: [%d]\n", SPI_NUMOF);
    return 0;
}

int cmd_get_firmware_id(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    puts("Success: [periph_common_bus]");
    return 0;
}

int cmd_get_params(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    printf("Success: get_params\n");
    return 0;
}

int cmd_set_params(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    //common_bus_setup(common_setup_t* setup);
    printf("Success: set_params\n");
    return 0;
}

int cmd_print_common_bus_params(int argc, char **argv)
{
    int bus;
    bus = _check_param(argc, argv, 1, 1, "BUS");
    if (bus == ARG_ERROR) {
        return 1;
    }

    _print_bus(bus);
    _print_common_bus_status();
    return 0;
}

int cmd_print_all_common_bus_params(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    _print_all_buses();
    _print_common_bus_status();
    return 0;
}

static const shell_command_t shell_commands[] = {
//    { "acquire", "Get access to the I2C bus", cmd_acquire },
//    { "release", "Release to the I2C bus", cmd_release },
//    { "read_reg", "Read byte from register", cmd_read_reg },
//    { "read_regs", "Read bytes from registers", cmd_read_regs },
//    { "read_byte", "Read byte from the I2C device", cmd_read_byte },
//    { "read_bytes", "Read bytes from the I2C device", cmd_read_bytes },
//    { "write_byte", "Write byte to the I2C device", cmd_write_byte },
//    { "write_bytes", "Write bytes to the I2C device", cmd_write_bytes },
//    { "write_reg", "Write byte to register", cmd_write_reg },
//    { "write_regs", "Write bytes to registers", cmd_write_regs },
    { "get_devs", "Gets amount of supported i2c devices", cmd_get_devs },
    { "get_id", "Get the firmware id", cmd_get_firmware_id },
    { "get_params", "Get the commn bus parameters", cmd_get_params },
    { "set_params", "Set the common bus parameters", cmd_set_params },
    { "p_param", "Print common bus parameters", cmd_print_common_bus_params },
    { "p_params", "Print all common bus parameters", cmd_print_all_common_bus_params },
};

int main(void)
{
    puts("Start: Test for the low-level I2C driver");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
