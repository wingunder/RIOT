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

static inline void _print_spi_clk(spi_clk_t clk)
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

static inline void _print_soft_spi_clk(soft_spi_clk_t clk)
{
    switch (clk) {
    case SOFT_SPI_CLK_100KHZ:
        printf("clk: SPI_CLK_100KHZ (%d)", clk);
        break;
    case SOFT_SPI_CLK_400KHZ:
        printf("clk: SPI_CLK_400KHZ (%d)", clk);
        break;
    case SOFT_SPI_CLK_DEFAULT:
        printf("clk: max possible");
        break;
    default:
        break;
    }
}

static inline void _print_bus_params(int i)
{
    switch (common_bus_get_type(i)) {
    case COMMON_BUS_I2C:
        printf("addr: 0x%02x", common_bus_get_i2c_addr(i));
        break;
    case COMMON_BUS_SPI:
        printf("cs_port: %d cs_pin: %d mode: %d ",
               common_bus_get_spi_cs_port(i),
               common_bus_get_spi_cs_pin(i),
               common_bus_get_spi_mode(i));
        _print_spi_clk(common_bus_get_spi_clk(i));
        break;
    case COMMON_BUS_SOFT_SPI:
        printf("cs_port: %d cs_pin: %d mode: %d ",
               common_bus_get_soft_spi_cs_port(i),
               common_bus_get_soft_spi_cs_pin(i),
               common_bus_get_soft_spi_mode(i));
        _print_spi_clk(common_bus_get_soft_spi_clk(i));
        break;
    default:
        break;
    }
}

static inline void _print_bus_type(int i)
{
    switch (common_bus_get_type(i)) {
    case COMMON_BUS_I2C:
        printf("I2C");
        break;
    case COMMON_BUS_SPI:
        printf("SPI");
        break;
    case COMMON_BUS_SOFT_SPI:
        printf("SOFT_SPI");
        break;
    default:
        break;
    }
}

static inline void _print_bus_type_index(int i)
{
    switch (common_bus_get_type(i)) {
    case COMMON_BUS_I2C:
        printf("I2C[%d]", common_bus_get_i2c_dev(i));
        break;
    case COMMON_BUS_SPI:
        printf("SPI[%d]", common_bus_get_spi_dev(i));
        break;
    case COMMON_BUS_SOFT_SPI:
        printf("SOFT_SPI[%d]", common_bus_get_soft_spi_dev(i));
        break;
    default:
        break;
    }
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
    size_t dev;

    if (argc - 1 < c_min || argc - 1 > c_max) {
        printf("Usage: %s %s\n", argv[0], use);
        INVALID_ARGS;
        return ARG_ERROR;
    }

    dev = _get_num(argv[1]);
    if (dev >= COMMON_BUS_DEV_NUMOF) {
        printf("Error: No device, only %d supported\n", COMMON_BUS_DEV_NUMOF);
        return ARG_ERROR;
    }
    return dev;
}

int cmd_print_params(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    for (size_t i=0; i<COMMON_BUS_DEV_NUMOF; i++) {
        printf("BUS %d: ", (int)i);
        _print_bus_type_index(i);
        printf(": ");
        _print_bus_params(i);
        printf("\n");
    }
    return 0;
}

static int _show_init_usage(char * cmd)
{
    printf("Usage (I2C): %s %s\n", cmd, "BUS ADDR");
    printf("BUS: %d to %d (see 'b' command)\n", COMMON_BUS_I2C_DEV_OFFSET,
           COMMON_BUS_I2C_DEV_OFFSET + I2C_NUMOF - 1);
    printf("I2C ADDR: 0 to 255\n");
    printf("Usage (SPI): %s %s\n", cmd, "BUS CS_PORT CS_PIN MODE CLK");
    printf("BUS: %d to %d (see 'b' command)\n", COMMON_BUS_SPI_DEV_OFFSET,
           COMMON_BUS_SPI_DEV_OFFSET + SPI_NUMOF - 1);
    printf("SPI CS_PORT: chip select port number (-1 for hardware CS)\n");
    printf("SPI CS_PIN: chip select pin number\n");
    printf("SPI MODE: 0 to 3\n");
    printf("SPI CLK: 0 to 3 (0: 100kHz, 1: 400kHz, 2: 1MHz, 3: 5MHz, 4: 10MHz)\n");
    printf("Usage (SOFT_SPI): %s %s\n", cmd, "BUS CS_PORT CS_PIN MODE CLK");
    printf("BUS: %d to %d (see 'b' command)\n", COMMON_BUS_SOFT_SPI_DEV_OFFSET,
           COMMON_BUS_SOFT_SPI_DEV_OFFSET + SOFT_SPI_NUMOF - 1);
    printf("SOFT_SPI CS_PORT: chip select port number\n");
    printf("SOFT_SPI CS_PIN: chip select pin number\n");
    printf("SOFT_SPI MODE: 0 to 3\n");
    printf("SOFT_SPI CLK: 0 to 2 (0: 100kHz, 1: 400kHz, 2: max_possible)\n");
    return ARG_ERROR;
}

int cmd_common_bus_init(int argc, char **argv)
{
    int ret = 0;
    size_t bus;
    common_bus_type_t bus_type;
    if (argc > 1) {
        bus = _get_num(argv[1]);
        if (bus >= COMMON_BUS_DEV_NUMOF) {
            printf("BUS out of range!\n");
            cmd_print_params(argc, argv);
            return _show_init_usage(argv[0]);
        }
        else {
            bus_type = common_bus_get_type(bus);
        }
    }
    if (argc == 6) {
        int8_t port = _get_num(argv[2]);
        uint8_t pin = _get_num(argv[3]);
        spi_mode_t mode = _get_num(argv[4]);
        if (mode > SPI_MODE_3) {
            printf("MODE out of range!\n");
            return _show_init_usage(argv[0]);
        }
        spi_clk_t clk = _get_num(argv[5]);
        if (clk > SPI_CLK_10MHZ) {
            printf("CLK out of range!\n");
            return _show_init_usage(argv[0]);
        }

        if (bus_type == COMMON_BUS_SPI) {
            spi_t dev = common_bus_get_spi_dev(bus);
            ret = common_bus_spi_init(dev, port, pin, mode, clk);
        }
        else if (bus_type == COMMON_BUS_SOFT_SPI) {
            soft_spi_t dev = common_bus_get_soft_spi_dev(bus);
            ret = common_bus_soft_spi_init(dev, port, pin, mode, clk);
        }
        else {
            printf("ERROR: BUS %d is neither an SPI or a SOFT SPI bus.\n", bus);
            return _show_init_usage(argv[0]);
        }
        printf("BUS %d is now initialized with port=%d pin=%d mode=%d clk=%d\n", ret, port, pin, mode, clk);
    }
    else if (argc == 3) {
        uint8_t addr = _get_num(argv[2]);
        if (bus_type != COMMON_BUS_I2C) {
            printf("ERROR: BUS %d is not an I2C bus.\n", bus);
            return _show_init_usage(argv[0]);
        }
        i2c_t dev = common_bus_get_i2c_dev(bus);
        ret = common_bus_i2c_init(dev, addr);
        printf("BUS %d is now initialized with addr 0x%02x\n", ret, addr);
    }
    else {
        printf("ERROR: expected either 3 or 6 params, but %d params were supplied.\n", argc);
        return _show_init_usage(argv[0]);
    }
    cmd_print_params(argc, argv);
    return ret;
}

int cmd_common_bus_whoami(int argc, char **argv)
{
    size_t bus;
    bus = _check_param(argc, argv, 1, 1, "BUS");
    if (bus >= COMMON_BUS_DEV_NUMOF) {
        printf("BUS out of range!\n");
        cmd_print_params(argc, argv);
        return ARG_ERROR;
    }
    uint8_t buf = 0;
    int ret = common_bus_read_regs(bus, 0x0f, &buf, 1, 0);
    printf("Result = 0x%02x\n", buf);
    return ret;
}

static const shell_command_t shell_commands[] = {
    { "i", "common bus init", cmd_common_bus_init },
    { "w", "Read the whoami register", cmd_common_bus_whoami },
    { "b", "Print all common bus parameters", cmd_print_params },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Start: Test for the low-level common bus (I2C/SPI) driver");
    common_bus_init();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
