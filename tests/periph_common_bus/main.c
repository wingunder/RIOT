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

#define SETUPS_I2C_DEV_OFFSET 0
#define SETUPS_SPI_DEV_OFFSET (I2C_NUMOF)
#define SETUPS_DEV_NUMOF      (I2C_NUMOF + SPI_NUMOF)

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
    switch (common_bus_get_type(i)) {
    case COMMON_BUS_I2C:
        printf("addr: 0x%02x", common_bus_get_i2c_addr(i));
        break;
    case COMMON_BUS_SPI:
        printf("cs_port: %d cs_pin: %d mode: %d ",
               0, //common_bus_get_spi_cs_port(i),
               0, //common_bus_get_spi_cs_pin(i),
               common_bus_get_spi_mode(i));
        _print_spi_mode(common_bus_get_spi_clk(i));
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
    if (dev >= SETUPS_DEV_NUMOF) {
        printf("Error: No device, only %d supported\n", SETUPS_DEV_NUMOF);
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

int cmd_print_params(int argc, char **argv)
{
    (void)argv;
    (void)argc;
    for (size_t i=0; i<SETUPS_DEV_NUMOF; i++) {
        printf("BUS %d: ", i);
        _print_bus_type_index(i);
        printf(": ");
        _print_bus_params(i);
        printf("\n");
    }
    return 0;
}

int cmd_common_bus_init(int argc, char **argv)
{
    int ret = 0;
    size_t bus;
    bool show_usage = false;
    if (argc > 1) {
        bus = _get_num(argv[1]);
        if (bus >= SETUPS_DEV_NUMOF) {
            printf("BUS out of range!\n");
            cmd_print_params(argc, argv);
            show_usage = true;
        }
    }
    if (argc == 6) {
        int8_t port = _get_num(argv[2]);
        uint8_t pin = _get_num(argv[3]);
        spi_mode_t mode = _get_num(argv[4]);
        if (mode > SPI_MODE_3) {
            printf("MODE out of range!\n");
            show_usage = true;
        }
        spi_clk_t clk = _get_num(argv[5]);
        if (clk > SPI_CLK_10MHZ) {
            printf("CLK out of range!\n");
            show_usage = true;
        }
        if (!show_usage) {
            ret = common_bus_spi_init(bus, port, pin, mode, clk);
        }
    }
    else if (argc == 3) {
        uint8_t addr = _get_num(argv[2]);
        ret = common_bus_i2c_init(bus, addr);
    }
    else {
        show_usage = true;
    }

    if (show_usage) {
        printf("Usage: %s %s\n", argv[0], "BUS ADDR");
        printf("BUS: 0 to %d (see 'buses' command)\n", SETUPS_DEV_NUMOF-1);
        printf("I2C ADDR: 0 to 255\n");
        printf("Usage: %s %s\n", argv[0], "BUS CS_PORT CS_PIN MODE CLK");
        printf("BUS: 0 to %d (see 'buses' command)\n", SETUPS_DEV_NUMOF-1);
        printf("SPI CS_PORT: chip select port number\n");
        printf("SPI CS_PIN: chip select pin number\n");
        printf("SPI MODE: 0 to 3\n");
        printf("SPI CLK: 0 to 4 (0: 100kHz, 1: 400kHz, 2: 1MHz, 3: 5MHz, 4: 10MHz)\n");
        printf("ERROR: expected either 3 or 6 params, but %d params were supplied.\n", argc);
        INVALID_ARGS;
        return ARG_ERROR;
    }
    return ret;
}

int cmd_common_bus_whoami(int argc, char **argv)
{
    size_t bus;
    bus = _check_param(argc, argv, 1, 1, "BUS");
    //bus = _get_num(argv[1]);
    if (bus >= SETUPS_DEV_NUMOF) {
        printf("BUS out of range!\n");
        cmd_print_params(argc, argv);
        return ARG_ERROR;
    }

//    uint8_t reg_val;
//    common_bus_cmd_t cmd;
//    cmd.type = COMMON_BUS_READ_REGS;
//    cmd.args.read_regs.reg = 0x0F;
//    cmd.args.read_regs.data = &reg_val;
//    cmd.args.read_regs.len = 1;
//    if (common_bus_setups[bus].f.add(&common_bus_setups[bus].bus, &cmd) == COMMON_BUS_OK) {
//        if (common_bus_setups[bus].f.run(&common_bus_setups[bus].bus) == COMMON_BUS_OK) {
//            printf("WHO_AM_I returned 0x%02x\n", reg_val);
//        }
//        common_bus_setups[bus].f.clear(&common_bus_setups[bus].bus);
//    }
    return 0;
}

int cmd_init_lps(int argc, char **argv)
{
    if (argv || argc) {}

    uint8_t buf = 0;
    size_t bus = 1;
    spi_t dev = common_bus_get_spi_dev(bus);
//    spi_t dev = SPI_DEV(1);
//    printf("dev = 0x%02x devX = 0x%02x\n", dev, devX);
////    spi_mode_t mode = 0;
////    spi_clk_t clk = 0;
    //spi_cs_t cs = GPIO_PIN(1,12);
    int8_t port = common_bus_get_spi_cs_port(bus);
    uint8_t pin = common_bus_get_spi_cs_pin(bus);
    spi_cs_t cs = GPIO_PIN(port, pin);

//    if  (spi_init_cs(dev, cs) != SPI_OK) {
//        puts("error: unable to initialize the given chip select line");
//        return 1;
//    }
//
//    int tmp = spi_acquire(dev, cs, mode, clk);
//    if (tmp == SPI_NOMODE) {
//        puts("error: given SPI mode is not supported");
//        return 1;
//    }
//    else if (tmp == SPI_NOCLK) {
//        puts("error: targeted clock speed is not supported");
//        return 1;
//    }
//    else if (tmp != SPI_OK) {
//        puts("error: unable to acquire bus with given parameters");
//        return 1;
//    }
//    spi_release(dev);

    /* get bus access */
    printf("dev = 0x%02x\n", dev);
    if (spi_acquire(dev, cs, 0, 0) != SPI_OK) {
        puts("error: unable to acquire the SPI bus");
        return 1;
    }

    /* Read reg at address 0x8f */
    spi_transfer_regs(dev, cs, 0x8f, NULL, &buf, 1);
    printf("Result = 0x%02x\n", buf);

    /* release the bus */
    spi_release(dev);

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "init", "common bus init", cmd_common_bus_init },
//    { "add", "common bus add", cmd_common_bus_add },
//    { "run", "common bus run", cmd_common_bus_run },
//    { "clear", "common bus clear", cmd_common_bus_clear },
    { "whoami", "Read the whoami register", cmd_common_bus_whoami },
    { "buses", "Print all common bus parameters", cmd_print_params },
    { "lps", "lps", cmd_init_lps },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Start: Test for the low-level common bus (I2C/SPI) driver");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
