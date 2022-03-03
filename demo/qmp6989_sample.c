/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-18     kyle         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "qmp6989.h"
#include "sensor_meas_qmp6989.h"

#define QMP6989_SPI_DEVICE_NAME "qmp6989"

int qmp6989_attach(void)
{
    rt_err_t ret;

#ifdef PKG_QMP6989_USING_INTF_SPI
    static struct rt_spi_device dev_qmp6989;

    rt_pin_mode(PKG_QMP6989_INTF_SPI_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PKG_QMP6989_INTF_SPI_CS_PIN, PIN_HIGH);

    /* control */
    ret = rt_spi_bus_attach_device(&dev_qmp6989, QMP6989_SPI_DEVICE_NAME, PKG_QMP6989_INTF_SPI_BUS_NAME, (void *)PKG_QMP6989_INTF_SPI_CS_PIN);
    if (ret != RT_EOK)
    {
        rt_kprintf("cant`t attach '%s' device to '%s' bus\n", QMP6989_SPI_DEVICE_NAME, PKG_QMP6989_INTF_SPI_BUS_NAME);
        return ret;
    }
#endif

    struct rt_sensor_config cfg;
#if defined(PKG_QMP6989_USING_INTF_I2C)
    cfg.intf.dev_name  = PKG_QMP6989_INTF_I2C_BUS_NAME;
    cfg.intf.type      = RT_SENSOR_INTF_I2C;
    cfg.intf.user_data = (void *)PKG_QMP6989_INTF_I2C_ADDR;
#elif defined(PKG_QMP6989_USING_INTF_SPI)
    cfg.intf.dev_name  = QMP6989_SPI_DEVICE_NAME;
    cfg.intf.type      = RT_SENSOR_INTF_SPI;
    cfg.intf.user_data = RT_NULL;
#endif
    cfg.mode  = RT_SENSOR_MODE_POLLING;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    rt_hw_qmp6989_init("qmp6989", &cfg);

    return RT_EOK;
}
INIT_APP_EXPORT(qmp6989_attach);
