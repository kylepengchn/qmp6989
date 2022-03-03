#include <sensor_meas_qmp6989.h>
#include <rtdevice.h>
#include <rthw.h>

#define DBG_TAG              "qmp6989"
#define DBG_LVL              DBG_ERROR
#include <rtdbg.h>

#define QMP6989_DEV_NAME    "qmp6989"

static qmp6989_device_t _qmp6989_init(struct rt_sensor_intf *intf)
{
    if (intf->type == RT_SENSOR_INTF_I2C) {
        rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;
        return qmp6989_init(intf->dev_name, i2c_addr);
    }
    else if (intf->type == RT_SENSOR_INTF_SPI) {
        return qmp6989_init(intf->dev_name, RT_NULL);
    }

    return RT_NULL;
}

static rt_size_t _qmp6989_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    qmp6989_device_t qmp6989 = (qmp6989_device_t)sensor->parent.user_data;
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    int32_t baro;
    int16_t temp;

    if (sensor->info.type == RT_SENSOR_CLASS_BARO) {
        qmp6989_get_data(qmp6989, &baro, &temp);

        data->type = RT_SENSOR_CLASS_BARO;
        data->data.baro = baro;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        qmp6989_get_data(qmp6989, &baro, &temp);

        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = temp * 10 / 256;
        data->timestamp = rt_sensor_get_ts();
    }
    else
        return 0;

    return 1;
}

static rt_err_t _qmp6989_control(struct rt_sensor_device *sensor, int cmd, void *arg)
{
    rt_err_t result = RT_EOK;

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    .fetch_data = _qmp6989_fetch_data,
    .control    = _qmp6989_control,
};

static int rt_hw_qmp6989_baro_init(const char *name, struct rt_sensor_config *cfg, qmp6989_device_t hdev)
{
    rt_err_t ret;
    rt_sensor_t sensor = RT_NULL;
    
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (!sensor)
        return -RT_ERROR;
    
    sensor->info.type       = RT_SENSOR_CLASS_BARO;
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model      = "qmp6989_baro";
    sensor->info.unit       = RT_SENSOR_UNIT_PA;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C | RT_SENSOR_INTF_SPI;
    sensor->info.range_max  = 110000;
    sensor->info.range_min  = 30000;
    sensor->info.period_min = 42;
    
    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    ret = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, hdev);
    if (ret != RT_EOK) {
        LOG_E("device register err code: %d", ret);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("baro sensor init success");

    return RT_EOK;
}

static int rt_hw_qmp6989_temp_init(const char *name, struct rt_sensor_config *cfg, qmp6989_device_t hdev)
{
    rt_err_t ret;
    rt_sensor_t sensor = RT_NULL;
    
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (!sensor)
        return -RT_ERROR;
    
    sensor->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
    sensor->info.model      = "qmp6989_temp";
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C | RT_SENSOR_INTF_SPI;
    sensor->info.range_max  = 85;
    sensor->info.range_min  = -40;
    sensor->info.period_min = 42;
    
    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    ret = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, hdev);
    if (ret != RT_EOK) {
        LOG_E("device register err code: %d", ret);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("temp sensor init success");

    return RT_EOK;
}

int rt_hw_qmp6989_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_err_t res = RT_EOK;
    qmp6989_device_t dev;
    
    dev = _qmp6989_init(&cfg->intf);
    if (!dev) {
        LOG_E("_qmp6989_init init err!");
        goto __exit;
    }

    res += rt_hw_qmp6989_baro_init(name, cfg, dev);
    res += rt_hw_qmp6989_temp_init(name, cfg, dev);
    if (res != RT_EOK)
        goto __exit;

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (dev)
        qmp6989_deinit(dev);

    return -RT_ERROR;
}
