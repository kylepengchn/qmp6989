#include <qmp6989.h>
#include <rtdevice.h>
#include <rthw.h>

#define DBG_TAG              "qmp6989"
#define DBG_LVL              DBG_ERROR
#include <rtdbg.h>

union spi_ins_head {
    rt_uint16_t data;
    struct {
        rt_uint16_t rw      : 1;
        rt_uint16_t word    : 2;
        rt_uint16_t addr    : 13;
    } b;
};


#define QMP6989_CALIBRATION_DATA_CNT	(16)

static const float QMP6989_CALIB_SCALE_FACTOR[] = {
    1.0E+00,
    1.0E-05,
    1.0E-10,
    1.0E-05,
    1.0E-10,
    1.0E-15,
    1.0E-12,
    1.0E-17,
    1.0E-21 
};

static const int32_t QMP6989_POWER_SCALE[] = {1, 10, 100, 1000};
// QMP6989
#ifdef PKG_QMP6989_SUPPORT_FLOAT
static float f_calibparam[QMP6989_CALIBRATION_PARAMETER_COUNT], ft_celsius, fp_pa;//, fAlt_m;
#else
static int32_t g_celsius, g_pa;
static int16_t g_value_record[QMP6989_CALIBRATION_PARAMETER_COUNT];
static uint8_t g_power_record[QMP6989_CALIBRATION_PARAMETER_COUNT];
#endif
// QMP6989


static rt_err_t i2c_qmp6989_send_cmd(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
#ifdef RT_USING_I2C
    int tmp;
    struct rt_i2c_msg msg;
    rt_uint8_t *pbuf;
    rt_uint8_t buf[2];

    if (len <= 1) {
        buf[0] = reg;
        buf[1] = *pd;
        pbuf = buf;
    }
    else {
        pbuf = rt_malloc(1 + len);
        pbuf[0] = reg;
        rt_memcpy(pbuf + 1, pd, len);
    }

    msg.addr    = dev->i2c_addr;
    msg.flags   = RT_I2C_WR;
    msg.buf     = pbuf;
    msg.len     = 1 + len;
    tmp = rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msg, 1);
    if (len > 1)
        rt_free(pbuf);

    if (tmp != 1)
        return -RT_ERROR;

    return RT_EOK;
#else
    return -RT_ERROR;
#endif
}

static rt_err_t i2c_qmp6989_recv_data(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
#ifdef RT_USING_I2C
    int tmp;
    struct rt_i2c_msg msg[2];

    msg[0].addr    = dev->i2c_addr;
    msg[0].flags   = RT_I2C_WR;
    msg[0].buf     = &reg;
    msg[0].len     = 1;
    
    msg[1].addr    = dev->i2c_addr;
    msg[1].flags   = RT_I2C_RD;
    msg[1].buf     = pd;
    msg[1].len     = len;

    tmp = rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msg, 2);
    if (tmp != 2)
        return -RT_ERROR;

    return RT_EOK;
#else
    return -RT_ERROR;
#endif
}

static rt_err_t spi_qmp6989_send_cmd(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
#ifdef RT_USING_SPI
    union spi_ins_head head;

    RT_ASSERT(pd && (len > 0));
    
    head.data   = 0;
    head.b.rw   = 0;
    head.b.word = (len > 4) ? 3 : (len - 1);
    head.b.addr = reg;
    
    return rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &head, sizeof(union spi_ins_head), pd, len);
#else
    return -RT_ERROR;
#endif
}

static rt_err_t spi_qmp6989_recv_data(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
#ifdef RT_USING_SPI
    union spi_ins_head head;

    RT_ASSERT(pd && (len > 0));
    
    head.data   = 0;
    head.b.rw   = 1;
    head.b.word = (len > 4) ? 3 : (len - 1);
    head.b.addr = reg;
    
    return rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &head, sizeof(union spi_ins_head), pd, len);
#else
    return -RT_ERROR;
#endif
}

static rt_err_t qmp6989_burst_read(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
    if (dev->bus->type == RT_Device_Class_I2CBUS) {
        return i2c_qmp6989_recv_data(dev, reg, pd, len);
    }
    else if (dev->bus->type == RT_Device_Class_SPIBUS) {
        return spi_qmp6989_recv_data(dev, reg, pd, len);
    }

    return -RT_ERROR;
}

static rt_err_t qmp6989_burst_write(qmp6989_device_t dev, uint8_t reg, uint8_t *pd, uint8_t len)
{
    if (dev->bus->type == RT_Device_Class_I2CBUS) {
        return i2c_qmp6989_send_cmd(dev, reg, pd, len);
    }
    else if (dev->bus->type == RT_Device_Class_SPIBUS) {
        return spi_qmp6989_send_cmd(dev, reg, pd, len);
    }

    return -RT_ERROR;
}

static rt_err_t qmp6989_get_pid(qmp6989_device_t dev, uint8_t *pid)
{
    int ret = RT_EOK;

    ret = qmp6989_burst_read(dev, QMP6989_REG_PID, pid, 1);
    if (ret >= 0)
        LOG_I("pid: 0x%x", *pid);

    return ret;
}

static rt_err_t qmp6989_soft_reset(qmp6989_device_t dev)
{
    uint8_t data = QMP6989_RST_SET_VAL;

    return qmp6989_burst_write(dev, QMP6989_REG_RESET, &data, 1);
}

/*!
 * @brief Get qmp6989 calibration parameters
 *        - Read calibration register AAh~BBh total 18 bytes
 *        - Compose 9 calibration parameters from the 18 bytes
 *
 * @param f_calibparam: the calibration parameter array returned to caller
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
static rt_err_t qmp6989_get_calibration_param(qmp6989_device_t dev, float *f_calibparam)
{
    int ret = RT_EOK;
    uint8_t buf[QMP6989_CALIBRATION_REGISTER_COUNT];
    int32_t i, shift, tmp;

    ret = qmp6989_burst_read(dev, QMP6989_REG_CALIB00, buf, QMP6989_CALIBRATION_REGISTER_COUNT);
    if (ret < 0)
        goto __exit;

    // Get the parameters
    shift = sizeof(int32_t) * 8 - 16;
    for (i = 0; i < QMP6989_CALIBRATION_PARAMETER_COUNT; i++) {
        tmp = (buf[2 * i] << 8) + buf[2 * i + 1];
        f_calibparam[i] = ((tmp << shift) >> (shift + 2)) * QMP6989_POWER_SCALE[(buf[2 * i + 1] & 0x03)] * QMP6989_CALIB_SCALE_FACTOR[i];
    }

__exit:
    return ret;
}

/*!
 * @brief Get qmp6989 calibration parameters for fixed-point compensation
 *        - Read calibration register AAh~BBh total 18 bytes
 *        - Return 9 calibration parameters with fixed-point value and power parts
 *
 * @param value[]: array of the value part of the calibration parameter
 * @param power[]: array of the power part of the calibration parameter
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
static rt_err_t qmp6989_get_calibration_param_fixed_point(qmp6989_device_t dev, int16_t value[], uint8_t power[])
{
    int ret = RT_EOK;
    uint8_t buf[QMP6989_CALIBRATION_REGISTER_COUNT];
    int16_t i, tmp;

    //read the calibration registers
    ret = qmp6989_burst_read(dev, QMP6989_REG_CALIB00, buf, QMP6989_CALIBRATION_REGISTER_COUNT);
    if (ret < 0)
        goto __exit;

    for (i = 0; i < QMP6989_CALIBRATION_PARAMETER_COUNT; i++) {
        tmp = (buf[2 * i] << 8) + buf[2 * i + 1];
        value[i] = (tmp >> 2);
        power[i]  = (tmp & 0x03);
    }

__exit:
    return ret;
}

/*!
 * @brief qmp6989 initialization
 *        Set AAh ~ ADh to 0x00
 *
 * @param None
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
qmp6989_device_t qmp6989_init(const char *dev_name, rt_uint8_t param)
{
    int res = RT_EOK;
    qmp6989_device_t dev;
    uint8_t data[] = {0, 0, 0, 0};
    uint8_t pid = 0;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct qmp6989_device));
    if (!dev) {
        LOG_E("can't allocate memory for qmp6989 device on '%s'", dev_name);
        goto __exit;
    }
    
    dev->bus = rt_device_find(dev_name);
    if (!dev->bus) {
        LOG_E("can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS) {
        if (param) {
            dev->i2c_addr = param;
        }
        else {
            dev->i2c_addr = QMP6989_ADDRESS_AD0_LOW;
        }
    }
    else if (dev->bus->type == RT_Device_Class_SPIDevice) {
#ifdef RT_USING_SPI
        struct rt_spi_configuration cfg;

        cfg.data_width  = 8;
        cfg.mode        = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz      = 8 * 1000 * 1000; /* Set spi max speed */

        rt_spi_configure((struct rt_spi_device *)dev->bus, &cfg);
#endif
    }
    else {
        LOG_E("unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    qmp6989_get_pid(dev, &pid);
    if (pid != QMP6989_PID_VAL)
        goto __exit;

    qmp6989_soft_reset(dev);
    rt_thread_mdelay(100);
#ifdef PKG_QMP6989_SUPPORT_FLOAT
    res += qmp6989_get_calibration_param(dev, f_calibparam);
#else
    res += qmp6989_get_calibration_param_fixed_point(dev, g_value_record, g_power_record);
#endif

	/* QMP6989 initialization setup */
    //Set AAh ~ AD to 0x00
    res += qmp6989_burst_write(dev, QMP6989_REG_CALIB00, data, 4);

    if (res != RT_EOK) {
        LOG_E("error in device initialization!");
        goto __exit;
    }

    LOG_I("device init succeed!");

    return dev;

__exit:
    if (dev)
        rt_free(dev);

    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void qmp6989_deinit(qmp6989_device_t dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

/*!
 * @brief qmp6989 measure temperature
 *
 * @param *temperature calibrated temperature code returned to caller
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int qmp6989_measure_temperature(qmp6989_device_t dev)
{
    int ret = RT_EOK;
    int8_t timecnt = 0;
    uint8_t data[2];

    // Set A5h = 0x00, temperature Calibrated data out
    data[0] = 0x00;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG1, data, 1);
    if (ret < 0)
        goto __exit;

    // Set 30h = 0x08, T-Forced mode
    data[0] = 0x08;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CMD, data, 1);
    if (ret < 0)
        goto __exit;

    // Wait for 02h[0] DRDY bit set
    do {
        //wait a while
        rt_thread_mdelay(1);

        ret = qmp6989_burst_read(dev, QMP6989_REG_STATUS, data, 1);
        if (ret < 0)
            goto __exit;
    } while ((QMP6989_GET_BITSLICE(data[0], QMP6989_DRDY) != 1) && (timecnt++ < 100));

    // Read 09h~0Ah
    ret = qmp6989_burst_read(dev, QMP6989_REG_TEMPH, data, 2);
    if (ret < 0)
        goto __exit;

    // Get the calibrated temperature in code
    dev->temp = (data[0] << 8) + data[1];
    LOG_D("temperature: 0x%x, 0x%x", data[0], data[1]);

__exit:
    return ret;
}

/*!
 * @brief qmp6989 measure pressure
 *
 * @param *pressure raw pressure in code returned to caller
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int qmp6989_measure_pressure(qmp6989_device_t dev)
{
    int ret = RT_EOK;
    int8_t tmp, timecnt = 0;
    uint8_t data[3];

    // Set A5h = 0x02, pressure raw data out
    data[0] = 0x02;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG1, data, 1);
    if (ret < 0)
        goto __exit;

    // Set 30h = 0x09, P-Forced mode
    data[0] = 0x09;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CMD, data, 1);
    if (ret < 0)
        goto __exit;

    // Wait for 02h[0] DRDY bit set
    do {
        //wait a while
        rt_thread_mdelay(1);

        ret = qmp6989_burst_read(dev, QMP6989_REG_STATUS, data, 1);
        if (ret < 0)
            goto __exit;
    } while ((QMP6989_GET_BITSLICE(data[0], QMP6989_DRDY) != 1) && (timecnt++ < 100));

    // Read 06h~08h
    ret = qmp6989_burst_read(dev, QMP6989_REG_PRESSH, data, 3);
    if (ret < 0)
        goto __exit;

    tmp = sizeof(dev->baro) * 8 - 24;
    // Get the raw pressure in code
    dev->baro = (data[0] << 16) + (data[1] << 8) + data[2];
    dev->baro = (dev->baro << tmp) >> tmp; //24 bit sign extension

__exit:
    return ret;
}

/*!
 * @brief qmp6989 measure pressure and temperature
 *        Read pressure first then commit pressure data conversion for the next call
 *
 * @param *pressure raw pressure in code returned to caller
 * @param *temperature calibrated temperature code returned to caller
 * @param s8WaitPDrdy 1: P wait for DRDY bit set, 0: P no wait
 *
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int qmp6989_measure_p_and_t(qmp6989_device_t dev, int8_t wait_drdy)
{
    int ret = RT_EOK;
    int8_t tmp, timecnt = 0;
    uint8_t data[3];

    /*
    * Read raw P code
    */
    if (wait_drdy) {
        // Wait for 02h[0] DRDY bit set if wait_drdy is 1
        do {
            //wait a while
            rt_thread_mdelay(1);

            ret = qmp6989_burst_read(dev, QMP6989_REG_STATUS, data, 1);
            if (ret < 0)
                goto __exit;

        } while ((QMP6989_GET_BITSLICE(data[0], QMP6989_DRDY) != 1) && (timecnt++ < 100));
    }

    // Read 06h~08h
    ret = qmp6989_burst_read(dev, QMP6989_REG_PRESSH, data, 3);
    if (ret < 0)
        goto __exit;

    tmp = sizeof(dev->baro) * 8 - 24;
    // Get the raw pressure in code
    dev->baro = (data[0] << 16) + (data[1] << 8) + data[2];
    dev->baro = (dev->baro << tmp) >> tmp; //24 bit sign extension

    /*
    * Measure calibrated T code
    */
    // Set A5h = 0x00, Calibrated data out
    data[0] = 0x00;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG1, data, 1);
    if (ret < 0)
        goto __exit;

    // Set 30h = 0x08, T-Forced mode
    data[0] = 0x08;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CMD, data, 1);
    if (ret < 0)
        goto __exit;

    timecnt	= 0;
    // Wait for 02h[0] DRDY bit set
    do {
        //wait a while
        rt_thread_mdelay(1);

        ret = qmp6989_burst_read(dev, QMP6989_REG_STATUS, data, 1);
        if (ret < 0)
            goto __exit;

    } while ((QMP6989_GET_BITSLICE(data[0], QMP6989_DRDY) != 1) && (timecnt++ < 100));

    // Read 09h~0Ah
    ret = qmp6989_burst_read(dev, QMP6989_REG_TEMPH, data, 2);
    if (ret < 0)
        goto __exit;

    // Get the calibrated temperature in code
    dev->temp = (data[0] << 8) + data[1];

    /*
    * Commit the next pressure conversion
    */
    // Set A5h = 0x02, raw data out
    data[0] = 0x02;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG1, data, 1);
    if (ret < 0)
        goto __exit;

    // Set 30h = 0x09, P-Forced mode
    data[0] = 0x09;
    ret = qmp6989_burst_write(dev, QMP6989_REG_CMD, data, 1);
    if (ret < 0)
        goto __exit;

__exit:
    return ret;
}

float qmp6989_calibration_process(qmp6989_device_t dev, float pfs_pa, float pfp_pa)
{
	static float pfo_pa = 0.0f;
	static uint8_t cnt = 0;

    if (cnt < QMP6989_CALIBRATION_DATA_CNT) {
		pfo_pa += pfp_pa;
		cnt++;
	}
	else {
		pfo_pa =(pfo_pa / cnt - pfs_pa);
	}

	return pfo_pa;
}

/*!
 * @brief qmp6989 temperature and pressure compensation
 *
 * @param temp calibrated temperature in code
 * @param press raw pressure in code
 * @param f_param[] pressure calibration parameters
 * @param *pft_celsius calibrated temperature in Celsius returned to caller
 * @param *pfp_pa calibraated pressure in Pa returned to caller
 *
 * @return None
 *
 */
void qmp6989_compensation(
    qmp6989_device_t dev,
    float f_param[],
    float *pft_celsius,
    float *pfp_pa)
{
    *pft_celsius = QMP6989_T_CODE_TO_CELSIUS(dev->temp);

    *pfp_pa =   f_param[0] +                                        \
                f_param[1] * dev->temp +                            \
                f_param[2] * dev->temp * dev->temp +                \
                f_param[3] * dev->baro +                            \
                f_param[4] * dev->temp * dev->baro +                \
                f_param[5] * dev->temp * dev->temp * dev->baro +    \
                f_param[6] * dev->baro * dev->baro +                \
                f_param[7] * dev->temp * dev->baro * dev->baro +    \
                f_param[8] * dev->temp * dev->temp * dev->baro * dev->baro;
}

#define SHIFT_RIGHT(v, s)   (((v) + (1 << ((s) - 1))) >> (s))
#define ROUND_DIVIDE(v, d)  (((v) + ((d) / 2)) / (d))

/*!
 * @brief qmp6989 temperature and pressure compensation, int64_t fixed point operation
 *
 * @param temp raw temperature in code
 * @param press raw pressure in code
 * @param value[]: array of the value part of the calibration parameter
 * @param power[]: array of the power part of the calibration parameter
 * @param *celsius calibrated temperature in 1/256*Celsius returned to caller
 * @param *p_pa calibrated pressure in Pa returned to caller
 *
 * @return None
 *
 */
void qmp6989_compensation_fixed_point_s64(
    qmp6989_device_t dev,
    int16_t value[], 
    uint8_t power[],
    int32_t* celsius,
    int32_t* p_pa
)
{
  int64_t tmp, val, tmp_t, tmp_p;
  tmp_t = dev->temp;
  tmp_p = dev->baro;

  //Temperature
  *celsius = dev->temp;

  //Pressure
  val = 0;
  //beta0
  tmp = value[0] * QMP6989_POWER_SCALE[power[0]] * 10;
  val += tmp;
  //beta1*T
  tmp = tmp_t * value[1];
  tmp = tmp * QMP6989_POWER_SCALE[power[1]];
  tmp = ROUND_DIVIDE(tmp, 10000);
  val += tmp;
  //beta2*T*T
  tmp = tmp_t * value[2];
  tmp = tmp * tmp_t;
  tmp = tmp * QMP6989_POWER_SCALE[power[2]];
  tmp = ROUND_DIVIDE(tmp, 1000000000);
  val += tmp;
  //beta3*P
  tmp = tmp_p * value[3];
  tmp = tmp * QMP6989_POWER_SCALE[power[3]];
  tmp = ROUND_DIVIDE(tmp, 10000);
  val += tmp;
  //beta4*P*T
  tmp = tmp_p * value[4];
  tmp = tmp * tmp_t;
  tmp = tmp * QMP6989_POWER_SCALE[power[4]];
  tmp = ROUND_DIVIDE(tmp, 1000000000);
  val += tmp;
  //beta5*P*T*T
  tmp = tmp_p * value[5];
  tmp = tmp * tmp_t;
  tmp = SHIFT_RIGHT(tmp, 10) * tmp_t;
  tmp = SHIFT_RIGHT(tmp, 10) * QMP6989_POWER_SCALE[power[5]];
  tmp = ROUND_DIVIDE(tmp, 95367432);
  val += tmp;
  //beta6*P*P
  tmp = tmp_p * value[6];
  tmp = tmp * tmp_p;
  tmp = SHIFT_RIGHT(tmp, 7) * QMP6989_POWER_SCALE[power[6]];
  tmp = ROUND_DIVIDE(tmp, 781250000);
  val += tmp;
  //beta7*P*P*T
  tmp = tmp_p * value[7];
  tmp = tmp * tmp_p;
  tmp = SHIFT_RIGHT(tmp, 10) * tmp_t;
  tmp = SHIFT_RIGHT(tmp, 10) * QMP6989_POWER_SCALE[power[7]];
  tmp = ROUND_DIVIDE(tmp, 9536743164);
  val += tmp;
  //beta8*P*P*T*T
  tmp = tmp_p * value[8];
  tmp = tmp * tmp_p;
  tmp = SHIFT_RIGHT(tmp, 9) * SHIFT_RIGHT(tmp_t, 1);
  tmp = SHIFT_RIGHT(tmp, 12) * SHIFT_RIGHT(tmp_t, 3);
  tmp = SHIFT_RIGHT(tmp, 7) * QMP6989_POWER_SCALE[power[8]];
  tmp = ROUND_DIVIDE(tmp, 23283064365);
  val += tmp;

  *p_pa = (int32_t)ROUND_DIVIDE(val, 10);
}

/*!
 * @brief qmp6989 set pressure OSR
 *
 * @param osrP OSR to set
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int qmp6989_set_p_osr(qmp6989_device_t dev, QMP6989_P_OSR_Type osrP)
{
    int ret = RT_EOK;
    uint8_t data;

    //Read A6h
    ret = qmp6989_burst_read(dev, QMP6989_REG_CONFIG2, &data, 1);
    if (ret < 0)
        goto __exit;

    //Set the A6h[2:0] OSR bits
    data = QMP6989_SET_BITSLICE(data, QMP6989_P_OSR, osrP);
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG2, &data, 1);
    if (ret < 0)
        goto __exit;

__exit:
    return ret;
}

/*!
 * @brief qmp6989 set temperature OSR
 *
 * @param osrT OSR to set
 *
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int qmp6989_set_t_osr(qmp6989_device_t dev, QMP6989_T_OSR_Type osrT)
{
    int ret = RT_EOK;
    uint8_t data;

    //Read A7h
    ret = qmp6989_burst_read(dev, QMP6989_REG_CONFIG3, &data, 1);
    if (ret < 0)
        goto __exit;

    //Set the A7h[2:0] OSR bits
    data = QMP6989_SET_BITSLICE(data, QMP6989_T_OSR, osrT);
    ret = qmp6989_burst_write(dev, QMP6989_REG_CONFIG3, &data, 1);
    if (ret < 0)
        goto __exit;

__exit:
    return ret;
}

void qmp6989_get_data(qmp6989_device_t dev, int32_t *baro, int16_t *temp)
{
	qmp6989_measure_pressure(dev);
	qmp6989_measure_temperature(dev);

#ifdef PKG_QMP6989_SUPPORT_FLOAT
	qmp6989_compensation(dev, f_calibparam, &ft_celsius, &fp_pa);
	*baro = fp_pa;
	*temp = ft_celsius;
#else
	qmp6989_compensation_fixed_point_s64(dev, g_value_record, g_power_record, &g_celsius, &g_pa);
	*baro = g_pa;
	*temp = g_celsius;
//	*temp = g_celsius / 256.0;
#endif
}

