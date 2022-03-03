#ifndef __QMP6989_H__
#define __QMP6989_H__

#include <stdint.h>
#include <rtdef.h>
#include <qmp6989_reg.h>

#ifdef __cplusplus 
extern "C" { 
#endif

struct qmp6989_device {
    rt_device_t bus;
    uint8_t i2c_addr;
    int16_t temp;
    int32_t baro;
};
typedef struct qmp6989_device *qmp6989_device_t;




//int qmp6989_get_calibration_param(qmp6989_device_t dev, float* f_calibparam);
//int qmp6989_get_calibration_param_fixed_point(qmp6989_device_t dev, int16_t s16Value[], uint8_t u8Power[]);
qmp6989_device_t    qmp6989_init(const char *dev_name, rt_uint8_t param);
void                qmp6989_deinit(qmp6989_device_t dev);
//int qmp6989_measure_temperature(qmp6989_device_t dev, int16_t* ps16T);
//int qmp6989_measure_pressure(qmp6989_device_t dev, int32_t* ps32P);
int qmp6989_measure_P_T(qmp6989_device_t dev, int32_t* ps32P, int16_t* ps16T, int8_t s8PWaitDrdy);
//void qmp6989_compensation(qmp6989_device_t dev, int16_t s16T, int32_t s32P, float fParam[], float* pfT_Celsius, float* pfP_Pa);
//void qmp6989_compensation_fixed_point_s64(qmp6989_device_t dev, int16_t s16T, int32_t s32P, int16_t s16Value[], uint8_t u8Power[], int32_t* ps32T_Celsius, int32_t* ps32P_Pa);
int qmp6989_set_p_osr(qmp6989_device_t dev, QMP6989_P_OSR_Type osrP);
int qmp6989_set_t_osr(qmp6989_device_t dev, QMP6989_T_OSR_Type osrT);

void qmp6989_get_data(qmp6989_device_t dev, int32_t *baro, int16_t *temp);

#ifdef __cplusplus 
} 
#endif

#endif /* __QMP6989_H__ */
