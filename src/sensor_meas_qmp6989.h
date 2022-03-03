#ifndef __SENSOR_MEAS_QMP6989_H__
#define __SENSOR_MEAS_QMP6989_H__

#include <stdint.h>
#include <rtdef.h>
#include <sensor.h>
#include <qmp6989.h>

#ifdef __cplusplus 
extern "C" { 
#endif

int rt_hw_qmp6989_init(const char *name, struct rt_sensor_config *cfg);

#ifdef __cplusplus 
} 
#endif

#endif /* __SENSOR_MEAS_QMP6989_H__ */
