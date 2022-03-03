#ifndef __QMP6989_REG_H__
#define __QMP6989_REG_H__

#include <stdint.h>
#include <rtdef.h>


#define QMP6989_ADDRESS_AD0_LOW                 0x6C // address pin low (GND), default for InvenSense evaluation board
#define QMP6989_ADDRESS_AD0_HIGH                0x6D // address pin high (VCC)


#define QMP6989_TEMPERATURE_SENSITIVITY 256  	//   Celsius = 256 code
#define QMP6989_T_CODE_TO_CELSIUS(tCode) (((float)(tCode)) / QMP6989_TEMPERATURE_SENSITIVITY)

//Registers Address
#define QMP6989_REG_RESET		0x00
#define QMP6989_REG_PID			0x01
#define QMP6989_REG_STATUS		0x02
#define QMP6989_REG_PRESSH		0x06
#define QMP6989_REG_PRESSM		0x07
#define QMP6989_REG_PRESSL		0x08
#define QMP6989_REG_TEMPH		0x09
#define QMP6989_REG_TEMPL		0x0A
#define QMP6989_REG_CMD			0x30
#define QMP6989_REG_CONFIG1		0xA5
#define QMP6989_REG_CONFIG2		0xA6
#define QMP6989_REG_CONFIG3		0xA7
#define QMP6989_REG_CALIB00		0xAA
//Total calibration register count: AAh~BBh total 18
#define QMP6989_CALIBRATION_REGISTER_COUNT 18
//Total calibration parameter count: total 9
#define QMP6989_CALIBRATION_PARAMETER_COUNT (QMP6989_CALIBRATION_REGISTER_COUNT/2)


/* PID */
#define QMP6989_PID_VAL         0x02
/* Soft Rest bit */
#define QMP6989_RST_SET_VAL     0x24
/* DRDY bit */
#define QMP6989_DRDY__REG       QMP6989_REG_STATUS
#define QMP6989_DRDY__MSK       0x01
#define QMP6989_DRDY__POS       0
/* P OSR bits */
#define QMP6989_P_OSR__REG      QMP6989_REG_CONFIG2
#define QMP6989_P_OSR__MSK      0x07
#define QMP6989_P_OSR__POS      0
/* T OSR bits */
#define QMP6989_T_OSR__REG      QMP6989_REG_CONFIG3
#define QMP6989_T_OSR__MSK      0x07
#define QMP6989_T_OSR__POS      0

#define QMP6989_GET_BITSLICE(regvar, bitname)	\
    ((regvar & bitname##__MSK) >> bitname##__POS)

#define QMP6989_SET_BITSLICE(regvar, bitname, val)			\
    ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

typedef enum {
    QMP6989_P_OSR_256   = 0x04,
    QMP6989_P_OSR_512   = 0x05,
    QMP6989_P_OSR_1024  = 0x00,
    QMP6989_P_OSR_2048  = 0x01,
    QMP6989_P_OSR_4096  = 0x02,
    QMP6989_P_OSR_8192  = 0x03,
    QMP6989_P_OSR_16384 = 0x06,
    QMP6989_P_OSR_32768 = 0x07,	
} QMP6989_P_OSR_Type;

typedef enum {
    QMP6989_T_OSR_256   = 0x04,
    QMP6989_T_OSR_512   = 0x05,
    QMP6989_T_OSR_1024  = 0x00,
    QMP6989_T_OSR_2048  = 0x01,
    QMP6989_T_OSR_4096  = 0x02,
    QMP6989_T_OSR_8192  = 0x03,
    QMP6989_T_OSR_16384 = 0x06,
    QMP6989_T_OSR_32768 = 0x07,	
} QMP6989_T_OSR_Type;

#endif /* __QMP6989_REG_H__ */
