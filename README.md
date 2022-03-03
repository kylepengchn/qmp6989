- # qmp6989

  ## 简介

  本软件包是为 QST矽睿公司的气压传感器提供的通用传感器驱动包。本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。

  ## 支持情况

  | 包含设备     | 气压计 | 温度计 |
  | ------------ | ------ | ------ |
  | **通讯接口** |        |        |
  | IIC          | √      | √      |
  | SPI          | √      | √      |
  | **工作模式** |        |        |
  | 轮询         | √      | √      |
  | 中断         |        |        |
  | FIFO         |        |        |
  | **电源模式** |        |        |
  | 掉电         | √      | √      |

  ## 使用说明

  ### 依赖

  - RT-Thread 3.1.0+
  - Sensor 组件
  - IIC/SPI 驱动：QMP6989 设备使用 IIC/SPI 进行数据通讯，需要系统 IIC/SPI 驱动支持；

  ### 获取软件包

  使用 QMP6989 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

  ```
  RT-Thread online packages  --->
    peripheral libraries and drivers  --->
      sensors drivers  --->
        QMP6989: qmp6989: High accuracy and small size barometric pressure sensor,support: barometer, temperature.
  			  qmp6989 select intf(intf i2c)  --->
  	   (i2c0) qmp6989 intf i2c bus name
  	   (0x6c) qmp6989 intf i2c addr
  	   [] 	  support float
                Version (latest)  --->
  ```

  **support float**：已接入sensor框架，不使用

  ### 使用软件包

  QMP6989 软件包初始化函数如下所示：

  ```
  int rt_hw_qmp6989_init(const char *name, struct rt_sensor_config *cfg);
  ```

  该函数需要由用户调用，函数主要完成的功能有，

  - 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
  - 注册相应的传感器设备，完成 QMP6989 设备的注册；

  #### 初始化示例

  ```
  #include "sensor_meas_qmp6989.h"
  
  int rt_hw_qmp6989_port(void)
  {
      struct rt_sensor_config cfg;
      
      cfg.intf.dev_name = "i2c0";
      cfg.intf.type      = RT_SENSOR_INTF_I2C;
      cfg.intf.user_data = (void *)QMP6989_ADDR_DEFAULT;
  
      rt_hw_qmp6989_init("qmp6989", &cfg);
      return 0;
  }
  INIT_APP_EXPORT(rt_hw_qmp6989_port);
  ```

  ## 注意事项

  暂无

  ## 联系人信息

  维护人:

  - [kylepengchn](https://github.com/kylepengchn) 

  - 主页：<https://github.com/kylepengchn/qmp6989>

