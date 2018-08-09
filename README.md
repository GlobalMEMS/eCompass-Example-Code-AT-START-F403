AT-START-F403 + GMEMS eCompass example code
===========================================

Requirements
-----------
- AT-START-F403 development board
- GMEMS eCompass
  - magentometer: GMC303, GMC306 or GMC306A
  - accelerometer: GMA302KU, GMA303KU, or GMA305KU

The example code default assumes using **Sensor Fusion Arduino Daughter Board V1.0**, which contains GMA303KU and GMC306A.

I2C Connections
---------------
- Use I2C1
  - SCL: PB8
  - SDA: PB9
- Magnetometer
  - GMAC303 I2C 7-bit slave address: 0x0C
  - GMC306x I2C 7-bit slave address: 0x0C
- Accelerometer
    - GMA30xKU I2C 7-bit slave address: 0x18

Magnetometer selection
----------------------
* Default magnetometer is GMC306x. Uncoment the `USE_GMC303` macro in the **eCompassConfig.h** to select GMC303.
```
// Default magnetometer GMC306x
// Uncommet below macro to select GMC303
//#define USE_GMC303
```

* For GMC306x, default sensor is GMC306A. For GMC306, comment out the `USE_GMC306A` macro in the **gmc306x.h** for correct configuration.
```
/* Uncomment below macro for GMC306A */
#define USE_GMC306A
```

Sensor Layout Pattern
---------------------
Sensor layout pattern is defined by the following macro in the "main.c". The default layout pattern definition is referred to the **Sensor Fusion Arduino Daughter Board V1.0**.
```
#define MAG_LAYOUT_PATTERN          PAT2   //magnetometer layout pattern
#define ACC_LAYOUT_PATTERN          PAT5   //accelerometer layout pattern
```

Please refer to the ["Sensor_Layout_Pattern_Definition.pdf"](https://github.com/GlobalMEMS/eCompass-Example-Code-AT-START-F403/blob/master/Sensor_Layout_Pattern_Definition.pdf) document for the definition and modify accordingly to fit your actual layout.

Usage of AutoNil
----------------
 * The program will do an g-sensor offset AutoNil when executed. Hold the board steady and maintain in level facing up, then press **Key1** after the program prompt for input.
 * You may change the `DATA_AVE_NUM` macro in the gSensor_autoNil.h for the moving averae order for the offset estimation. Defautl is 32.

Debug print
-----------
To enable debug output, uncomment the following macro in "akmdfs/AKFS_Log.h" and rebuild the project.
```
//#define ENABLE_AKMDEBUG	1
```

AKMDFS Library
--------------
Hard iron offset is compensated by the AKMDFS demo library, which is adapted from [Compass Control Program for Android Open Source Project](https://github.com/akm-multisensor/AKMDFS).

The compensation algorithm data rate should be kept at 8Hz regardless of the sensor sampling rate. The sensor sampling and algorithm data rate is defined by the following macro in the "main.c".
```
#define SENSOR_SAMPLING_RATE_HZ     (40)               // Sensor sampling rate
#define ALGORITHM_DATA_RATE_HZ      (8)                // Algorithm data rate
```

For general description of the hard iron effect, please refer to the application note ["Application guidelines for using GMEMS eCompass"](https://github.com/GlobalMEMS/Application-Notes/blob/master/Application%20guidelines%20for%20using%20GMEMS%20eCompass%20V1.0.pdf).


