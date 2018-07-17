/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : main.c
 *
 * Usage: main function
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file main.c
 *  @brief main program
 *  @author Joseph FC Tseng
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c_gmems.h"
#include "gma30xku.h"
#include "gSensor_autoNil.h"
#include "AKFS_APIs.h"
#include "eCompassConfig.h"
#ifdef USE_GMC303
#include "gmc303.h"
#else
#include "gmc306x.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define SYSTICK_US                  (1000)             // system tick in us
#define SENSOR_SAMPLING_RATE_HZ     (40)               // Sensor sampling rate
#define ALGORITHM_DATA_RATE_HZ      (8)                // Algorithm data rate
#define PRINTOUT_RATE_HZ            (1)                // Orientation printout rate
#define SENSOR_SAMPLING_TICK        (1000000 / SYSTICK_US / SENSOR_SAMPLING_RATE_HZ)
#define MAG_LAYOUT_PATTERN          PAT2   //magnetometer layout pattern
#define ACC_LAYOUT_PATTERN          PAT5   //accelerometer layout pattern

/* global variables ---------------------------------------------------------*/
u8 ui8StartAutoNilFlag = 0;

/* Private variables ---------------------------------------------------------*/
USART_InitType USART_InitStructure;
static __IO uint32_t TimingDelay, TimingPeriodicMeasure;
static __IO uint8_t ui8PeriodicMeasureFlag;
static AKMPRMS akmdfsPRMS;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime * 1000 / SYSTICK_US;

  while(TimingDelay != 0);
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00){
    TimingDelay--;
  }
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingPeriodicMeasure_Decrement(void)
{
  if (TimingPeriodicMeasure != 0x00){
    TimingPeriodicMeasure--;
  }
  else{
    ui8PeriodicMeasureFlag = 1;
    TimingPeriodicMeasure = SENSOR_SAMPLING_TICK;
  }
}

/**
 * @brief  Retargets the C library printf function to the USART1.
 * @param
 * @retval
 */
int fputc(int ch, FILE *f)
{
  while((USART1->STS & 0X40) == 0)
    ;

  USART1->DT = (u8)ch;
  return ch;
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures COM port.
 * @param  None
 * @retval None
 */
void USART_COMInit()
{

  GPIO_InitType GPIO_InitStructure;

  /* USARTx configured as follow:
     - BaudRate = 115200 baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity check
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);

  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pins = TX_PIN_NUMBER;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_Init(TXRX_GPIOx, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pins = RX_PIN_NUMBER;
  GPIO_Init(TXRX_GPIOx, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

  /* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the
     EVAL_COM1 receive data register is not empty */
  USART_INTConfig(USART1, USART_INT_RDNE, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
    {}
}

#endif

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  RCC_ClockType RccClkSource;
  bus_support_t gma30xku_bus, gmc30x_bus;
  raw_data_xyzt_t gRawData, mRawData;
  float_xyzt_t gOffsetData;
  s16 i16Accuracy, i16Accuracy_pre, i16Orientation, i16RawData[3];
  s32 i, algIcounter, printIcounter = 0;
  u8 u8Asaxyz[3]; //ASAX/Y/Z
  float_xyzt_t fv_avec;
  float_xyzt_t fv_hvec;
  AKFVEC fv_ho_pre = {0.0f, 0.0f, 0.0f};
  AKFLOAT f_azimuth, f_pitch, f_roll;
  const s32 ALG_OSR = (s32)(((float)SENSOR_SAMPLING_RATE_HZ) / ((float)ALGORITHM_DATA_RATE_HZ) + 0.5f);
  const s32 PRINT_OSR = (s32)(((float)ALGORITHM_DATA_RATE_HZ) / ((float)PRINTOUT_RATE_HZ) + 0.5f);
  /* NVIC configuration */
  NVIC_Configuration();

  /* USART COM configuration */
  USART_COMInit();

  /* I2C1 initialization */
  I2C1_Init();

  RCC_GetClocksFreq(&RccClkSource);
  if (SysTick_Config(RccClkSource.AHBCLK_Freq / (1000000 / SYSTICK_US))){
    /* Capture error */
    while(1);
  }

  /* GMA30xKU I2C bus setup */
  bus_init_I2C1(&gma30xku_bus, GMA30xKU_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gma30xku_bus_init(&gma30xku_bus);  //Initailze GMA30xKU bus to I2C1

  /* GMC30X I2C bus setup */
#ifdef USE_GMC303
  bus_init_I2C1(&gmc30x_bus, GMC303_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gmc303_bus_init(&gmc30x_bus);  //Initailze GMC303 bus to I2C1
#else
  bus_init_I2C1(&gmc30x_bus, GMC306_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gmc306_bus_init(&gmc30x_bus);  //Initailze GMC306x bus to I2C1
#endif

  /* GMA30xKU soft reset */
  gma30xku_soft_reset();

#ifdef USE_GMC303
  /* GMC303 soft reset */
  gmc303_soft_reset();
#else
  /* GMC306x soft reset */
  gmc306_soft_reset();
#endif

  /* Wait 10ms for reset complete */
  Delay(10);

  /* GMA30xKU initialization */
  gma30xku_initialization();

  //Read ASAX/Y/Z
#ifdef USE_GMC303
  gmc303_burst_read(GMC303_ASA_XYZ_START__REG, u8Asaxyz, 3);
#else
  gmc306_burst_read(GMC306_ASA_XYZ_START__REG, u8Asaxyz, 3);
#endif
  printf("ASAX/Y/Z=0x%02X, 0x%02X, 0x%02X\n", u8Asaxyz[0], u8Asaxyz[1], u8Asaxyz[2]);

  //Set to CM 50Hz
#ifdef USE_GMC303
  gmc303_set_operation_mode(GMC303_OP_MODE_CM_50HZ);
#else
  gmc306_set_operation_mode(GMC306_OP_MODE_CM_50HZ);
#endif

  /* GMA30xKU Offset AutoNil */
  printf("Place and hold g-sensor in level for offset AutoNil.\r");
  printf("Press y when ready.\n");

  do{
    Delay(10);
  }while(ui8StartAutoNilFlag == 0);

  //Conduct g-sensor AutoNil, gravity is along the positive Z-axis
  gSensorAutoNil_f(gma30xku_read_data_xyz, AUTONIL_POSITIVE + AUTONIL_Z, GMA30xKU_RAW_DATA_SENSITIVITY, &gOffsetData);
  printf("gOffset_XYZ=%.1f, %.1f, %.1f\n", gOffsetData.u.x, gOffsetData.u.y, gOffsetData.u.z);

  //Initialization akmdfs algorithm
  AKFS_Init(&akmdfsPRMS,
	    (AKFS_PATNO)MAG_LAYOUT_PATTERN,
	    (AKFS_PATNO)ACC_LAYOUT_PATTERN,
	    u8Asaxyz,
	    gOffsetData.v,
#ifdef USE_GMC303
	    GMC303_RAW_DATA_SENSITIVITY,
#else
	    GMC306_RAW_DATA_SENSITIVITY,
#endif
	    GMA30xKU_RAW_DATA_SENSITIVITY);

  //Start akmdfs algorithm
  AKFS_Start(&akmdfsPRMS);

  printf("Data rate: %dHz, Alg rate: %dHz, OSR: %d, %d\n",
	 SENSOR_SAMPLING_RATE_HZ, ALGORITHM_DATA_RATE_HZ, ALG_OSR, PRINT_OSR);

  while (1){

    if(ui8PeriodicMeasureFlag){

      ui8PeriodicMeasureFlag = 0;

      // Read g-sensor data
      gma30xku_read_data_xyzt(&gRawData);

      // Read m-sensor data
#ifdef USE_GMC303
      gmc303_read_data_xyz(&mRawData);
#else
      gmc306_read_data_xyz(&mRawData);
#endif
      if(++algIcounter < ALG_OSR) continue;
      algIcounter = 0; //reset the counter

      // Feed accelerometer readings to algorithm
      for(i = 0; i < 3; ++i){
	i16RawData[i] = gRawData.v[i];
      }
      AKFS_Get_ACCELEROMETER(&akmdfsPRMS,
			     i16RawData,
			     0, //status of accelerometer, not used
			     &fv_avec.u.x, &fv_avec.u.y, &fv_avec.u.z,
			     &i16Accuracy);

      // Feed magnetometer readings to algorithm
      for(i = 0; i < 3; ++i){
	i16RawData[i] = mRawData.v[i];
      }
      AKFS_Get_MAGNETIC_FIELD(&akmdfsPRMS,
			      i16RawData,
			      0x01,  //success
			      &fv_hvec.u.x, &fv_hvec.u.y, &fv_hvec.u.z,
			      &i16Accuracy);

      if(fv_ho_pre.u.x != akmdfsPRMS.fv_ho.u.x ||
	 fv_ho_pre.u.y != akmdfsPRMS.fv_ho.u.y ||
	 fv_ho_pre.u.z != akmdfsPRMS.fv_ho.u.z ||
	 i16Accuracy != i16Accuracy_pre){

	//print magnetometer offset
	printf("ho(uT)@%d=%.2f, %.2f, %.2f(%.2fr)\n",
	       i16Accuracy,
	       akmdfsPRMS.fv_ho.u.x, akmdfsPRMS.fv_ho.u.y, akmdfsPRMS.fv_ho.u.z,
	       akmdfsPRMS.s_aocv.hraoc);
      }

      for(i = 0; i < 3; ++i)
	fv_ho_pre.v[i] = akmdfsPRMS.fv_ho.v[i];

      i16Accuracy_pre = i16Accuracy;

      if(i16Accuracy == 0) continue;

      //  Get orientation
      AKFS_Get_ORIENTATION(&akmdfsPRMS,
			   &f_azimuth,
			   &f_pitch,
			   &f_roll,
			   &i16Orientation);

      if(++printIcounter < PRINT_OSR) continue;
      printIcounter = 0; //reset the counter

      //print orientation
      printf("y,p,r=%.2f, %.2f, %.2f\n", f_azimuth, f_pitch, f_roll);
    }
  }
}

