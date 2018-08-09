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
#include "Lcd_Driver.h"
#include "GUI.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "string.h"
#include "math.h"

/* Private macro -------------------------------------------------------------*/
#define LED2_GPIO_PIN    GPIO_Pins_13
#define LED3_GPIO_PIN    GPIO_Pins_14
#define LED4_GPIO_PIN    GPIO_Pins_15
#define LED_GPIO_PORT    GPIOD
#define SENSOR_SAMPLING_RATE_HZ     (40)               // Sensor sampling rate
#define ALGORITHM_DATA_RATE_HZ      (8)                // Algorithm data rate
#define MAG_LAYOUT_PATTERN          PAT2               //magnetometer layout pattern
#define ACC_LAYOUT_PATTERN          PAT5               //accelerometer layout pattern
#define SMPLRT_TMR_TICK_FREQ_kHZ       5               // Sensor sampling timer tick@5kHz

/* global variables ---------------------------------------------------------*/
u8 ui8PeriodicMeasureFlag = 0;

/* Private variables ---------------------------------------------------------*/
static AKMPRMS akmdfsPRMS;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  GPIO Initialize For LED.
 * @param  None
 * @retval None
 */
void LED_Init(void)
{
  GPIO_InitType GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);

  /*PD13->LED2 PD14->LED3 PD15->LED4*/
  GPIO_InitStructure.GPIO_Pins = LED2_GPIO_PIN | LED3_GPIO_PIN | LED4_GPIO_PIN;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
  GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

  GPIO_SetBits(LED_GPIO_PORT, LED2_GPIO_PIN);
  GPIO_SetBits(LED_GPIO_PORT, LED3_GPIO_PIN);
  GPIO_SetBits(LED_GPIO_PORT, LED4_GPIO_PIN);

}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;

  /* Enable the TMR6 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TMR6Init(){

  RCC_ClockType RccClkSource;
  TMR_TimerBaseInitType  TIM_TimeBaseStructure;

  /* TMR6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);

  /* Init the TMR6 configuration */
  RCC_GetClocksFreq(&RccClkSource);
  TIM_TimeBaseStructure.TMR_Period = 1000 * SMPLRT_TMR_TICK_FREQ_kHZ / SENSOR_SAMPLING_RATE_HZ - 1;
  TIM_TimeBaseStructure.TMR_DIV = RccClkSource.AHBCLK_Freq / SMPLRT_TMR_TICK_FREQ_kHZ / 1000 - 1;
  TIM_TimeBaseStructure.TMR_ClockDivision = 0;
  TIM_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Down;
  TMR_TimeBaseInit(TMR6, &TIM_TimeBaseStructure);

  /* Enable TMR6 interrupt */
  TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);

  /* TMR6 Enable */
  TMR_Cmd(TMR6, ENABLE);
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

const u16 RESOLUTION_X = 128;
const u16 RESOLUTION_Y = 120;
const u16 FONT_HEIGHT = 16;
const u16 LINE_HEIGHT = FONT_HEIGHT + 2;
const u16 MAX_DISPLAY_ITEM = 9;
void showMsg(u16 x, u16 line, u8* str, u16 color, u8 reDraw){

  int i;
  char* subStr;

  if(reDraw) Lcd_Clear(GRAY0);

  subStr = strtok((char*)str, "\n");

  for(i = line; subStr; ++i){
    Gui_DrawFont_GBK16(x, LINE_HEIGHT * i, color, GRAY0, (u8*)subStr);
    subStr = strtok(NULL, "\n");
  }
}

void floatCatToStr(float fIn, u8 precision, u8* outStr){

  s32 i = 0;
  float fTmp;
  s32 s32Dec, s32Dig;

  if(fIn < 0){
    fIn = -fIn;
    strcat((char*)outStr, "-");
  }

  s32Dec = (s32)fIn;
  fTmp = fIn - s32Dec;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  s32Dig = (s32)(fTmp + 0.5f);

  itoa(s32Dec, &outStr[strlen((const char*)outStr)]);
  strcat((char*)outStr, ".");

  fTmp = 1;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  for(i = 0; i < precision; ++i){
    fTmp /= 10;
    if(s32Dig < fTmp){
      strcat((char*)outStr, "0");
    }
    else{
      itoa(s32Dig, &outStr[strlen((const char*)outStr)]);
      break;
    }
  }
}

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  bus_support_t gma30xku_bus, gmc30x_bus;
  raw_data_xyzt_t gRawData, mRawData;
  float_xyzt_t gOffsetData;
  s16 i16Accuracy, i16Accuracy_pre, i16Orientation, i16RawData[3];
  s32 i, algIcounter;
  u8 u8Asaxyz[3]; //ASAX/Y/Z
  float_xyzt_t fv_avec;
  float_xyzt_t fv_hvec;
  AKFVEC fv_ho_pre = {0.0f, 0.0f, 0.0f};
  AKFLOAT f_azimuth, f_pitch, f_roll;
  const s32 ALG_OSR = (s32)(((float)SENSOR_SAMPLING_RATE_HZ) / ((float)ALGORITHM_DATA_RATE_HZ) + 0.5f);
  u8 str[64];

  /* NVIC configuration */
  NVIC_Configuration();

  /* User LED initialization */
  LED_Init();

  /* I2C1 initialization */
  I2C1_Init();

  /* TMR6 initialization */
  TMR6Init();

  /* Init Key */
  KEY_Init();

  /* Initialize the LCD */
  uart_init(19200);
  delay_init();
  Lcd_Init();

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
  delay_ms(10);

  /* GMA30xKU initialization */
  gma30xku_initialization();

  //Read ASAX/Y/Z
#ifdef USE_GMC303
  gmc303_burst_read(GMC303_ASA_XYZ_START__REG, u8Asaxyz, 3);
#else
  gmc306_burst_read(GMC306_ASA_XYZ_START__REG, u8Asaxyz, 3);
#endif

  /* User message: show sensitivity adjustment value */
#ifdef USE_GMC303
  strcpy((char*)str, "GMC303\n");
#else
#ifdef USE_GMC306A
  strcpy((char*)str, "GMC306A\n");
#else
  strcpy((char*)str, "GMC306\n");
#endif
#endif
  strcat((char*)str, "ASAX= ");
  itoa(u8Asaxyz[0], &str[strlen((const char*)str)]);
  strcat((char*)str, "\nASAY= ");
  itoa(u8Asaxyz[1], &str[strlen((const char*)str)]);
  strcat((char*)str, "\nASAZ= ");
  itoa(u8Asaxyz[2], &str[strlen((const char*)str)]);
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 to\ncontinue");
  showMsg(0, 5, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  //Set to CM 50Hz
#ifdef USE_GMC303
  gmc303_set_operation_mode(GMC303_OP_MODE_CM_50HZ);
#else
  gmc306_set_operation_mode(GMC306_OP_MODE_CM_50HZ);
#endif

  /* User message: press Key1 to start offset AutoNil */
  strcpy((char*)str, "Hold g-sensor in\nlevel for offset\nAutoNil.");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 when\nready.");
  showMsg(0, 4, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  //Conduct g-sensor AutoNil, gravity is along the positive Z-axis
  gSensorAutoNil_f(gma30xku_read_data_xyz, AUTONIL_POSITIVE + AUTONIL_Z, GMA30xKU_RAW_DATA_SENSITIVITY, &gOffsetData);

  /* User message: show offset */
  strcpy((char*)str, "Offset(code):\nX= ");
  itoa(gOffsetData.u.x, &str[strlen((const char*)str)]);
  strcat((char*)str, "\nY= ");
  itoa(gOffsetData.u.y, &str[strlen((const char*)str)]);
  strcat((char*)str, "\nZ= ");
  itoa(gOffsetData.u.z, &str[strlen((const char*)str)]);
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 to\ncontinue");
  showMsg(0, 5, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

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

  strcpy((char*)str, "Data,Alg=");
  itoa(SENSOR_SAMPLING_RATE_HZ, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  itoa(ALGORITHM_DATA_RATE_HZ, &str[strlen((const char*)str)]);
  strcat((char*)str, "Hz");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "yaw  =");
  showMsg(0, 1, str, GRAY1, 0);
  strcpy((char*)str, "picth=");
  showMsg(0, 2, str, GRAY1, 0);
  strcpy((char*)str, "roll =");
  showMsg(0, 3, str, GRAY1, 0);
  strcpy((char*)str, "Mag offset(uT):");
  showMsg(0, 4, str, BLACK, 0);
  strcpy((char*)str, "accuracy:");
  showMsg(0, 5, str, GRAY1, 0);
  strcpy((char*)str, "X=");
  showMsg(0, 6, str, GRAY1, 0);
  strcpy((char*)str, "Y=");
  showMsg(0, 7, str, GRAY1, 0);
  strcpy((char*)str, "Z=");
  showMsg(0, 8, str, GRAY1, 0);

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

	/* User message: m-sensor offset */
	strcpy((char*)str, "");
	itoa(i16Accuracy, &str[strlen((const char*)str)]);
	showMsg(80, 5, str, BLUE, 0);
	strcpy((char*)str, "");
	floatCatToStr(akmdfsPRMS.fv_ho.u.x, 2, str);
	strcat((char*)str, "       ");
	showMsg(30, 6, str, BLUE, 0);
	strcpy((char*)str, "");
	floatCatToStr(akmdfsPRMS.fv_ho.u.y, 2, str);
	strcat((char*)str, "       ");
	showMsg(30, 7, str, BLUE, 0);
	strcpy((char*)str, "");
	floatCatToStr(akmdfsPRMS.fv_ho.u.z, 2, str);
	strcat((char*)str, "       ");
	showMsg(30, 8, str, BLUE, 0);
      }

      for(i = 0; i < 3; ++i)
	fv_ho_pre.v[i] = akmdfsPRMS.fv_ho.v[i];

      i16Accuracy_pre = i16Accuracy;

      LED_GPIO_PORT->OPTDT ^= LED4_GPIO_PIN;

      if(i16Accuracy == 0) continue;

      //  Get orientation
      AKFS_Get_ORIENTATION(&akmdfsPRMS,
			   &f_azimuth,
			   &f_pitch,
			   &f_roll,
			   &i16Orientation);

      /* User message: orientation in degree*/
      strcpy((char*)str, "");
      floatCatToStr(f_azimuth, 1, str);
      strcat((char*)str, "       ");
      showMsg(60, 1, str, BLUE, 0);
      strcpy((char*)str, "");
      floatCatToStr(f_pitch, 1, str);
      strcat((char*)str, "       ");
      showMsg(60, 2, str, BLUE, 0);
      strcpy((char*)str, "");
      floatCatToStr(f_roll, 1, str);
      strcat((char*)str, "       ");
      showMsg(60, 3, str, BLUE, 0);
    }
  }
}

