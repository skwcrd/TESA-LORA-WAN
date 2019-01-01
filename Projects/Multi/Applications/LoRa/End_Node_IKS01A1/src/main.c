/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.5
  * @date    30-March-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "sensor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define MAG_LPF_FACTOR  0.4f
#define ACC_LPF_FACTOR  0.1f
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_ANALOG_INPUT  	0x2
//#define LPP_DATATYPE_ANALOG_OUTPUT 	0x3
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_DATATYPE_ACCELERO    		0x71
#define LPP_DATATYPE_GYRO    				0x86
#define LPP_DATATYPE_GPS    				0x88
#define LPP_APP_PORT 								99
/*!
 * Defines the application data transmission duty cycle. 30s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            30000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE 													LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE 									DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                  64
/*!
 * Defines the magneto data transmission duty cycle. 1s, value in [ms].
 */
#define MAG_DUTYCYCLE                            		1000
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData = { AppDataBuff, 0, 0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData );

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* calculate heading from magneto value*/
static void Cal_Heading( void );

/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx( TxEventType_t EventType );

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* megneto timer callback function*/
static void OnMagTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded };

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
                                               
static TimerEvent_t TxTimer;
static TimerEvent_t MagTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit = { LORAWAN_ADR_STATE,
																			LORAWAN_DEFAULT_DATA_RATE,  
																			LORAWAN_PUBLIC_NETWORK };

/* Private functions ---------------------------------------------------------*/

extern SensorAxesRaw_t MAGNETO_Value_Raw;
extern SensorAxesRaw_t ACCELERO_Value_Raw;
extern SensorAxes_t ACCELERO_Value;
extern SensorAxes_t GYRO_Value;

SensorAxesRaw_t MAG_MIN = { 0XFE64, 0XE43B, 0 };
SensorAxesRaw_t MAG_MAX = { 0X1987, 0, 0X1774 };
SensorAxesRaw_t MAGNETO_Value_Old 	= { 0, 0, 0 };
SensorAxesRaw_t ACCELERO_Value_Old 	= { 0, 0, 0 };
																			
float HEADING;
float TEMP_VALUE;
float PRESSURE_VALUE;
float HUMIDITY_VALUE;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init();
  
  /* Configure the system clock*/
  SystemClock_Config();
  
  /* Configure the debug mode*/
  DBG_Init();
  
  /* Configure the hardware*/
  HW_Init();
  
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  PRINTF("VERSION: %X\n\r", VERSION);
  
  LORA_Join();
  
  LoraStartTx( TX_ON_TIMER) ;
  
  while( 1 )
  {
    DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */

#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower( );
#endif

    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Cal_Heading( void )
{
	float MAG_X_Comp, MAG_Y_Comp, ACC_X_Norm, ACC_Y_Norm, pitch, roll;
	SensorAxesRaw_t magnetos 	= { 0, 0, 0 };
	SensorAxesRaw_t accelero 	= { 0, 0, 0 };
	
	BSP_Magneto_sensor_Read();
	
	// Calculator Heading
	if ( MAGNETO_Value_Raw.AXIS_X < MAG_MIN.AXIS_X ) {
		MAG_MIN.AXIS_X = MAGNETO_Value_Raw.AXIS_X;
	}
	if ( MAGNETO_Value_Raw.AXIS_Y < MAG_MIN.AXIS_Y ) {
		MAG_MIN.AXIS_Y = MAGNETO_Value_Raw.AXIS_Y;
	}
	if ( MAGNETO_Value_Raw.AXIS_Z < MAG_MIN.AXIS_Z ) {
		MAG_MIN.AXIS_Z = MAGNETO_Value_Raw.AXIS_Z;
	}
	if ( MAGNETO_Value_Raw.AXIS_X > MAG_MAX.AXIS_X ) {
		MAG_MAX.AXIS_X = MAGNETO_Value_Raw.AXIS_X;
	}
	if ( MAGNETO_Value_Raw.AXIS_Y > MAG_MAX.AXIS_Y ) {
		MAG_MAX.AXIS_Y = MAGNETO_Value_Raw.AXIS_Y;
	}
	if ( MAGNETO_Value_Raw.AXIS_Z > MAG_MAX.AXIS_Z ) {
		MAG_MAX.AXIS_Z = MAGNETO_Value_Raw.AXIS_Z;
	}
	
	magnetos.AXIS_X = ( MAGNETO_Value_Raw.AXIS_X * MAG_LPF_FACTOR + MAGNETO_Value_Old.AXIS_X * ( 1 - MAG_LPF_FACTOR ) );
	magnetos.AXIS_Y = ( MAGNETO_Value_Raw.AXIS_Y * MAG_LPF_FACTOR + MAGNETO_Value_Old.AXIS_Y * ( 1 - MAG_LPF_FACTOR ) );
	magnetos.AXIS_Z = ( MAGNETO_Value_Raw.AXIS_Z * MAG_LPF_FACTOR + MAGNETO_Value_Old.AXIS_Z * ( 1 - MAG_LPF_FACTOR ) );
	
	accelero.AXIS_X = ( ACCELERO_Value_Raw.AXIS_X * ACC_LPF_FACTOR + ACCELERO_Value_Old.AXIS_X * ( 1 - ACC_LPF_FACTOR ) );
	accelero.AXIS_Y = ( ACCELERO_Value_Raw.AXIS_Y * ACC_LPF_FACTOR + ACCELERO_Value_Old.AXIS_Y * ( 1 - ACC_LPF_FACTOR ) );
	accelero.AXIS_Z = ( ACCELERO_Value_Raw.AXIS_Z * ACC_LPF_FACTOR + ACCELERO_Value_Old.AXIS_Z * ( 1 - ACC_LPF_FACTOR ) );
	
	MAGNETO_Value_Old 	= magnetos;
	ACCELERO_Value_Old 	= accelero;
	
	magnetos.AXIS_X -= ( ( MAG_MIN.AXIS_X + MAG_MAX.AXIS_X ) / 2 );
	magnetos.AXIS_Y -= ( ( MAG_MIN.AXIS_Y + MAG_MAX.AXIS_Y ) / 2 );
	magnetos.AXIS_Z -= ( ( MAG_MIN.AXIS_Z + MAG_MAX.AXIS_Z ) / 2 );
	
	ACC_X_Norm = ( accelero.AXIS_X / sqrt( accelero.AXIS_X * accelero.AXIS_X + accelero.AXIS_Y * accelero.AXIS_Y + accelero.AXIS_Z * accelero.AXIS_Z ) );
	ACC_Y_Norm = ( accelero.AXIS_Y / sqrt( accelero.AXIS_X * accelero.AXIS_X + accelero.AXIS_Y * accelero.AXIS_Y + accelero.AXIS_Z * accelero.AXIS_Z ) );
	
	pitch = ( asin( ACC_X_Norm ) );
	roll 	= ( -asin( ACC_Y_Norm / cos( pitch ) ) );
	
	MAG_X_Comp 	= ( magnetos.AXIS_X * cos( pitch ) + magnetos.AXIS_Z * sin( pitch ) );
	MAG_Y_Comp 	= ( magnetos.AXIS_X * sin( roll ) * sin( pitch ) + magnetos.AXIS_Y * cos( roll ) + magnetos.AXIS_Z * sin( roll ) * cos( pitch ) );
	
	HEADING			= ( 180.0 * atan2( MAG_Y_Comp, MAG_X_Comp ) / M_PI );
	
	if ( ( HEADING <= -180.0 ) || ( HEADING >= 180.0 ) ) {
		HEADING = 0.0;
	}
	else if ( ( HEADING > -180.0 ) && ( HEADING < 0.0 ) ) {
		HEADING = 180 + HEADING;
	}
	else if ( ( HEADING < 180.0 ) && ( HEADING >= 0.0 ) ) {
		HEADING = -( 180 - HEADING );
	}
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */
  uint16_t 	pressure 		= 0;
  int16_t 	temperature = 0;
  uint16_t 	humidity 		= 0;
	int16_t 	magneto 		= 0;
	/*int32_t 	latitude 		= 0;
	int32_t 	longitude 	= 0;
	int32_t 	altitude 		= 0;*/
	SensorAxesRaw_t accelero 	= { 0, 0, 0 };
	SensorAxesRaw_t gyro 			= { 0, 0, 0 };
  uint8_t batteryLevel;
  sensor_t sensor_data;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
  
  DBG_PRINTF("SEND REQUEST\n\r");
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  
#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  TimerSetValue( &TxLedTimer, 200 );
  LED_On( LED_RED1 );
  TimerStart( &TxLedTimer );
#endif

  BSP_sensor_Read( &sensor_data );
	
	TEMP_VALUE 			= sensor_data.temperature;
	PRESSURE_VALUE 	= sensor_data.pressure;
	HUMIDITY_VALUE 	= sensor_data.humidity;

#ifdef CAYENNE_LPP
  uint8_t cchannel=0;
  temperature 			= ( int16_t )( TEMP_VALUE * 10 );     				/* in �C * 10 */
  pressure    			= ( uint16_t )( PRESSURE_VALUE * 100 / 10 );  /* in hPa / 10 */
  humidity    			= ( uint16_t )( HUMIDITY_VALUE * 2 );        	/* in %*2     */
	
	// ACCELERO value from ACCELERO SENSOR to CAYENNE_LPP
	accelero.AXIS_X 	= ( int16_t )( ACCELERO_Value.AXIS_X  * 1000 );
	accelero.AXIS_Y 	= ( int16_t )( ACCELERO_Value.AXIS_Y  * 1000 );
	accelero.AXIS_Z 	= ( int16_t )( ACCELERO_Value.AXIS_Z  * 1000 );
	// GYROSCOPE value from GYROSCOPE SENSOR to CAYENNE_LPP
	gyro.AXIS_X 			= ( int16_t )( GYRO_Value.AXIS_X  * 100 );
	gyro.AXIS_Y 			= ( int16_t )( GYRO_Value.AXIS_Y  * 100 );
	gyro.AXIS_Z 			= ( int16_t )( GYRO_Value.AXIS_Z  * 100 );
	// GPS value to CAYENNE_LPP
	/*latitude 					= ( int32_t )( sensor_data.latitude * 10000 );
	longitude 				= ( int32_t )( sensor_data.longitude * 10000 );
	altitude 					= ( int32_t )( sensor_data.altitudeGps * 100 );*/		/* in m */
	
	// HEADING value from MAGNETO SENSOR to CAYENNE_LPP
	magneto 					= ( int16_t )( HEADING * 100 );
	
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( ); /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;

  //*IsTxConfirmed = LORAWAN_CONFIRMED_MSG;
	
	// PRESSURE SENSOR
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_BAROMETER;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
	
	// TEMPERATURE SENSOR
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_TEMPERATURE; 
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
	
	// HUMIDITY SENSOR
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_HUMIDITY;
  AppData.Buff[i++] = humidity & 0xFF;
	
	// ACCELERO SENSOR
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_ACCELERO;
  AppData.Buff[i++] = ( accelero.AXIS_X >> 8 ) & 0xFF;
  AppData.Buff[i++] = accelero.AXIS_X & 0xFF;
	AppData.Buff[i++] = ( accelero.AXIS_Y >> 8 ) & 0xFF;
  AppData.Buff[i++] = accelero.AXIS_Y & 0xFF;
	AppData.Buff[i++] = ( accelero.AXIS_Z >> 8 ) & 0xFF;
  AppData.Buff[i++] = accelero.AXIS_Z & 0xFF;
	
	// GYROSCOPE SENSOR
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_GYRO;
  AppData.Buff[i++] = ( gyro.AXIS_X >> 8 ) & 0xFF;
  AppData.Buff[i++] = gyro.AXIS_X & 0xFF;
	AppData.Buff[i++] = ( gyro.AXIS_Y >> 8 ) & 0xFF;
  AppData.Buff[i++] = gyro.AXIS_Y & 0xFF;
	AppData.Buff[i++] = ( gyro.AXIS_Z >> 8 ) & 0xFF;
  AppData.Buff[i++] = gyro.AXIS_Z & 0xFF;
	
	// GPS
	/*AppData.Buff[i++] = cchannel++;
	AppData.Buff[i++] = LPP_DATATYPE_GPS;
	AppData.Buff[i++] = ( latitude >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( latitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = latitude & 0xFF;
	AppData.Buff[i++] = ( longitude >> 16 ) & 0xFF;
	AppData.Buff[i++] = ( longitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = longitude & 0xFF;
	AppData.Buff[i++] = ( altitude >> 16 ) & 0xFF;
	AppData.Buff[i++] = ( altitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = altitude & 0xFF;*/
	
	// ANALOG MAGNETO SENSOR : send value range -180 to 180 degree, North is zero degree.
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_ANALOG_INPUT;
	AppData.Buff[i++] = ( magneto >> 8 ) & 0xFF;
	AppData.Buff[i++] = magneto & 0xFF;
	
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
#else
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT; 
  AppData.Buff[i++] = batteryLevel * 100 / 254;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT; 
  AppData.Buff[i++] = AppLedStateOn;
#endif  /* REGION_XX915 */
#else  /* not CAYENNE_LPP */

  temperature = ( int16_t )( sensor_data.temperature * 100 );     /* in �C * 100 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 10 );        /* in %*10     */
  latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
  AppData.Buff[i++] = 0;
#else  /* not REGION_XX915 */
  AppData.Buff[i++] = AppLedStateOn;
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData.Buff[i++] = humidity & 0xFF;
  AppData.Buff[i++] = batteryLevel;
  AppData.Buff[i++] = ( latitude >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( latitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = latitude & 0xFF;
  AppData.Buff[i++] = ( longitude >> 16 ) & 0xFF;
  AppData.Buff[i++] = ( longitude >> 8 ) & 0xFF;
  AppData.Buff[i++] = longitude & 0xFF;
  AppData.Buff[i++] = ( altitudeGps >> 8 ) & 0xFF;
  AppData.Buff[i++] = altitudeGps & 0xFF;
#endif  /* REGION_XX915 */
#endif  /* CAYENNE_LPP */
  AppData.BuffSize = i;
  
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
  
  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  DBG_PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass( CLASS_A );
          break;
        }
        case 1:
        {
          LORA_RequestClass( CLASS_B );
          break;
        }
        case 2:
        {
          LORA_RequestClass( CLASS_C );
          break;
        }
        default:
          break;
      }
    }
    break;

    case LORAWAN_APP_PORT:

    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    }
    break;

    case LPP_APP_PORT:
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      LED_On( LED_BLUE ) ; 
    }
    break;

    default:
      break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
  Send( );
  /*Wait for next tx slot*/
  TimerStart( &TxTimer );
}

static void OnMagTimerEvent( void )
{
  Cal_Heading( );
  /*Wait for next tx slot*/
  TimerStart( &MagTimer );
}

static void LoraStartTx( TxEventType_t EventType )
{
  if ( EventType == TX_ON_TIMER )
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
		TimerInit( &MagTimer, OnMagTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE );
		TimerSetValue( &MagTimer,  MAG_DUTYCYCLE );
		OnMagTimerEvent( );
    OnTxTimerEvent( );
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct = { 0 };
  
    initStruct.Mode   = GPIO_MODE_IT_RISING;
    initStruct.Pull   = GPIO_PULLUP;
    initStruct.Speed  = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF( "switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG );
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG );
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  LED_Off( LED_RED1 );
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
