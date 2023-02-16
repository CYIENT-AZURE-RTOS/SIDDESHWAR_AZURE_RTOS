/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include"string.h"
#include"main.h"
#include "hts221_reg.h"
#include "ism330dhcx_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HIGH_TEMP 20
#define    BOOT_TIME          10 //ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD Body_temp_thread_id;
TX_THREAD Body_movement_thread_id;
TX_THREAD UART_thread_id;
TX_QUEUE Myqueue;

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;
static int16_t data_raw_humidity;
static int16_t data_raw_temperature;
static float humidity_perc;
static float temperature_degC;
static uint8_t whoamI,rst;
char tx_buffer[100];

///acceleration
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];

stmdev_ctx_t dev_ctx;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Body_temp_thread(ULONG thread_input);
//void Mutexone_entry(ULONG thread_input);
void Body_movement_thread(ULONG thread_input);
void UART_thread(ULONG thread_input);
void App_Delay(uint32_t Delay);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(char *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
void Acceleration_AngularRate(void);//Acce Sensor

//static void platform_init(void);
void temperature_reading(void);//HTS sensor
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;

float linear_interpolation(lin_t *lin, int16_t x)
{
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
}
stmdev_ctx_t dev_ctx;

/* USER CODE END 0 */


/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
	UINT ret = TX_SUCCESS;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

	/* USER CODE BEGIN App_ThreadX_MEM_POOL */
	(void)byte_pool;

	CHAR *pointer1,*pointer2, *pointer3,*Qpointer;

	/* USER CODE END App_ThreadX_MEM_POOL */

	/* USER CODE BEGIN App_ThreadX_Init */


	/* Allocate the stack for ThreadOne.  */


	if (tx_byte_allocate(byte_pool, (VOID **) &pointer1,
			APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		ret = TX_POOL_ERROR;
	}

	/* Create Body temperature thread*/
	if (tx_thread_create(&Body_temp_thread_id, "Body temperature thread", Body_temp_thread, 0,
			pointer1, APP_STACK_SIZE,
			THREAD_ONE_PRIO, THREAD_ONE_PREEMPTION_THRESHOLD,
			TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
	{
		ret = TX_THREAD_ERROR;
	}

	if (tx_byte_allocate(byte_pool, (VOID **) &pointer2,
			APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		ret = TX_POOL_ERROR;
	}

	/* Create Body movement thread */
	if (tx_thread_create(&Body_movement_thread_id, "Body movement thread", Body_movement_thread, 0,
			pointer2, APP_STACK_SIZE,
			THREAD_TWO_PRIO, THREAD_TWO_PREEMPTION_THRESHOLD,
			TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
	{
		ret = TX_THREAD_ERROR;
	}

	/* Create UART print thread */
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer3,
			APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		ret = TX_POOL_ERROR;
	}

	/* Create Thread three.  */
	if (tx_thread_create(&UART_thread_id, "UART print thread", UART_thread, 0,
			pointer3, APP_STACK_SIZE,
			THREAD_THREE_PRIO, THREAD_THREE_PREEMPTION_THRESHOLD,
			TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
	{
		ret = TX_THREAD_ERROR;
	}
	/*Allocate the byte for message queue*/
	if (tx_byte_allocate(byte_pool, (VOID **) &Qpointer,
			1024, TX_NO_WAIT) != TX_SUCCESS)
	{
		ret = TX_POOL_ERROR;
	}

	if(tx_queue_create(&Myqueue, "queue", 16, (VOID *) Qpointer, 1000) != TX_SUCCESS )
	{
		ret = TX_THREAD_ERROR;
	}

	/* USER CODE END App_ThreadX_Init */

	return ret;
}

/**
 * @brief  MX_ThreadX_Init
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init(void)
{
	/* USER CODE BEGIN  Before_Kernel_Start */

	/* USER CODE END  Before_Kernel_Start */

	tx_kernel_enter();

	/* USER CODE BEGIN  Kernel_Start_Error */

	/* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/**
 * @brief  Function implementing the ThreadOne thread.
 * @param  thread_input: Not used
 * @retval None
 */

void Body_temp_thread(ULONG thread_input)
{
	uint8_t status;
	while(1)
	{
		temperature_reading();



		HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Body_Temp_Thread1 \n\r", 27, 200);
		status = tx_queue_send(&Myqueue, tx_buffer, TX_WAIT_FOREVER);
		if(TX_SUCCESS == status)
		{
			HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Queue send Temp\n\r", 24, 200);
			App_Delay(100);
		}
		else
		{
			HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Queue sending fail Temp\n\r", 32, 200);
			App_Delay(100);
		}
		if(temperature_degC>HIGH_TEMP)
		{
			status = tx_queue_send(&Myqueue, tx_buffer, TX_WAIT_FOREVER);
			if(TX_SUCCESS == status)
			{
				HAL_UART_Transmit(&huart1, (const char*)"\r\n Queue send High Temp \n\r", 30, 200);
				App_Delay(100);
			}
			else
			{
				HAL_UART_Transmit(&huart1, (const char*)"\r\n Queue sending fail High Temp \n\r", 38, 200);
				App_Delay(100);
			}
		}
	}
}


void Body_movement_thread(ULONG thread_input)
{
	uint8_t status;
	while(1)
	{
		Acceleration_AngularRate();
		HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Body_Acc_Thread2 \n\r", 27, 200);
				status = tx_queue_send(&Myqueue, tx_buffer, TX_WAIT_FOREVER);
				if(TX_SUCCESS == status)
				{
					HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Queue send Acc\n\r", 24, 200);
					App_Delay(100);
				}
				else
				{
					HAL_UART_Transmit(&huart1, (const uint8_t *)"\r\n Queue sending fail Acc\n\r", 32, 200);
					App_Delay(100);
				}


	}
}

void UART_thread(ULONG thread_input)
{
	uint8_t status;
	uint8_t des[100]= "\0";
	while(1)
	{
		status = _tx_queue_receive(&Myqueue, des, TX_WAIT_FOREVER );

		if(TX_SUCCESS == status)
		{
			sprintf((char *)tx_buffer, "Temperature [degC]:%s\r\n",des );
			 tx_com(tx_buffer, strlen((char const *)tx_buffer));
			 App_Delay(100);
		}
		else
		{
			HAL_UART_Transmit(&huart1,(const uint8_t *) "\r\n Queue received fail \n\r", 40, 200);
			App_Delay(100);
		}
	}
}

/**
 * @brief  Application Delay function.
 * @param  Delay : number of ticks to wait
 * @retval None
 */
void App_Delay(uint32_t Delay)
{
	UINT initial_time = tx_time_get();
	while ((tx_time_get() - initial_time) < Delay);
}

void tx_com(char *tx_buffer, uint16_t len)
{

	HAL_UART_Transmit(&huart1,(const uint8_t *)tx_buffer,len, 100);
}

void temperature_reading(void)
{
	  /* Initialize mems driver interface */
	//  stmdev_ctx_t dev_ctx;
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.handle = &hi2c2;
	  /* Check device ID */
	  whoamI = 0;
	  hts221_device_id_get(&dev_ctx, &whoamI);

	  if ( whoamI != HTS221_ID )
	    while (1); /*manage here device not found */

	  /* Read humidity calibration coefficient */
	  lin_t lin_hum;
	  hts221_hum_adc_point_0_get(&dev_ctx, &lin_hum.x0);
	  hts221_hum_rh_point_0_get(&dev_ctx, &lin_hum.y0);
	  hts221_hum_adc_point_1_get(&dev_ctx, &lin_hum.x1);
	  hts221_hum_rh_point_1_get(&dev_ctx, &lin_hum.y1);
	  /* Read temperature calibration coefficient */
	  lin_t lin_temp;
	  hts221_temp_adc_point_0_get(&dev_ctx, &lin_temp.x0);
	  hts221_temp_deg_point_0_get(&dev_ctx, &lin_temp.y0);
	  hts221_temp_adc_point_1_get(&dev_ctx, &lin_temp.x1);
	  hts221_temp_deg_point_1_get(&dev_ctx, &lin_temp.y1);
	  /* Enable Block Data Update */
	  hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
	  /* Device power on */
	  hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  /* Read samples in polling mode */
        hts221_reg_t reg;
	      hts221_status_get(&dev_ctx, &reg.status_reg);

	      if (reg.status_reg.h_da) {
	        /* Read humidity data */
	        memset(&data_raw_humidity, 0x00, sizeof(int16_t));
	        hts221_humidity_raw_get(&dev_ctx, &data_raw_humidity);
	        humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity);

	        if (humidity_perc < 0) {
	          humidity_perc = 0;
	        }

	        if (humidity_perc > 100) {
	          humidity_perc = 100;
	        }

	        sprintf((char *)tx_buffer, "Humidity [%%]:%3.2f\r\n", humidity_perc);
//	        tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
	      }

	      if (reg.status_reg.t_da) {
	        /* Read temperature data */
	        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	        hts221_temperature_raw_get(&dev_ctx, &data_raw_temperature);
	        temperature_degC = linear_interpolation(&lin_temp,
	                                                data_raw_temperature);
	        sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n",
	                temperature_degC );
//	        tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
	      }


}
void Acceleration_AngularRate(void)
{
		dev_ctx.write_reg = platform_write;
	    dev_ctx.read_reg = platform_read;
	    dev_ctx.handle = &hi2c2;
	    /* Init test platform */
	//    platform_init();
	    /* Wait sensor boot time */
//	    platform_delay(BOOT_TIME);
	    HAL_Delay(BOOT_TIME);

	    /* Check device ID */
	    ism330dhcx_device_id_get(&dev_ctx, &whoamI);

	    if (whoamI != ISM330DHCX_ID)
	      while (1);

	    /* Restore default configuration */
	    ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

	    do {
	      ism330dhcx_reset_get(&dev_ctx, &rst);
	    } while (rst);

	    /* Start device configuration. */
	    ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
	    /* Enable Block Data Update */
	    ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	    /* Set Output Data Rate */
	    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_12Hz5);
	    ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_12Hz5);
	    /* Set full scale */
	    ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
	    ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_2000dps);
	    /* Configure filtering chain(No aux interface)
	     *
	     * Accelerometer - LPF1 + LPF2 path
	     */
	    ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
	    ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	    /* Read samples in polling mode (no int) */
	  uint8_t reg;
		     /* Read output only if new xl value is available */
		     ism330dhcx_xl_flag_data_ready_get(&dev_ctx, &reg);

		     if (reg) {
		       /* Read acceleration field data */
		       memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		       ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		       acceleration_mg[0] =
		         ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[0]);
		       acceleration_mg[1] =
		         ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[1]);
		       acceleration_mg[2] =
		         ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[2]);
		       sprintf((char *)tx_buffer,
		               "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
		               acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//		       tx_com(tx_buffer, strlen((char const *)tx_buffer));
		     }

		     ism330dhcx_gy_flag_data_ready_get(&dev_ctx, &reg);

//		     if (reg) {
//		       /* Read angular rate field data */
//		       memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
//		       ism330dhcx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
//		       angular_rate_mdps[0] =
//		         ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
//		       angular_rate_mdps[1] =
//		         ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
//		       angular_rate_mdps[2] =
//		         ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
//		       sprintf((char *)tx_buffer,
//		               "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
//		               angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
////		       tx_com(tx_buffer, strlen((char const *)tx_buffer));
		  //   }

		    // ism330dhcx_temp_flag_data_ready_get(&dev_ctx, &reg);




}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/* USER CODE END 1 */
