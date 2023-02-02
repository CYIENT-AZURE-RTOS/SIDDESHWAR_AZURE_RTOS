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
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include"string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD ThreadOne;
TX_THREAD ThreadOne2;
TX_THREAD ThreadOne3;
TX_THREAD ThreadOne4;
TX_SEMAPHORE semaphoreone;

extern UART_HandleTypeDef huart1;

/* USER CODE END PV */
uint8_t arr[10] = {0};
uint8_t pin;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void ThreadOne_Entry(ULONG thread_input);
void Mutexone_entry(ULONG thread_input);
void ThreadOne_Entry2(ULONG thread_input);
void ThreadOne_Entry3(ULONG thread_input);
void ThreadOne_Entry4(ULONG thread_input);
void App_Delay(uint32_t Delay);

/* USER CODE END PFP */

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

  CHAR *pointer1,*pointer2, *pointer3,*pointer4;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */


  /* Allocate the stack for ThreadOne.  */


  if (tx_byte_allocate(byte_pool, (VOID **) &pointer1,
                         APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      ret = TX_POOL_ERROR;
    }

    /* Create ThreadOne.  */
    if (tx_thread_create(&ThreadOne, "Thread One", ThreadOne_Entry, 0,
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

        /* Create Threadtwo.  */
        if (tx_thread_create(&ThreadOne2, "Thread two", ThreadOne_Entry2, 0,
                             pointer2, APP_STACK_SIZE,
							 THREAD_TWO_PRIO, THREAD_TWO_PREEMPTION_THRESHOLD,
							 TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
        {
          ret = TX_THREAD_ERROR;
        }

        /* thread three */
        if (tx_byte_allocate(byte_pool, (VOID **) &pointer3,
                                    APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
               {
                 ret = TX_POOL_ERROR;
               }

               /* Create Thread three.  */
               if (tx_thread_create(&ThreadOne3, "Thread three", ThreadOne_Entry3, 0,
                                    pointer3, APP_STACK_SIZE,
       							 THREAD_THREE_PRIO, THREAD_THREE_PREEMPTION_THRESHOLD,
       							 TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
               {
                 ret = TX_THREAD_ERROR;
               }

               /* thread 4*/

               if (tx_byte_allocate(byte_pool, (VOID **) &pointer4,
                                           APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
                      {
                        ret = TX_POOL_ERROR;
                      }

                      /* Create Thread four.  */
                      if (tx_thread_create(&ThreadOne4, "Thread four", ThreadOne_Entry4, 0,
                                           pointer4, APP_STACK_SIZE,
              							 THREAD_FOUR_PRIO, THREAD_FOUR_PREEMPTION_THRESHOLD,
              							 TX_16_ULONG , TX_AUTO_START) != TX_SUCCESS)
                      {
                        ret = TX_THREAD_ERROR;
                      }

        /* MUTEX Creation*/

        if(tx_semaphore_create(&semaphoreone,"semaphore one",2)!= TX_SUCCESS)
        {
        	ret = TX_SEMAPHORE_ERROR;
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

void ThreadOne_Entry(ULONG thread_input)
{
  UINT status = 0;
  while(1)									 //Infinite loop
  {
	  UINT t1_count=0;
	  status=tx_semaphore_get(&semaphoreone,TX_WAIT_FOREVER);
	  if(TX_SUCCESS==status)
	  {
		  if(0==arr[GPIO_PIN_6])
		  	  {
			  	  arr[GPIO_PIN_6]=1;
			  	  pin=GPIO_PIN_6;
		  	  }
		  else if(0==arr[GPIO_PIN_7])
		  	  {
			  	  arr[GPIO_PIN_7]=1;
			  	  pin=GPIO_PIN_7;
		  	  }

		  for(t1_count=0;t1_count<20;t1_count++)
		  	  {
					HAL_GPIO_TogglePin(GPIOH, pin);
					App_Delay(10);                              // delay 100=1000ms, 10=100ms
		  	  }

		  	  	  arr[pin]=0;
			tx_semaphore_put(&semaphoreone);
	  }
  }
}


void ThreadOne_Entry2(ULONG thread_input)
{
  UINT status = 0;
  while(1)									 //Infinite loop
  {
	  UINT t2_count=0;
	  status=tx_semaphore_get(&semaphoreone,TX_WAIT_FOREVER);
	  if(TX_SUCCESS==status)
	  {
		  if(0==arr[GPIO_PIN_6])
		 		 {
		 			 arr[GPIO_PIN_6]=1;
		 			 pin=GPIO_PIN_6;
		 		 }
		 		 else if(0==arr[GPIO_PIN_7])
		 		 {
		 			 arr[GPIO_PIN_7]=1;
		 			 pin=GPIO_PIN_7;
		 		 }

		 		for(t2_count=0;t2_count<9;t2_count++)
		 		{
		 		 HAL_GPIO_TogglePin(GPIOH, pin);
		 		  App_Delay(100);                              // delay 100=1000ms, 10=100ms
		 		}

		 		arr[pin]=0;

	  tx_semaphore_put(&semaphoreone);
	  }
  }
}

void ThreadOne_Entry3(ULONG thread_input)
{
  UINT status = 0;
  while(1)									 //Infinite loop
  {
	  UINT t3_count=0;
	  status=tx_semaphore_get(&semaphoreone,TX_WAIT_FOREVER);
	  if(TX_SUCCESS==status)
	  {
		  if(0==arr[GPIO_PIN_6])
		 		 {
		 			 arr[GPIO_PIN_6]=1;
		 			 pin=GPIO_PIN_6;
		 		 }
		 		 else if(0==arr[GPIO_PIN_7])
		 		 {
		 			 arr[GPIO_PIN_7]=1;
		 			 pin=GPIO_PIN_7;
		 		 }

		 		for(t3_count=0;t3_count<5;t3_count++)
		 		{
		 		 HAL_GPIO_TogglePin(GPIOH, pin);
		 		  App_Delay(250);                              // delay 100=1000ms, 10=100ms
		 		}

		 		arr[pin]=0;

		 	  tx_semaphore_put(&semaphoreone);
	  }
  }
}

void ThreadOne_Entry4(ULONG thread_input)
{
  UINT status = 0;
  while(1)									 //Infinite loop
  {
	  UINT t4_count=0;
	  status=tx_semaphore_get(&semaphoreone,TX_WAIT_FOREVER);
	  if(TX_SUCCESS==status)
	  {
		  	  if(0==arr[GPIO_PIN_6])
		  	  	  {
		  		  	  arr[GPIO_PIN_6]=1;
		  		  	  pin=GPIO_PIN_6;
		  	  	  }
		  	  else if(0==arr[GPIO_PIN_7])
		  	  	  {
		  		  	  arr[GPIO_PIN_7]=1;
		  		  	  pin=GPIO_PIN_7;
		  	  	  }

		  	  for(t4_count=0;t4_count<3;t4_count++)
		  	  	  {
		  		  	  HAL_GPIO_TogglePin(GPIOH, pin);
		  		  	  App_Delay(500);                              // delay 100=1000ms, 10=100ms
		  	  	  }

		  	  	  arr[pin]=0;

		  tx_semaphore_put(&semaphoreone);
	  }
  }
}

/*
void ThreadOne_Entry(ULONG thread_input)
{
 uint8_t t1_count=0;
  while(1)									 //Infinite loop
  {
	 printf("\n\r Thread 1");
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_6);	 //Toggle RED LED
	App_Delay(100);							 // Delay for 100=1000ms
	t1_count++;
	if(10==t1_count)
	{
		t1_count=0;
		tx_thread_sleep(100);
	}
  }
}
*/

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
/* USER CODE END 1 */
