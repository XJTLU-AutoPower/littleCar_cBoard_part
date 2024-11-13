/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
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
RC_ctrl_t RC_ctrl;
uint8_t sbus_rx_buffer[18];
pid_type_def motor_pid;
const fp32 PID[3]={1.2,0,0.00001};
chassis_motor_t motor_chassis[4];

int mode_forward = 0;
int i = 0;

static int t = 0;
bool flag = true;
float sin1 = 0.0;
float motor_current = 0;

uint32_t motorCnt = 0;
bool motorDir = 0;
long motorSpeed = 0;
long pwmSpeed = 0;


	uint32_t tim8_ch2_set = 1499;
	uint32_t tim8_ch3_set = 1499;
	int tim8_ch3_out = 0;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct{
    uint32_t TX_ID;
    uint32_t RX_ID;
} TYPEDEF_CAN_IDs;

CAN_RxHeaderTypeDef CanRx;//接收消息报文信息
CAN_TxHeaderTypeDef CanTx;//发送消息报文信息
uint8_t RxData[8] = {0,0,0,0,0,0,0,0};  //接收数据区
uint8_t TxData[8] = {0,0,0,0,0,0,0,0};  //发送数据区
uint8_t Flag_Rx = 0;		//接收到数据的标志
uint32_t pTxMailbox = 0;	//发送邮箱盒子

HAL_StatusTypeDef Can_Config(TYPEDEF_CAN_IDs CAN_ID)
{

    CAN_FilterTypeDef sFilterConfig;


    CanTx.DLC = 8;				//数据长度
    CanTx.ExtId = 0x00;
    CanTx.IDE = CAN_ID_STD;		//标准帧模式
    CanTx.RTR = CAN_RTR_DATA;	//数据帧
    CanTx.StdId = CAN_ID.TX_ID;			//使用扩展帧时需置零
    CanTx.TransmitGlobalTime = DISABLE;//时间戳

    CanRx.DLC = 8;				//数据长度
    CanRx.ExtId = 0x00;
    CanRx.IDE = CAN_ID_STD;		//标准帧模式
    CanRx.RTR = CAN_RTR_DATA;	//数据帧
    CanRx.StdId = CAN_ID.RX_ID;			//使用扩展帧时需置零
    //CanRx.TransmitGlobalTime = DISABLE;//时间戳

    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;	//筛选器使能
    sFilterConfig.FilterBank = 0;						//筛选器0
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//指定将分配给过滤器的FIFO(0或1U)

//    sFilterConfig.FilterIdHigh = (((CanRx.ExtId<<3) | CAN_ID_EXT | CAN_RTR_DATA)&0xFFFF0000)>>16;	//ID的高16位
//    sFilterConfig.FilterIdLow = ((CanRx.ExtId<<3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;	//ID的低16位，只接收扩展帧模式、数据帧
//    sFilterConfig.FilterMaskIdHigh = 0xFFFF;//FilterMask高低字节数据中位为1时代表必须与ID该位一致，0xFFFFFFFF代表接收筛选必须与ID一致才通过
//    sFilterConfig.FilterMaskIdLow = 0xFFFF;

    sFilterConfig.FilterIdHigh   =(((uint32_t)CanRx.StdId<<21)&0xffff0000)>>16;
    sFilterConfig.FilterIdLow  = (((uint32_t)CanRx.StdId<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
    sFilterConfig.FilterMaskIdHigh  = 0xFFFF;
    sFilterConfig.FilterMaskIdLow   = 0xFFFF;

//不设过滤器
//    sFilterConfig.FilterIdHigh   = 0x0;
//    sFilterConfig.FilterIdLow    = 0x0;
//    sFilterConfig.FilterMaskIdHigh  = 0x0;
//    sFilterConfig.FilterMaskIdLow   = 0x0;

    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//32位
    sFilterConfig.SlaveStartFilterBank = 0;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if(HAL_CAN_Start(&hcan1) != HAL_OK) //启动CAN
    {
        return HAL_ERROR;
    }

    if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//激活CAN_IT_RX_FIFO0_MSG_PENDING中断
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//报文接收中断
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CanRx, RxData) == HAL_OK)
    {
        Flag_Rx = 1;
    }

}


void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)//当接收的Fifo0满了的时候产生该中断
{

}



int map(float val, float I_Min, float I_Max, float O_Min, float O_Max) //映射函数
{
    return (int)((((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min));
}


TYPEDEF_CAN_IDs Switch_Canid(void)   //开关检测
{
    TYPEDEF_CAN_IDs keys_canid;
    keys_canid.TX_ID = NULL;
    keys_canid.RX_ID = NULL;

        keys_canid.TX_ID = 0x111;
        keys_canid.RX_ID = 0x112;

    return keys_canid;
}

void countMotorSpeed()
{
		// 编码器轮询
		motorDir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
		motorCnt = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));//获取定时器的值
		__HAL_TIM_SET_COUNTER(&htim1, 0);	// 置零
		if (motorDir == 0) motorSpeed = (long)(motorCnt);
	  else if (motorCnt == 0) motorSpeed = 0;
		else motorSpeed = -(long)(65535 - motorCnt);
		
		pwmSpeed = map(motorSpeed, -88, 88, -750, 750);
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim4)
    {
      //500ms trigger
			t ++;
			countMotorSpeed();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	// 初始化pid
	PID_init(&motor_pid,PID_POSITION,PID,755,755);
	//motor_pid.derivative_output_filter_coefficient = 0.99995;
	//motor_pid.proportion_output_filter_coefficient = 0.99004;
	motor_pid.out=0;
	
	// 定时器1 编码器模式ch1A ch2B
	//HAL_TIM_Base_Start_IT(&htim1);	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	
	// 定时器4中断
	HAL_TIM_Base_Start_IT(&htim4);
	
	can_filter_init();
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
	
	// 蜂鸣器
		//HAL_TIM_Base_Start_IT(&htim4);
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	// 三色led灯
		//HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

	// 舵机ch7 ch8	0 ~ 1999 500 ~ 2500
		//HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	// ch7
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	// ch8
	
	
	Can_Config(Switch_Canid()); //引入canid并初始化结构体
	
	float psc_tim8_max = 19999.0;
	

  uint16_t interval_time = 8; //调节连续发包频率的间隔时间
	
	// 双通道电机舵机回中
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	HAL_Delay(100);
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1499);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1499);
	HAL_Delay(3000);
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1499);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1499);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CanRx,RxData);   //试图收包


      if(Flag_Rx)
      {
				// 亮灯
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 50000);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 50000);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 50000);
				
				tim8_ch2_set = (int)RxData[0]*(16*16) + (int)RxData[1];
				// tim8_ch2_set = 0x13*16*16 +0x88;
				tim8_ch3_set = (int)RxData[2]*(16*16) + (int)RxData[3];



          for(int i = 0; i < sizeof TxData; i++) {
              TxData[i] = RxData[i];
          }

                  Flag_Rx = 0;

      }
			
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, tim8_ch2_set);
			
			tim8_ch3_out = PID_calc(&motor_pid, pwmSpeed + 1499, tim8_ch3_set);
      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, tim8_ch3_out + 1500);
			
			//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, tim8_ch3_set);
			

      HAL_CAN_AddTxMessage(&hcan1, &CanTx, TxData, &pTxMailbox); // 发包

      HAL_Delay(interval_time); // 调节发包频率
			
			// 暗灯
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 500);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 500);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 500);
		

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	  RC_ctrl.rc.ch[0] = (sbus_rx_buffer[0] | (sbus_rx_buffer[1] << 8)) & 0x07ff;        //!< Channel 0
    RC_ctrl.rc.ch[1] = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
    RC_ctrl.rc.ch[2] = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) |          //!< Channel 2
                         (sbus_rx_buffer[4] << 10)) &0x07ff;
    RC_ctrl.rc.ch[3] = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
    RC_ctrl.rc.s[0] = ((sbus_rx_buffer[5] >> 4) & 0x0003);                  //!< Switch left
    RC_ctrl.rc.s[1] = ((sbus_rx_buffer[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    RC_ctrl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis
    RC_ctrl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis
    RC_ctrl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis
    RC_ctrl.mouse.press_l = sbus_rx_buffer[12];                                  //!< Mouse Left Is Press ?
    RC_ctrl.mouse.press_r = sbus_rx_buffer[13];                                  //!< Mouse Right Is Press ?
    RC_ctrl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);                    //!< KeyBoard value

	
		RC_ctrl.rc.ch[0] -=1024;
		RC_ctrl.rc.ch[1] -=1024;
		RC_ctrl.rc.ch[2] -=1024;
		RC_ctrl.rc.ch[3] -=1024;
	
		mode_forward=RC_ctrl.rc.ch[0];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
