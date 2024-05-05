/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "mainpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Size_of_buffer  2500			//Fifo_buffer-iin hemjee
#define  M 33						//A,B duguinii hoorondokh distance
#define  L 45							//A,C duguinii hoorondokh distance
#define  alpha 63
#define  beta 333
#define  gamma 90
#define  delta  27
#define  per_step 5.3*M_PI/2000		//Interrupt hoorondokh ue-iin hemjee

// Gyroscope define
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define BUFFER_SIZE 8

//Can define
//#define CAN1_DEVICE_NUM     4
//#define FIRST_GROUP_ID      0x190
//#define CAN_DATA_SIZE       8
//#define CAN1_RX_ID_START    0x201
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int32_t valueA;
//EncoderB-iin utga
int32_t valueB;
//EncoderC-iin utga
int32_t valueC;
//niit orj irsen encoder-iin interruptiin too
uint32_t value_s=0;

uint32_t t_temp;

int16_t theta_angle;
uint32_t interrupt_counter;
uint8_t rxData[8];
uint8_t flag=0;

double Speed_EncoderA;		//A_encoder-iin hurd-iig hadgalakh huwisagch
double Speed_EncoderB;		//B_encoder-iin hurd-iig hadgalakh huwisagch
double Speed_EncoderC;		//C_encoder-iin hurd-iig hadgalakh huwisagch

double ha_perp;
double hb_perp;
double hc_perp;
double vA_perp;


double RX[2][1];
double RY[2][1];
double at[2][1];
double at_perp[2][1];
double rA[2][1]={0};
double velocity_A[2][1];
double velocity_A_perp[2][1];
double A_velocity[2][1];
double distanceA = 0.0;
double distanceB = 0.0;
double distanceC = 0.0;
/*
 *
 */
double omega ;
double theta ;

//Gyroscope
uint16_t Accel_X_Raw;
uint16_t Accel_Y_Raw;
uint16_t Accel_Z_Raw;
float Ax;
float Ay;
float Az;

int16_t buffer[5];
uint8_t msg1;
uint8_t msg2;
uint8_t msg3;
uint8_t msg4;
uint8_t msg5;

uint8_t angle_X;
uint8_t angle_Y;
double radian_Alpha;
double radian_Beta;
double radian_Gamma;
double radian_Delta;
double AB[2][1];
double AC[2][1];
double a_encoder[2][1];
double a_perp[2][1];
double b_encoder[2][1];
double c_encoder[2][1];
double g[2][1];
double g_perp[2][1];
double b_c_encoder[2][2];
double theta_0=0;
uint32_t old_couter=0;
//CAN_FilterTypeDef sFilterConfig;
//CAN_RxHeaderTypeDef rxHeader;
//CAN_TxHeaderTypeDef txHeader;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, uint8_t msg0, uint8_t msg1, uint16_t msg2,uint16_t msg3,uint16_t msg4){
//	CAN_TxHeaderTypeDef tx_header;
//	uint8_t             buffer[8];
//	uint32_t            pTxMailbox;
//
//	tx_header.StdId = id;
//	tx_header.IDE   = CAN_ID_STD;
//	tx_header.RTR   = CAN_RTR_DATA;
//	tx_header.DLC   = 8;
//	tx_header.TransmitGlobalTime = DISABLE;
//	buffer[0]=msg0;
//	buffer[1]=msg1;
//	buffer[2] = msg2>>8;
//	buffer[3] = msg2;
//	buffer[4] = msg3>>8;
//	buffer[5] = msg3;
//	buffer[6] = msg4>>8;
//	buffer[7] = msg4;
//
//	if (HAL_CAN_AddTxMessage(hcan, &tx_header, buffer, &pTxMailbox) == HAL_OK){
//		HAL_CAN_IsTxMessagePending(hcan, pTxMailbox);
//	}
//}
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//
//	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
//	if(rxHeader.StdId == 0x200){
//		flag++;
//	}
//
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//	if(htim==&htim12)
//	{
////		can_transmit(&hcan1, 0x201, buffer[0], buffer[1], buffer[2],buffer[3],buffer[4]);
//		loop();
//	}
	if(htim==&htim5)
	{
		uint16_t new_countA = TIM3->CNT;
		uint16_t new_countB = TIM4->CNT;
		uint16_t new_countC = TIM8->CNT;
		int16_t diffA = new_countA - valueA;
		int16_t diffB = new_countB - valueB;
		int16_t diffC = new_countC - valueC;

		if (diffA > 32767) {
			diffA -= 65536; // Handle overflow
		} else if (diffA < -32767) {
			diffA += 65536; // Handle underflow
		}

		if (diffB > 32767) {
			diffB -= 65536; // Handle overflow
		} else if (diffB < -32767) {
			diffB += 65536; // Handle underflow
		}

		if (diffC > 32767) {
			diffC -= 65536; // Handle overflow
		} else if (diffC < -32767) {
			diffC += 65536; // Handle underflow
		}

		// Update total distances traveled
		Speed_EncoderA=diffA*per_step;
		Speed_EncoderB=diffB*per_step;
		Speed_EncoderC=diffC*per_step;
		distanceA += Speed_EncoderA;
		distanceB += Speed_EncoderB;
		distanceC += Speed_EncoderC;
		// Update current counts
		valueA = new_countA;
		valueB = new_countB;
		valueC = new_countC;
		interrupt_counter++;
	}
}
void MPU6050_Init (void)
{
	uint8_t check, Data;
	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, WHO_AM_I_REG,1,&check,1,1000);

	if( check == 104)
	{
		Data=0;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		Data=0x07;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}
void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_Raw=(int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_Raw=(int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_Raw=(int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	//		can_transmit(&hcan1, 0x191, Rec_Data[0], Rec_Data[1]);
	Ax=Accel_X_Raw/4096.0;
	Ay=Accel_Y_Raw/4096.0;
	Az=Accel_Z_Raw/4096.0;
}
float Accel_X_Angle(float Ax, float Ay, float Az) {
	float angle_deg = (180/3.141592)*(atan(Az/Ax));
	return angle_deg;
}
float Accel_Y_Angle(float Ax, float Ay, float Az) {
	float angle_deg = (180/3.141592)*(atan(Ay/Az));
	return angle_deg;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);

	/*initial buyu neg robot geometry cm */
	radian_Alpha=(M_PI/180)*alpha;		//alpha untsugiig degree to radian bolgoj hurwuulj  baigaa uildel
	radian_Beta=(M_PI/180)*beta;		//beta untsugiig degree to radian bolgoj hurwuulj  baigaa uildel
	radian_Gamma=(M_PI/180)*gamma;		//gamma untsugiig degree to radian bolgoj hurwuulj  baigaa uildel;
	radian_Delta=(M_PI/180)*delta;		//delta untsugiig degree to radian bolgoj hurwuulj  baigaa uildel;
	// AB vector
	AB[0][0]=L*cos(radian_Delta);
	AB[1][0]=L*sin(radian_Delta);

	// AC vector
	AC[0][0]=M*1;
	AC[1][0]=M*0;

	//a_encoder Vector
	a_encoder[0][0]=cos(radian_Delta + radian_Alpha),
	a_encoder[1][0]=sin(radian_Delta + radian_Alpha);
	//a_perp Vector
	a_perp[0][0]=-1*a_encoder[1][0];
	a_perp[1][0]=1*a_encoder[0][0];
	//b vector
	b_encoder[0][0]=cos(radian_Delta+radian_Beta);
	b_encoder[1][0]=sin(radian_Delta+radian_Beta);

	//c_vector
	c_encoder[0][0]=cos(radian_Gamma);
	c_encoder[1][0]=sin(radian_Gamma);
	/*
	 * g=inv([b,c])*a;
	 * [b,c]=[b_encoder,c_encoder]  [4][4]
	 * inv([b,c])
	 * a=[2][1]
	 */
	b_c_encoder[0][0]=b_encoder[0][0];
	b_c_encoder[1][0]=b_encoder[1][0];
	b_c_encoder[0][1]=c_encoder[0][0];
	b_c_encoder[1][1]=c_encoder[1][0];
	/*
	 * Determinant
	 */
	double determinant;
	determinant=b_c_encoder[0][0] * b_c_encoder[1][1] - b_c_encoder[0][1] * b_c_encoder[1][0];
	/*
	 * Matrix inverse
	 */
	double inv_det;
	inv_det=1/determinant;
	double temp[2][2];
	temp[0][0] =  b_c_encoder[1][1] * inv_det;
	temp[0][1] = -b_c_encoder[0][1] * inv_det;
	temp[1][0] = -b_c_encoder[1][0] * inv_det;
	temp[1][1] =  b_c_encoder[0][0] * inv_det;
	/*
	 *
	 */
	g[0][0]=temp[0][0]*a_encoder[0][0]+temp[0][1]*a_encoder[1][0];
	g[1][0]=temp[1][0]*a_encoder[0][0]+temp[1][1]*a_encoder[1][0];
	/*
	 * g_perp
	 */
	g_perp[0][0]=temp[0][0]*a_perp[0][0]+temp[0][1]*a_perp[1][0];
	g_perp[1][0]=temp[1][0]*a_perp[0][0]+temp[1][1]*a_perp[1][0];
	double Bb=AB[0][0]*b_encoder[1][0]-AB[1][0]*b_encoder[0][0];
	double Cc=AC[0][0]*c_encoder[1][0]-AC[1][0]*c_encoder[0][0];
	ha_perp=(g_perp[0][0]*Bb+g_perp[1][0]*Cc)/(g[0][0]*Bb+g[1][0]*Cc);
	hb_perp=(g_perp[0][0]*g[1][0]-g_perp[1][0]*g[1][1])*Cc/(g[0][0]*Bb+g[1][0]*Cc);
	hc_perp=-(g_perp[0][0]*g[1][0]-g_perp[1][0]*g[0][0])*Bb/(g[0][0]*Bb+g[1][0]*Cc);

	HAL_TIM_Base_Start_IT(&htim12);
	MPU6050_Init();
	setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(interrupt_counter>old_couter)
		{
			//omega=(g[0][0]*Speed_EncoderB+g[1][0]*Speed_EncoderC-Speed_EncoderA)/(g[1][1]*Bb+g[1][0]*Cc);
			theta=theta_0+((g[0][0]*distanceB+g[1][0]*distanceC)-distanceA)/(g[0][0]*Bb+g[1][0]*Cc);
			vA_perp=Speed_EncoderA*ha_perp+Speed_EncoderB*hb_perp+Speed_EncoderC*hc_perp;
			/*
			 * Rotation matrix
			 */
			RX[0][0]=cos(theta);
			RX[1][0]=sin(theta);
			RY[0][0]=-sin(theta);
			RY[1][0]=cos(theta);

			at[0][0]=a_encoder[0][0]*RX[0][0]+a_encoder[1][0]*RY[0][0];
			at[1][0]=a_encoder[0][0]*RX[1][0]+a_encoder[1][0]*RY[1][0];

			//			at_perp[0][0]=a_perp[0][0]*RX[0][0]+a_perp[1][0]*RX[0][1];
			//			at_perp[1][0]=a_perp[0][0]*RY[0][0]+a_perp[1][0]*RY[0][1];

			at_perp[0][0]=a_perp[0][0]*RX[0][0]+a_perp[1][0]*RY[0][1];
			at_perp[1][0]=a_perp[0][0]*RX[1][0]+a_perp[1][0]*RY[1][0];

			velocity_A[0][0]=Speed_EncoderA*at[0][0];
			velocity_A[1][0]=Speed_EncoderA*at[1][0];

			velocity_A_perp[0][0]=vA_perp*at_perp[0][0];
			velocity_A_perp[0][1]=vA_perp*at_perp[1][0];

			A_velocity[0][0]=velocity_A[0][0]+velocity_A_perp[0][0];
			A_velocity[1][0]=velocity_A[1][0]+velocity_A_perp[1][0];
			rA[0][0]=rA[0][0]+(A_velocity[0][0]);
			rA[1][0]=rA[1][0]+(A_velocity[1][0]);
			old_couter=interrupt_counter;

			MPU6050_Read_Accel();
			angle_X = Accel_X_Angle(Ax, Ay, Az);
			angle_Y = Accel_Y_Angle(Ax, Ay, Az);
			theta_angle=theta*180/M_PI;
			int16_t rA_x_int16_t = (int16_t)rA[0][0];
			int16_t rA_y_int16_t = (int16_t)rA[1][0];
			buffer[0]= angle_X;
			buffer[1]= angle_Y;
			buffer[2]= theta_angle;
			buffer[3]= rA_x_int16_t;
			buffer[4]= rA_y_int16_t;
			loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 5;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 899999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 90-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 200-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
