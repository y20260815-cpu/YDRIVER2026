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
#include "extern.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
#define MAX_RX_BUFFER_SIZE 64
uint16_t adc_buf[ADC_CHANNEL_COUNT];

uint8_t one_millisec_flag=0;
uint8_t ten_millisec_flag=0;
uint8_t hnd_millisec_flag=0;

uint8_t one_sec_flag=0;
int8_t boot_time_out=20;
uint8_t rx_data;
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t rx_cnt=0;
uint8_t rx_flag=0;
uint16_t rpm=0;
//uint16_t rpm_duration_cnt=0;
uint8_t SYSTEM_setup_data_ok=0;
SYSTEM_CONF sysConf;
PID_CONFIG pidCONF;

//// DMA �????? 버퍼

//uint8_t rxBuffer[RX_BUFFER_SIZE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len)
{
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
	HAL_StatusTypeDef tx1 = HAL_UART_Transmit(&huart1, ptr, len, len);
	HAL_StatusTypeDef tx3 = HAL_UART_Transmit(&huart3, ptr, len, len);
	if(tx1 == HAL_OK || tx3 == HAL_OK) return len;
	else return 0;
}

void delay_us(uint32_t us)
{
    uint32_t count = us * 9;  // 72MHz 기�?
    while (count--) {
        __asm__ volatile ("nop");
    }
}

static const char* color_table[] = {
    "\033[0m",  "\033[31m", "\033[32m", "\033[33m",
    "\033[34m", "\033[35m", "\033[36m", "\033[37m"
};

void cprintf(ccolor_t color, const char *fmt, ...)
{
    va_list args;
    printf("%s", color_table[(int)color]);

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\033[0m");
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void Start_UART_DMA_Receive(void)
//{
//    // DMA ?��?�� 모드�????? UART ?��?�� ?��?��
////    HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);
//    // Idle ?��?��?��?�� ?��?��?�� (Idle ?��?��?��?�� 발생 ?��, DMA �????? 버퍼 ?�� ?�� ?��?��?�� 처리�????? ?��?��)
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//uint8_t once_for_vBEMF=10;
	static int emergency_cnt=5;
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
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("===MAIN START===..\r\n");

  pCAN = new tja1050();
  pDataClass=new data_class();
  pPWM=new PWM16();
  pFND595=new FND595();
  pFlash_mem=new stm32flash();
//  pZIGBEE =new zigbee();
//  pWIFI =new wifiParse();
  pEPS =new eps();

  /* SYSTEM_CONF is not used in this system. */
  pCAN->init();
  
  //__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
  //HAL_UART_Receive_IT(&huart2, rxBuffer, RX_BUFFER_SIZE);
  //Start_UART_DMA_Receive();
  HAL_TIM_Base_Start_IT(&htim4);  // TIM4 ?��?��?��?�� ?��?��
  __HAL_IWDG_START(&hiwdg);//40KHZ LCLK 128분주-3125 10(312.5*10)//4000
  HAL_ADCEx_Calibration_Start(&hadc1);
  printf("\r\n******** Start *******\r\n");
  HAL_Delay(500);
  pDataClass->power_off();

  uint8_t jh_switch=1;
  uint16_t throttle=0;
  HAL_GPIO_WritePin(pRY7_GPIO_Port, pRY7_Pin, GPIO_PIN_SET);//LED_ON
//  while(1)
//  {
//	  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
//	  //printf("(1)wait CAN...jenhujin[%d] boot_time_out[%d]\r\n",	pDataClass->vcu_sdu.toggle.jenhujin, boot_time_out);
//	  jh_switch=pDataClass->vcu_sdu.toggle.jenhujin;
//	  throttle=pDataClass->vcu_sdu.orgspeed;
//	  //printf("(2)wait CAN...jh_switch[%d] throttle[%d]\r\n",	jh_switch, throttle);
//	  if(boot_time_out>0)boot_time_out--;
//	  if(boot_time_out==0){
//		  printf("(3)waiting jh_switch[%d] trottle[%d]\r\n",jh_switch, throttle);
//	  	  if(throttle<50 && jh_switch==0){
//	  		  HAL_Delay(200);
//	  		  break;
//	  	  }
//	  	  //if(pDataClass->system_start_flag)break;
//	  }
//	  HAL_Delay(10);
//  }

  printf("(4) pDataClass->power_on..\r\n");
  pDataClass->power_on();
  pDataClass->HOLD_Emergency=0;
  //pFND595->PrintDigit(5);
  //pFND595->TestSegments();
  pFND595->PrintDigit(0);
  //pFND595->TestOutputBits();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //__HAL_IWDG_START(&hiwdg);//40KHZ LCLK 128분주-3125 10(312.5*10)//4000
  while (1)
  {
	 // printf(".####");
	 __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
#if 1
	 //처음 켜졌을때 HOLD_Emergency값이 들어오는 경우 있음
	  if(pDataClass->HOLD_Emergency){
		  if(emergency_cnt)emergency_cnt--;
		  if(emergency_cnt<=0){
			  printf("===HOLD_Emergency===\r\n");
			  HAL_GPIO_WritePin(pRY7_GPIO_Port, pRY7_Pin, GPIO_PIN_RESET);//LED_ON
			  //pDataClass->relayAllOFF();
			  if(pDataClass->PWR_ON_Flg==1) pDataClass->power_off();

		  }
	  }
	  // CAN Setup Data 전송 플래그 체크
	  if(pCAN->canFlag.needTxsetupData) {
		  pCAN->canFlag.needTxsetupData = 0;
		  pCAN->CAN_Request_Setup_Data();  // 메인 루프에서는 HAL_Delay 사용 가능
	  }
	  if(pCAN->canFlag.needRxConfigData) {
		  pCAN->canFlag.needRxConfigData = 0;
		  pCAN->canSetConfig();
	  }


//=========================================
	  if(ten_millisec_flag){
		  ten_millisec_flag=0;
		  //printf("10\r\n");
		  pDataClass->ten_millisec_routine();
	  }
	  else if(one_millisec_flag){
		  one_millisec_flag=0;
		  //printf("x");
		  pDataClass->one_millisec_routine();
	  }

//==========================================
	  if(hnd_millisec_flag){
		  hnd_millisec_flag=0;
		  pDataClass->hnd_millisec_routine();
	  }
#endif
	  if(one_sec_flag){
		  one_sec_flag=0;
		  pDataClass->onesec_routine();
//		  if(rpm_duration_cnt>5){
//			  rpm_duration_cnt=0;
//			  pDataClass->gMAIN.rpm=rpm;
//			  rpm=0;
//		  }
	  }
	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	static int tick=0;
	HAL_IncTick();
	if(tick%100==0){
		hnd_millisec_flag=1;
	}
	if(tick++>1000){
		tick=0;
		one_sec_flag=1;
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t one_milisec_tick=0;
  if (htim->Instance == TIM4)
  {
	  one_milisec_tick++;
	  one_millisec_flag=1;
	  if(one_milisec_tick>9){
		  one_milisec_tick=0;
		  ten_millisec_flag=1;
	  }
  }

}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    if (hadc->Instance == ADC1) {
//    	memcpy(&pDataClass->ladcValue, adc_buf,9);
//    }
//}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {

		uint8_t uart_data;
		uint32_t tmp_flag = 0, tmp_it_source = 0;
		tmp_flag =  __HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE);
		tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE);

//		if ((tmp_it_source != RESET) && (tmp_flag != RESET)){
//
//			uart_data=huart2.Instance->DR;
//			{
//				if(uart_data==0xBF && rx_flag==0){
//					rx_flag=1;
//					rx_cnt=0;
//					memset(&rx_buffer, 0, sizeof(rx_buffer));
//				}
//				if(rx_flag)rx_buffer[rx_cnt++]=uart_data;
//				if(rx_cnt>16){
//					rx_flag=0;
//					rx_cnt=0;
//					if(rx_buffer[0]==0xBF && rx_buffer[15]==0x40){
//						pZIGBEE->zigbee_received(rx_buffer);
//					}
//				}
//			}
//
//		}
		if ((tmp_it_source != RESET) && (tmp_flag != RESET)){

			uart_data=huart2.Instance->DR;
			printf("HAL_UART_RxCpltCallback [%x]\r\n", uart_data);
			{
				if(uart_data==0xFB && rx_flag==0){
					rx_flag=1;
					rx_cnt=0;

					memset(&rx_buffer, 0, sizeof(rx_buffer));
				}
				if(rx_flag)rx_buffer[rx_cnt++]=uart_data;
				if(rx_cnt>16){
					rx_flag=0;
					rx_cnt=0;
					if(rx_buffer[0]==0xBF && rx_buffer[15]==0x40){
						pZIGBEE->zigbee_received(rx_buffer);
					}
				}
			}

		}

		__HAL_UART_CLEAR_PEFLAG(&huart2);

	}
	 __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
}
//#else

int find_packet_start(uint8_t *buffer, int length) {
    for (int i = 0; i < length; i++) {
        if (buffer[i] == 0xFB) { // stx �?????? ?��?��
            return i;
        }
    }
    return -1; // stx�?????? ?��?�� 경우
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    PUB_UPDATE pubUpdate;
    //printf("HAL_UART_RxCpltCallback\r\n");
    uint8_t pData[64]={0,};
    if(huart->Instance == USART2)  // USART2?�� ???�� 처리
    {

		int start_index = find_packet_start(rxBuffer, RX_BUFFER_SIZE);
		if (start_index < 0 || (start_index + RX_BUFFER_SIZE) > MAX_RX_BUFFER_SIZE) {
			// ?��?�� ?��?��?��?�� 찾�? 못했거나 버퍼?�� �??????족한 경우 ?��?�� 처리 ?�� ?��?�� ?��?�� �??????�??????
			HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
			return;
		}
		//printf("HAL_UART_RxCpltCallback\r\n");
		// ?��?�� 버퍼?�� ?��?��?�� ?��?��?���??????? 구조체로 복사
		//memcpy(&pubUpdate, rxBuffer, RX_BUFFER_SIZE);
		memcpy(&pubUpdate, &rxBuffer[start_index], sizeof(PUB_UPDATE));
		memcpy(pData, &rxBuffer[start_index], sizeof(PUB_UPDATE));
		printf("[%d] pData[%x][%x][%x][%x]-[%x][%x]\r\n", RX_BUFFER_SIZE, pData[0], pData[1], pData[2], pData[3], pData[12], pData[13]);
    // ?��?��/종료 바이?�� �????????��
		if (pubUpdate.stx != 0xFB || pubUpdate.etx != '@')
		{
			// ?��?�� 경계 ?��?�� 처리 (?��?�� ?�� 로그 출력 ?��?�� ?��?�� 처리 코드 추�?)
			HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
			return;
		}

    // 체크?�� 계산
    // ?��기서?�� size?? data ?��?��?�� 모든 바이?�� ?��?�� (stx, sum, etx ?��?��)
    uint8_t calcSum = 0;
    uint8_t *p = (uint8_t*)&pubUpdate;

    printf("HAL_UART_RxCpltCallback\r\n");
    for (int i = 1; i < (RX_BUFFER_SIZE - 2); i++)  // index 1�????????�� (sum, etx ?��까�?)
    {
        calcSum += p[i];
    }

    // 계산?�� 체크?���??????? ?��?��?�� 체크?�� 비교
    if (calcSum != pubUpdate.sum)
    {
        // 체크?�� ?��?�� 처리
        HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
        return;
    }

    // ?��?��?�� ?��?�� ?���??????? ?? ?��?�� pubUpdate.data?�� �??????? ?��목을 ?��?��?���??????? ?��?��?��.
    uint16_t adc0 = pubUpdate.data.adc0;
    uint16_t adc1 = pubUpdate.data.adc1;
    uint16_t adc2 = pubUpdate.data.adc2;
    uint16_t adc3 = pubUpdate.data.adc3;

    // ?��?��: ?��?�� 비트?�� ?��?��
    uint8_t link  = pubUpdate.data.port_bit.link;
    uint8_t pFW   = pubUpdate.data.port_bit.pFW;
    // ?��?��?�� ?���??????? 비트?��?�� 마찬�???????�???????�??????? ?���??????? �????????��

    // ?��?�� ?��?��?�� ?��?��?���??????? ?��?��?�� 처리 코드�??????? 추�?

    // ?��?�� ?��?��?�� ?��?��?�� ?��?�� UART ?��?��?��?�� ?��?��?��
    HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
  }
}
#endif
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
  NVIC_SystemReset();//for rebooting
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
