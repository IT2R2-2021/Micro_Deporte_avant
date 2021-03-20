/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   STM32F4xx HAL API Template project 
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_CAN.h"                	// ::CMSIS Driver:CAN
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "Driver_SPI.h"                 // ::CMSIS Driver:SPI

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

//Variable Global Phare
ADC_HandleTypeDef myADC2Handle;
char etat_lumiere[2];
char tab_Ruban_led[60*4+4+4];

extern		ARM_DRIVER_CAN        	Driver_CAN2;
extern 		ARM_DRIVER_SPI        	Driver_SPI1;
#define ID_CAN_LIGHT 0x0F8
uint32_t  ADC_Value;

#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void init_SPI();
void init_PIN_PA0_ALS();
void gestion_phare();
void Ruban_LED (void const *argument);                             // thread function
void switch_ruban_led(char etat);

//OS des phares
osThreadId ID_gestion_phare;
osThreadDef(gestion_phare, osPriorityNormal,1,0);		// Phare : Priorité Basse

osThreadId ID_Ruban_LED;                                          // thread id
osThreadDef (Ruban_LED, osPriorityNormal, 1, 0);                   // thread object

//fonction de CB lancee si Event T ou R
void Ruban_LED_callback(uint32_t event)
{
	switch (event) {
		
		
		case ARM_SPI_EVENT_TRANSFER_COMPLETE  : 	 osSignalSet(ID_Ruban_LED, 0x01);
																							break;
		
		default : break;
	}
}

int main(void)
{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();

	//Initialisation pour les phares
	init_PIN_PA0_ALS(); 	
	LED_Initialize();
	init_SPI();
		
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();

  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */
	ID_gestion_phare = osThreadCreate(osThread (gestion_phare),NULL);
	ID_Ruban_LED = osThreadCreate (osThread(Ruban_LED), NULL);

  /* Start thread execution */
  osKernelStart();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
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
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





//Fonction d'initalisation de l'ADC2, sur PA0, pour la lecture du Ambiente Luminescence Sensor (ALS)
void init_PIN_PA0_ALS()
{
	GPIO_InitTypeDef ADCpin; //Création de structure de config PIN_ANALOG
	ADC_ChannelConfTypeDef Channel_AN0; // Création de structure de config ADC
	
	//Intialisation PA0
__HAL_RCC_GPIOA_CLK_ENABLE(); // activation Horloge GPIOA
	ADCpin.Pin = GPIO_PIN_0; // Selection pin PA0
	ADCpin.Mode = GPIO_MODE_ANALOG; // Selection mode analogique
	ADCpin.Pull = GPIO_NOPULL; // Désactivation des résistance de pull-up et pull-down
	
	//Paramétrage ADC2 
__HAL_RCC_ADC2_CLK_ENABLE(); // activation Horloge ADC2
	myADC2Handle.Instance = ADC2; // Selection de ADC2
	myADC2Handle.Init.Resolution = ADC_RESOLUTION_8B; // Selection résolution 12 bit 
	myADC2Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //Selection du mode single pour l'EOC(end of conversion)
	myADC2Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT; // Alignement des data à gauche des octets
	myADC2Handle.Init.ClockPrescaler =ADC_CLOCK_SYNC_PCLK_DIV8; //Syncronisation des horloges
	
	//Paramètrage CHANEL0
	Channel_AN0.Channel = ADC_CHANNEL_0; // Selection analog channel 0
	Channel_AN0.Rank = 1; // Selection du Rang : 1
	Channel_AN0.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Selection du temps d'échentillonage à 15
	
	HAL_GPIO_Init(GPIOA, &ADCpin); // Initialisation PA0 avec les paramètre ci-dessus
	HAL_ADC_Init(&myADC2Handle); // Initialisation ADC2 avec les paramètre ci-dessus
	HAL_ADC_ConfigChannel(&myADC2Handle, &Channel_AN0); // Initialisation Chanel_0 avec les paramètre ci-dessus. 
}

//fonction de gestion automatique des phares
void gestion_phare()
{
	char etat_led;
	uint32_t event;
	while (1)
	{
		LED_On(2);
		HAL_ADC_Start(&myADC2Handle); // start A/D conversion
		
		if(HAL_ADC_PollForConversion(&myADC2Handle, 5) == HAL_OK) //check if conversion is completed
		{
			ADC_Value=HAL_ADC_GetValue(&myADC2Handle); // read digital value and save it inside uint32_t variable
		}
		HAL_ADC_Stop(&myADC2Handle); // stop conversion
		
		if (ADC_Value<140)
		{
			LED_On(1);
			switch_ruban_led(0x01);
			etat_led=0xFF;
		}
		if (ADC_Value>180)
		{
			LED_Off(1);
			switch_ruban_led(0x00);
			etat_led=0x00;
		}		
		osDelay(1000);
	}
}

void init_SPI(void)
	{
	Driver_SPI1.Initialize(Ruban_LED_callback);
	Driver_SPI1.PowerControl(ARM_POWER_FULL);
	Driver_SPI1.Control(ARM_SPI_MODE_MASTER | 
											ARM_SPI_CPOL1_CPHA1 | 
//											ARM_SPI_MSB_LSB | 
											ARM_SPI_SS_MASTER_UNUSED |
											ARM_SPI_DATA_BITS(8),
											1000000);
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

void Ruban_LED (void const *argument) {
	osEvent evt;
  while (1)
	{
		Driver_SPI1.Send(tab_Ruban_led,60*4+4+4);
		evt = osSignalWait(0x01, osWaitForever);	// sommeil fin emission
  }
}
void switch_ruban_led(char etat)
{
	int i, nb_led;
	switch (etat)
	{
		case 0x01:
			
			//début de trame 0x00 00 00 00
			for (i=0;i<4;i++) tab_Ruban_led[i] = 0;
			
			// 20 LED bleues
				for (nb_led = 0; nb_led <20;nb_led++)
				{
					tab_Ruban_led[4+nb_led*4]=0xff; //0xFF
					tab_Ruban_led[5+nb_led*4]=0xff;	//BLUE
					tab_Ruban_led[6+nb_led*4]=0x00;	//GREEN
					tab_Ruban_led[7+nb_led*4]=0x00;	//RED
				}

				// 20 LED Blanc
				for (nb_led = 20; nb_led <40;nb_led++)
				{
					tab_Ruban_led[4+nb_led*4]=0xff; //0xFF
					tab_Ruban_led[5+nb_led*4]=0xff;	//BLUE
					tab_Ruban_led[6+nb_led*4]=0xff;	//GREEN
					tab_Ruban_led[7+nb_led*4]=0xff;	//RED
				}
				// 20 LED rouges
				for (nb_led = 40; nb_led <60;nb_led++)
				{
					tab_Ruban_led[4+nb_led*4]=0xff; //0xFF
					tab_Ruban_led[5+nb_led*4]=0x00;	//BLUE
					tab_Ruban_led[6+nb_led*4]=0x00;	//GREEN
					tab_Ruban_led[7+nb_led*4]=0xff;	//RED
				}
			
				// end
			for (i=243;i<248;i++) tab_Ruban_led[i] = 0;
		break;
				
		case 0x00:
			//début de trame 0x00 00 00 00
			for (i=0;i<4;i++) tab_Ruban_led[i] = 0;
			// 60 LED éteinte
				for (nb_led = 0; nb_led <60;nb_led++)
				{
					tab_Ruban_led[4+nb_led*4]=0xff; //0xFF
					tab_Ruban_led[5+nb_led*4]=0x00;	//BLUE
					tab_Ruban_led[6+nb_led*4]=0x00;	//GREEN
					tab_Ruban_led[7+nb_led*4]=0x00;	//RED
				}
				// end
				for (i=243;i<248;i++) tab_Ruban_led[i] = 0;
			break;

	}
}
