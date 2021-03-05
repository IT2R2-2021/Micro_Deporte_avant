/*
Projet : Voiture Autonome
Carte : Principal
Groupe : IT2R2
Contributeur :
	Matthieu / Conghui
*/

#include "stm32f4xx_hal.h"
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN
#include "biblo_IT2R2.h"

ADC_HandleTypeDef myADC2Handle;
uint32_t Adc_value;
char etat_lumiere;

void Configure_GPIO(void);
void Configure_ADC2_Channel_0(void);
void gestion_phare();

int main(void)
{
	Configure_GPIO(); // initialize PA0 pin 
	Configure_ADC2_Channel_0(); // configure ADC2
	LED_Initialize();
	init_CAN_Transmiter();
	
	while(1)
	{
		gestion_phare();
	}
}

/* Configure and initialize PA0 pin as analog input pin for A/D conversion */
void Configure_GPIO(void)
{
	GPIO_InitTypeDef ADCpin; //create an instance of GPIO_InitTypeDef C struct
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); // enable clock to GPIOA
	ADCpin.Pin = GPIO_PIN_0; // Select pin PA0
	ADCpin.Mode = GPIO_MODE_ANALOG; // Select Analog Mode
	ADCpin.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistor
	HAL_GPIO_Init(GPIOA, &ADCpin); // initialize PA0 as analog input pin
}
void Configure_ADC2_Channel_0(void)
{
	ADC_ChannelConfTypeDef Channel_AN0; // create an instance of ADC_ChannelConfTypeDef

__HAL_RCC_ADC2_CLK_ENABLE(); // enable clock to ADC2 module
	myADC2Handle.Instance = ADC2; // create an instance of ADC2
	myADC2Handle.Init.Resolution = ADC_RESOLUTION_12B; // select 12-bit resolution 
	myADC2Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //select  single conversion as a end of conversion event
	myADC2Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT; // set digital output data right justified
	myADC2Handle.Init.ClockPrescaler =ADC_CLOCK_SYNC_PCLK_DIV8; 
	HAL_ADC_Init(&myADC2Handle); // initialize AD2 with myADC2Handle configuration settings
	
  /*select ADC2 channel */
	Channel_AN0.Channel = ADC_CHANNEL_0; // select analog channel 0
	Channel_AN0.Rank = 1; // set rank to 1
	Channel_AN0.SamplingTime = ADC_SAMPLETIME_15CYCLES; // set sampling time to 15 clock cycles
	HAL_ADC_ConfigChannel(&myADC2Handle, &Channel_AN0); // select channel_0 for ADC2 module. 
}

//fonction de gestion automatique des phares
void gestion_phare()
{
	HAL_ADC_Start(&myADC2Handle); // start A/D conversion
	
	if(HAL_ADC_PollForConversion(&myADC2Handle, 5) == HAL_OK) //check if conversion is completed
	{
		Adc_value  = HAL_ADC_GetValue(&myADC2Handle); // read digital value and save it inside uint32_t variable
	}
	HAL_ADC_Stop(&myADC2Handle); // stop conversion
	
	if (Adc_value<1600)
	{
		LED_On(1);
		etat_lumiere=0xFF;
	}
	if (Adc_value>2000)
	{
		LED_Off(1);
		etat_lumiere=0x00;
	}
	send_CAN_DATA(ID_LIGHT,&etat_lumiere,1);
}