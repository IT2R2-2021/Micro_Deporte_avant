/*
Projet : Voiture Autonome
Fonction : Gestion autonome des phares
Carte : STM32_F4
Contributeur :
	Matthieu / Conghui
*/

#include "stm32f4xx_hal.h"
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN
#include "biblo_IT2R2.h"

//Variable Global Phare
ADC_HandleTypeDef myADC2Handle;
char etat_lumiere[2];

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
	uint32_t  ADC_Value;
	etat_led;
	HAL_ADC_Start(&myADC2Handle); // start A/D conversion
	
	if(HAL_ADC_PollForConversion(&myADC2Handle, 5) == HAL_OK) //check if conversion is completed
	{
		ADC_Value=HAL_ADC_GetValue(&myADC2Handle); // read digital value and save it inside uint32_t variable
	}
	HAL_ADC_Stop(&myADC2Handle); // stop conversion
	
	if (ADC_Value<1100)
	{
		LED_On(1);
		etat_led=0xFF;
	}
	if (ADC_Value>1300)
	{
		LED_Off(1);
		etat_led=0x00;
	}
	
	etat_lumiere[0]=(char)ADC_Value;
	etat_lumiere[1]=etat_led;
	send_CAN_DATA(ID_CAN_LIGHT,&etat_lumiere,2);
}