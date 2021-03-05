/*
Projet : Voiture Autonome
Carte : STM32_F4 Avant
Groupe : IT2R2
Contributeur :
	Conghui / Matthieu
*/

#include "stm32f4xx_hal.h"
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN
#include "biblo_IT2R2.h"
#include "Gestion_phare.h"

int main(void)
{
	//Gestion des phares
	Configure_ADC2_Channel_0(); // configure ADC2
	LED_Initialize();
	
	//Liaison CAN
	init_CAN_Transmiter();
	
	while(1)
	{
		gestion_phare();
	}
}
