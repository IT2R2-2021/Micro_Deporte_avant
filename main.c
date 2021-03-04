/*
Projet : Voiture Autonome
Carte : STM32_F4 Avant
Groupe : IT2R2
Contributeur :
	Conghui / Matthieu
*/

#define osObjectsPublic                 // define objects in main module
#include "osObjects.h"                  // RTOS object definitions
#include "stm32f4xx_hal.h"							// Startup	
#include "Board_LED.h"                  // ::Board Support:LED
#include "Driver_CAN.h"                	// ::CMSIS Driver:CAN
#include "biblo_IT2R2.h"								// Bibliothèque Réseau	
#include "Gestion_phare.h"							// Gestion des phares


//OS des phares
osThreadId ID_gestion_phare;
osThreadDef(gestion_phare, osPriorityBelowNormal,1,0);		// Phare : Priorité Basse


int main(void)
{
	osKernelInitialize ();                    // initialize CMSIS-RTOS
	
	//Initialisation pour les phares
	init_PIN_PA0_ALS(); 											// configure ADC2 (PIN PA0) branchée sur l'ALS
	LED_Initialize();													// configure les LEDs	
	
	//Liaison CAN
	init_CAN_Transmiter();
	
	ID_gestion_phare = osThreadCreate(osThread (gestion_phare),NULL);
	
	osKernelStart ();                         // start thread execution 
	osDelay(osWaitForever);
}