/*
Projet : Voiture Autonome
Groupe : IT2R2
Fichier pour ressenser toutes les fonctions utiles que l'on pourrait tous avoir besoin d'utiliser sur plusieurs micro
*/

#include "Driver_CAN.h"                 // ::CMSIS Driver:CAN
#include "Driver_UART.h"								// ::CMSIS Driver:UART
#include "Driver_I2C.h"									// ::CMSIS Driver:I2C

extern		ARM_DRIVER_CAN        	Driver_CAN1;
extern		ARM_DRIVER_CAN        	Driver_CAN2;
extern		ARM_DRIVER_I2C 					Driver_I2C1;
extern		ARM_DRIVER_USART				Driver_USART1;

//Table des IDs CAN

		#define ID_CAN_LIGHT 0x0F8
		
/*
Fonction : Permet d'initialis� le CAN 2 comme CAN de transmition
Cr�ateur : Matthieu
*/
void init_CAN_Transmiter()
{
	Driver_CAN2.Initialize(NULL,NULL);
	Driver_CAN2.PowerControl(ARM_POWER_FULL);
	Driver_CAN2.SetMode(	ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN2.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
													125000,
													ARM_CAN_BIT_PROP_SEG(5U)|
													ARM_CAN_BIT_PHASE_SEG1(1U)|
													ARM_CAN_BIT_PHASE_SEG2(1U)|
													ARM_CAN_BIT_SJW(1U));	
	Driver_CAN2.ObjectConfigure(1,ARM_CAN_OBJ_TX);
	Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL);
}

/*
Fonction : Permet d'initialis� le CAN 1 comme CAN de R�c�ption

Variable d'entr�e :
		- *adresse_Filtre : 		Pointeur renvoyant sur un tableau regroupant tout les IDs CAN qui doivent �tre lue
		- lengt :							Nombre d'ID lu par le r�cepteur.

Cr�ateur : Matthieu
*/
void init_CAN_Receiver(char *adresse_Filtre, char lengt)
{
	char i;
	Driver_CAN1.Initialize(NULL,myCAN1_callback);
	Driver_CAN1.PowerControl(ARM_POWER_FULL);
	Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN1.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
													125000,
													ARM_CAN_BIT_PROP_SEG(5U)   |         // Set propagation segment to 5 time quanta
                          ARM_CAN_BIT_PHASE_SEG1(1U) |         // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
                          ARM_CAN_BIT_PHASE_SEG2(1U) |         // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
                          ARM_CAN_BIT_SJW(1U));                // Resynchronization jump width is same as phase segment 2
	for(i=0,i<lengt,i++)
	{
		Driver_CAN1.ObjectSetFilter(0,ARM_CAN_FILTER_ID_EXACT_ADD,
																	ARM_CAN_STANDARD_ID(*(adresse_Filtre+i),0);
	}
}

/*
Fonction : Permet d'envoyer jusqu'� 10 octets de donn�e sur le bus CAN

Variable d'entr�e :
		- ID : 		 	Identifiant CAN de la trame. ID cod�e sur un octet
		- *Data :		Pointeur renvoyant sur le premi�re octet de DATA � transmettre
		- lengt :		Nombre d'octet � transmettre

Cr�ateur : Matthieu
*/
void send_CAN_DATA(char ID, char *DATA, char lengt);
{
	char data_Buf[10];
	char i;
	
	ARM_CAN_MSG_INFO   tx_msg_info;
	tx_msg_info.id = ARM_CAN_STANDARD_ID(ID);	//ID CAN du message
	tx_msg_info.rtr = 0;	//0 pour trame de DATA
	for (i=0;i<lengt;i++)data_Buf [i]= *(DATA+i);
	
	Driver_CAN2.MessageSend(1,&tx_msg_info,data_Buf,lengt);
}

/*
Fonction : Permet d'initialis� un bus I2C

Cr�ateur : Matthieu
*/
void init_I2C(void)
{
	Driver_I2C1.Initialize(NULL);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(	ARM_I2C_BUS_SPEED,							// 2nd argument = d�bit
												ARM_I2C_BUS_SPEED_STANDARD  );	// 100 kHz
	Driver_I2C1.Control(	ARM_I2C_BUS_CLEAR,0 );
}

/*
Fonction : Permet d'initialis� un bus I2C

Cr�ateur : Matthieu
*/
void init_I2C(void)
{
	Driver_I2C1.Initialize(NULL);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(	ARM_I2C_BUS_SPEED,							// 2nd argument = d�bit
												ARM_I2C_BUS_SPEED_STANDARD  );	// 100 kHz
	Driver_I2C1.Control(	ARM_I2C_BUS_CLEAR,0 );
}

/*
Fonction : Permet d'envoyer jusqu'� 10 octets de donn�e sur le bus I2C

Variable d'entr�e :
		- ADRESSE : 		 	Identifiant CAN de la trame. ID cod�e sur un octet
		- *Data :		Pointeur renvoyant sur le premi�re octet de DATA � transmettre
		- lengt :		Nombre d'octet � transmettre

Cr�ateur : Matthieu
*/
void send_I2C_DATA(char ADRESSE, char *DATA, char lengt)
{
	char data_Buf[10];
	char i;
	
	for (i=0;i<lengt;i++)data_Buf [i]= *(DATA+i);
	Driver_I2C1.MasterTransmit (ADRESSE, data_Buf, lengt, false);		// false = avec stop
	while (Driver_I2C1.GetStatus().busy == 1);	// attente fin transmission
}

/*
Fonction : Permet de demander � un slave d'envoyer jusqu'� 10 octets de donn�e sur le bus I2C

Variable d'entr�e :
		- ADRESSE : 			Identifiant CAN de la trame. ID cod�e sur un octet
		- *DATA_SEND :		Pointeur renvoyant sur le premi�re octet de DATA � transmettre pour la trame de commande
		- lengt_Send :		Nombre d'octet � transmettre pour la trame de commande
		- *DATA :					Pointeur renvoyant sur la premi�re case du tableau qui contiendra les DATAs � recevoir
		- lengt :					Nombre d'octet � recevoir

Cr�ateur : Matthieu
*/
void askf_I2C_DATA(char ADRESSE, char *DATA_SEND, char lengt_Send, char *DATA, char lengt)
{
	char data_Buf_Send[10];
	char i;
	
	if (lengt_Send==0)
	{
		Driver_I2C1.MasterTransmit (ADRESSE, NULL, 0, false);		// false = avec stop
	}
	else
	{
		for (i=0;i<lengt_Send;i++)data_Buf_Send [i]= *(DATA_SEND+i);
		Driver_I2C1.MasterTransmit (ADRESSE, data_Buf, lengt, false);		// false = avec stop
	}
	while (Driver_I2C1.GetStatus().busy == 1);	// attente fin transmission
	
	Driver_I2C0.MasterReceive (ADRESSE, &DATA, lengt, false);// false = avec stop
	while (Driver_I2C0.GetStatus().busy == 1);// attente fin transmission	
}

/*
Fonction : Permet d'initialis� une liaison UART

Cr�ateur : Matthieu
*/
void init_USART()
{
	Driver_USART1.Initialize(NULL);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(	ARM_USART_MODE_ASYNCHRONOUS |
													ARM_USART_DATA_BITS_8 |
													ARM_USART_STOP_BITS_1 |
													ARM_USART_PARITY_NONE |
													ARM_USART_FLOW_CONTROL_NONE,
													115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);
}

/*
Fonction : Permet d'envoyer jusqu'� 10 octets de donn�e sur le bus I2C

Variable d'entr�e :
		- *Data :		Pointeur renvoyant sur le premi�re octet de DATA � transmettre
		- lengt :		Nombre d'octet � transmettre

Cr�ateur : Matthieu
*/
void send_USART_DATA (char *DATA, char lengt)
{
	while (Driver_USART1.GetStatus().tx_busy==1); //attente que le buffer soit vide
	Driver_USART1.Send(&DATA, lengt);
}