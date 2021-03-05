/*
Projet : Voiture Autonome
Groupe : IT2R2
Fichier pour ressenser toutes les fonctions utiles que l'on pourrait tous avoir besoin d'utiliser sur plusieurs micro
*/

//Interface CAN
void init_CAN_Transmiter();
void init_CAN_Receiver(char *adresse_Filtre, char lengt);
void send_CAN_DATA(char ID, char *DATA, char lengt);

//Interface I2C
void init_I2C(void);
void send_I2C_DATA(char ADRESSE, char *DATA, char lengt);
void askf_I2C_DATA(char ADRESSE, char *DATA_SEND, char lengt_Send, char *DATA, char lengt);

//Interface UART
void init_USART();
void send_USART_DATA (char *DATA, char lengt);
