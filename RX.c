/*
* main.c: Parte Receptor
*
* Created on: 16 ago. 2021
* Author: Teresa Fiel
*/
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"
#include "IEEE_CAN.h"
#include "driverlib/sysctl.h"
int Sensor_1 = 0;
int Sensor_2 = 0;

float tp= 0; //Tiempo de propagacion
uint32_t encendido = 0; //bit de encendido, se transmitiran 32 bits
int i=0;
//--------------------------------------------------------------------
//%%%%%% INICIALIZACIï¿1⁄2N DE PUERTOS ASOCIADOS AL CAN0 %%%%%%%%%%%
// CAN0Rx: PA0 CAN0Tx: PA1
//--------------------------------------------------------------------
void Config_Puertos(void){ //(TM4C1294NCPDT)
SYSCTL_RCGCGPIO_R|=0x1; //Reloj Puerto A
while((SYSCTL_PRGPIO_R&0x1)==0){}
GPIO_PORTA_AHB_CR_R=0x3;
GPIO_PORTA_AHB_AFSEL_R=0x3; //PA0 y PA1 funciï¿1⁄2n alterna
GPIO_PORTA_AHB_PCTL_R=0x77; //Funciï¿1⁄2n CAN a los pines PA0-PA1
GPIO_PORTA_AHB_DIR_R=0x2; //PA1 Salida Tx y PA0 Entrada Rx
GPIO_PORTA_AHB_DEN_R=0x3; //Hab funciï¿1⁄2n digital PA0 y PA1
}
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%% INICIALIZACIï¿1⁄2N CAN0 %%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void Config_CAN(void){
SYSCTL_RCGCCAN_R=0x1; //Reloj modulo 0 CAN
while((SYSCTL_PRCAN_R&0x1)==0){}
//Bit Rate= 1 Mbps CAN clock=16 [Mhz]
CAN0_CTL_R=0x41; //Deshab. modo prueba, Hab. cambios en la config. y hab.
inicializacion
CAN0_BIT_R=0x2BC0; //TSEG2=4 TSEG1=9 SJW=0 BRP=0
//Lenght Bit time=[TSEG2+TSEG1+3]*tq
// =[(Phase2-1)+(Prop+Phase1-1)+3]*tq
CAN0_CTL_R&=~0x41; //Hab. cambios en la config. y deshab. inicializacion
CAN0_CTL_R|=0x2; //Hab de interrupciï¿1⁄2n en el mï¿1⁄2dulo CAN
NVIC_EN1_R|=((1<<(38-32)) & 0xFFFFFFFF); //(TM4C1294NCPDT)
} //El 38 sale de p.116
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%% RESOLUCION DE ERRORES %%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void CAN_Error(void){
static int ent=0;
if(CAN0_STS_R&0x80){
if(ent){
NVIC_APINT_R|=0x4; //Reinicio de todo el sistema
}else{
CAN0_CTL_R=0x41; //Hab. cambios en la config. y hab. inicializacion
CAN0_CTL_R|=0x80; //Hab. modo prueba
CAN0_TST_R|=0x4; //Hab. Modo silencio
CAN0_CTL_R&=~0x41; //Hab. cambios en la config. y deshab.
inicializacion
SysCtlDelay(333333);
CAN0_CTL_R=0x41; //Hab. cambios en la config. y hab. inicializacion
CAN0_TST_R&=~0x4; //Deshab. Modo silencio
CAN0_CTL_R&=~0x41; //Hab. cambios en la config. y deshab.
inicializacion
ent++;
}
}
}
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%% HARDWARE DEL MONITOR %%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
//Esta función inicará todos los leds y botones para poder activar el otro
microcnontrolador
void Monitor_init (void) {
SYSCTL_RCGCGPIO_R |= 0x2200; // 1) Habilita reloj para Puerto K,P (P.382)
while((SYSCTL_PRGPIO_R & 0x2200) == 0){}; // Se espera a que el reloj se
estabilice (p.499)
//Configuracion de puerto K LEDS Indicadores
//PIN 2 -----INDICADOR PARA SENSOR 1
//PIN 3 -----INDICADOR PARA SENSOR 2
GPIO_PORTK_DIR_R = 0XFF; // PK Salidas
GPIO_PORTK_DEN_R = 0xFF; // PK // Habilita funcion digital
}
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%% INTERRUPCION DEL CAN0 %%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void Inter_CAN0(void){
uint8_t NoInt;
uint32_t Rx[3];

NoInt=CAN0_INT_R; //Lectura del apuntador de interrupciones ¿Qué localidad
recibió el dato?
CAN0_STS_R&=~0x10; //Limpieza del bit de recepcion
if(NoInt==0x2)
{ //Recibe datos de los potenciómetros
Rx[0]=CAN_Rx(NoInt); //Recepción de datos
Sensor_1 = Rx[0] & 0xFFF; //Se obtienen los primeros 16 bits
Sensor_2 = (0xFFF & ( Rx[0] >>16)); //se obtienen los siguientes 16
bits
//Parte del sensor 1
if (Sensor_1 < 4096/2) //Cuando es menor a 3.3/2 Volts
GPIO_PORTK_DATA_R |= 0x01; //Enciende LED para el sensor 1 PK0
else
GPIO_PORTK_DATA_R = GPIO_PORTK_DATA_R & 0xFE; //APAGA UNICAMENTE
ESE LED
//Parte del sensor 2
if (Sensor_2 > 500) //Cuando es menor a 2.5 Volts
GPIO_PORTK_DATA_R |= 0x02; //Enciende LED para el sensor de 2 PK1
else
GPIO_PORTK_DATA_R = GPIO_PORTK_DATA_R & 0xFD; //APAGA UNICAMENTE
ESE LED
}
}
//-----------------------------------------------------------------------------
-------
//%%%%%%%%%%%%%%%%%%%% FUNCION QUE PREPARA LOS 32 MENSAJES OBJETO
%%%%%%%%%%%%%%%%%%%%
//-----------------------------------------------------------------------------
-------
void localidadesCAN(void)
{
//Localidad 2 Rx con Msk
CAN_Memoria_Arb(0x333,false,0x2);//Lectura de ambos potenciometros
CAN_Memoria_CtrlMsk(0x111,4,false,true,false,0x2);
}
//------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%% PROGRAMA PRINCIPAL %%%%%%%%%%%%%%%%%%%%
//------------------------------------------------------------------
void main(void){
Monitor_init ();// Inicializacion del hardware que no es CAN
Config_Puertos();
Config_CAN();
localidadesCAN();
while(1){}
}