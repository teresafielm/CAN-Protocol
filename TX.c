/*
* main.c: Parte Transmisor
*
* Created on: 16 ago. 2021
* Author: Teresa Fiel
*/
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"
#include "IEEE_CAN.h"
#include "driverlib/sysctl.h"
uint64_t Rx[5];
uint32_t MSJ = 0; //Mensaje de 32 bits a enviar
int i=1;
//Variables Globales
int encendido = 0;
int Sensor_1 = 0;
int Sensor_2 = 0;
float tp= 0; //Tiempo de propagacion
int c = 0;
//--------------------------------------------------------------------
//%%%%%% INICIALIZACION DE PUERTOS ASOCIADOS AL CAN0 %%%%%%%%%%%
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
//%%%%%%%%%%%%%%%%%%%% RESOLUCIÃ“N DE ERRORES %%%%%%%%%%%%%%%%%%%%
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
void Sensores_init (void) {
//Funcion que habilitará todos los sensores a utilizar.
SYSCTL_RCGCGPIO_R |= 0x2210; // 1) Habilita reloj para Puerto E,K,P (P.382)
SYSCTL_RCGCTIMER_R |= 0X8; //RELOJ Y HABILITA TIMER 3 (p.380)
while((SYSCTL_PRGPIO_R & 0x2210) == 0){}; // Se espera a que el reloj se
estabilice (p.499)
//CONFIGURACION DEL PUERTO PARA EL ADC.
//Configuracion del puerto E para potenciometro, sensor luz y sensor agua
GPIO_PORTE_AHB_DIR_R = 0x00; // 2) PE3 entrada (analógica)
GPIO_PORTE_AHB_AFSEL_R |= 0x0E; // 3) Habilita Función Alterna de
PE3,PE2,PE1 AIN0,AIN1, AIN2
GPIO_PORTE_AHB_DEN_R = 0x00; // 4) Deshabilita Función Digital de PE3,PE2,
PE1, AIN2
GPIO_PORTE_AHB_AMSEL_R |= 0x0E; // 5) Habilita Función Analógica de
PE3,PE2, PE1, AIN2
//Configuracion de puerto K LEDS Indicadores de encendido o apagado
GPIO_PORTK_DIR_R = 0XFF; // PK Salidas
GPIO_PORTK_DEN_R = 0xFF; // PK // Habilita funcion digital
//Configuracion del ADC
SYSCTL_RCGCADC_R |= 0x01; // 6) Habilita reloj para ADC0(p. 396)
while((SYSCTL_PRADC_R & 0x01) == 0); // Se espera a que el reloj se
estabilice
//MAPA DE REGISTROS EN 1073
ADC0_PC_R = 0x07; // 7) (p.1159) Maxima tasa de muestreo 1M muestra/seg
ADC0_SSPRI_R = 0x1023; // 8)SS2 con la más alta prioridad
ADC0_ACTSS_R = 0x000; // 9) Deshabilita SS2 antes de cambiar configuración
de registros (p. 1076)
ADC0_EMUX_R = 0x0500; // 10) Se configura SS2 para disparar muestreo por
timer (p.1091) Interrupcion
ADC0_SAC_R = 0x0; // 11) Se configura para no tener sobremuestreo por
hardware(default)(p. 1105)
ADC0_CTL_R = 0x0; //12) Se configura con referencias internas (default VDDA
and GNDA) (p. 1107)
ADC0_SSOP2_R = 0x0000; // 13) Se configura para salvar los resultados del
ADC en FIFO (default)(p. 1134)
ADC0_SSEMUX2_R = 0; // 16) Canales del SS2 para 1° y segunda muestra en
AIN(15:0) (p.1125)
ADC0_SSMUX2_R = 0x0210; // 15) Se configura entradas 1°muestra=AIN 0,
2°muestra=AIN 1(p.1109), 3 muestra AIN2
ADC0_SSTSH2_R = 0x0000; // 14) Se configura el ADC para un periodo de 4
para tmp S&H en el secuenciador 2 (default) (p. 1134)
//Esto se hace para que sea estable el sensor
ADC0_SSCTL2_R = 0x0600; // 17) Final con tercera muestra Si: Sensor tmp,no:
AIN, Habilita interrupcion; No:muestra diferencial (p.1111)
ADC0_IM_R = 0x0004; // 18) habilita interrupción SS2 (p. 1081) //ACTIVA
INTERRUPCION CUANDO
//TERMINE DE CONVERTIR TERMINANDO EL TIMER
NVIC_EN0_R|= 1<<(16-0); //HABILITA LA INTERRUPCION 16 (ADC_SS2)
ADC0_ACTSS_R |= 0x0004; // 19) Habilita SS2 (p. 1076)
//No se necesita establecer una prioridad en las interrupciones porque no
son continuas
// SINCRONIZACIÓN DEL PLL PARA UTILIZAR PIOSC
SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR; // encender PLL
while((SYSCTL_PLLSTAT_R&0x01)==0); // espera a que el PLL fije su
frecuencia
SYSCTL_PLLFREQ0_R &= ~SYSCTL_PLLFREQ0_PLLPWR; // apagar PLL
ADC0_ISC_R = 0x0004; // Se recomienda Limpia la bandera RIS del SS2
// --------------------------------------------------------------
// CONFIGURACION DEL TIMER DEL ADC
// -------------------------------------------------------------
//
TIMER3_CTL_R=0X0000000; //DESHABILITA TIMER 3 PARA CONFIGURAR (p.986)
TIMER3_CFG_R= 0X00000000; //CONFIGURA TIMER DE 32 BITS (p. 976)
//TIMER3_TAMR_R= 0X00000002; //CONFIGURAR PARA MODO PERIODICO CUENTA HACIA
ABAJO (p. 977)
TIMER3_TAMR_R= 0X00000012; //CONFIGURAR PARA MODO PERIODICO CUENTA HACIA
ARRIBA (p. 977)
TIMER3_TAILR_R= 0XFFFFF; // VALOR DE RECARGA (p.1004) 20ms //Cada 20ms hace
la interrupcion aprox
//TIMER3_TAILR_R= 0X0004E200; // VALOR DE RECARGA (p.1004)
TIMER3_TAPR_R= 0X00; // PRESCALADOR DE TIMER A, SOLO PARA MODOS DE 16 BITS
(p.1008)
TIMER3_ADCEV_R = 0X01; // HABILITA MODO CAPTURA DEL TIMER 3 COMO EVENTO DE
DISPARO PARA EL ADC
TIMER3_ICR_R= 0X00000001 ; //LIMPIA POSIBLE BANDERA PENDIENTE DE TIMER3
(p.1002)
//TIMER3_IMR_R |= 0X00000001; //ACTIVA INTRRUPCION DE TIMEOUT (p.993)
TIMER3_CTL_R |= 0X00000021; //HABILITA TIMER 3 Y ACTIVA TRIGGER PARA EL ADC
(p.986)
}
//--------------------------------------------------------------------
//%%%%%%%%%%%%%%%%% INTERRUPCION DEL CAN0 %%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------
void Inter_CAN0(void){
uint8_t NoInt;
NoInt=CAN0_INT_R; //Lectura del apuntador de interrupciones
CAN0_STS_R&=~0x10; //Limpieza del bit de recepcion
i++;
if(NoInt==0x1){
Rx[0]=CAN_Rx(NoInt); //Recepción de datos
encendido = Rx[0]; //Recibe si se enciende o no el coche
}
if(NoInt==0x4){//Solicitud de trama remota
Rx[1]=CAN_Rx(NoInt); //Recepción de datos
}
}
void ADC0_SS2IntHandler(void){
//LA INTERRUPCION SE DISPARA CADA 20ms CUANDO TERMINA DE HACER LA
CONVERSIÓN
//LIMPIA BANDERA
MSJ = 0;
if (encendido == 1)
{
Sensor_1 = (ADC0_SSFIFO2_R&0xFFF); //RESULTADO DE LA PRIMERA MUESTRA
Sensor_2 = (ADC0_SSFIFO2_R&0xFFF); //RESULTADO DE LA SEGUNDA MUESTRA
//Mensaje a enviar de ambos sensores
MSJ =(0x00000FFF & Sensor_1); //Byte 0-1
MSJ|=(0x0FFF0000 & ( Sensor_2<<16)); //Byte 2-3
CAN_Memoria_Dato(MSJ,0x2); //TX SENSORES
CAN_Tx(0x2);//Transmite datos sensores
SysCtlDelay(213); //Funcion de retardo: SysCtDelay(1) = 187.5 ns, 100us
}
ADC0_ISC_R = 0x0004;//Limpia la bandera RIS del ADC0
}//Fin de interrupcion

//------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%% FUNCION QUE PREPARA LOS 32 MENSAJES OBJETO
%%%%%%%%%%%%%%%%%%%%
//------------------------------------------------------------------
void localidadesCAN(void){
//Localidad 2 Tx, Transmisor
CAN_Memoria_Arb(0x333,true,0x2);//ID:0X2222 True, mandar-- a localidad 0x2
CAN_Memoria_CtrlMsk(0,4,false,false,false,0x2);
}
//------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%% PROGRAMA PRINCIPAL %%%%%%%%%%%%%%%%%%%%
//------------------------------------------------------------------
void main(void){
Sensores_init(); //Funcion que inicializa ADC0 para el sensor de
temperatura y el sensor de luz.
Config_Puertos();
Config_CAN();
localidadesCAN();//Se preparan los mensajes objeto
encendido = 1;
while(1)
{}
}