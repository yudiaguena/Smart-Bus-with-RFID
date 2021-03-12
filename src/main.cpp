//=================================================================================
// Trabalho de Conclusão de Curso- Técnico em eletroeletronica
//
// Smart Bus-Stop - 4TA - 2º Semestre de 2018.
//
// Professores orientadores : Tales Trocolletto e Celso Milan.
//
// Alunos: André Gomes Moreno ; Guilherme Xavier dos Santos ; Patrick Yudi Aguena ; Renan Verissimo Mendes 
//	
//=================================================================================

//				BIBLIOTECAS
//=================================================================================

#include <p18f4550.h>
#include <delays.h>
#include "lcd_4vias.h"
//#include "delays.h"

//#include "i2c.h"
#include "usart.h"
#include <adc.h>
//#include "Teclado_4x3.h"
#include "Teclado_5x5.h"
//#include "Teclado_4x4.h"
//#include "sw_spi.h"
#include <timers.h>

//=================================================================================
//			CONFIGURAÇÃO DO MICROCONTROLADOR
//=================================================================================

#pragma config	FOSC	=	HS
#pragma config	MCLRE	=	ON
#pragma config	PWRT	=	ON
#pragma config	PBADEN	=	OFF
#pragma config	WDT		=	OFF
#pragma config	LVP		=	OFF

//=================================================================================
//				DEFINIÇÕES DE HARDWARE
//=================================================================================

//Declaração dos Ports

//*******************************
//Port A, do RA0 até RA5 
//*******************************
//#define BACK_LIGHT	PORTAbits.RA0	//Saída para o BACK Light
#define IN_AD		PORTAbits.RA0	//Entrada analógica
#define BUZZER		PORTAbits.RA1	//Saída para o buzzer
//#define EN		PORTAbits.RA2	//*****Definido no lcd_4vias.h*****						Tem pull down externo
//#define LIVRE		PORTAbits.RA3	//														Tem pull down externo
#define Botao_S3	PORTAbits.RA4	//6ª entrada digital	pino 7 do J3					Tem pull up externo
#define tocando		PORTAbits.RA5	//7ª entrada digital	pino 8 do J3					Tem pull up externo

//*******************************
//Port B, do RB0 até RB7
//*******************************
#define I2C_SDA		PORTBbits.RB0	//Entrada e saída de dados da comunicação I2C
#define I2C_SCL		PORTBbits.RB1	//Saída de clock para a comunicação I2C
//#define RS		PORTBbits.RB2	//*****Definido no lcd_4vias.h*****
#define DB4_LCD		PORTBbits.RB3	//Saída para o DB4 do LCD	Pino 14 do J5
#define DB5_LCD		PORTBbits.RB4	//Saida para o DB5 do LCD   //Entrada = Tecla Right		Tem pull down externo
#define DB6_LCD		PORTBbits.RB5	//Saída para o DB6 do LCD	Pino 13 do J5
#define DB7_LCD		PORTBbits.RB6	//Saída para o DB7 do LCD   //Entrada = Tecla LEFT		Tem pull down externo
#define UP			PORTBbits.RB7	//Entrada	Tecla UP									Tem pull down externo

//*******************************
//Port C, do RC0 até RC7 menos o RC3 que foi dedicado para a USB
//*******************************
#define C1			PORTCbits.RC0	//0ª entrada digital	pino 1 do J3					Tem pull up externo
#define C2			PORTCbits.RC1	//1ª entrada digital	pino 2 do J3					Tem pull up externo
#define C3			PORTCbits.RC2	//2ª entrada digital	pino 3 do J3					Tem pull up externo
//RC3 é da USB
#define Botao_S1	PORTCbits.RC4	//4ª entrada digital	pino 5 do J3					Tem pull up externo
#define Botao_S2	PORTCbits.RC5	//5ª entrada digital	pino 6 do J3					Tem pull up externo
#define busy_voz	PORTCbits.RC6	//TX da serial ligado diretamente no pino 11 do MAX232
#define RX			PORTCbits.RC7	//RX da serial ligado diretamente no pino 12 do MAX232

//*******************************
//Port D, do RD0 até RD7
//*******************************
#define reset_voz	PORTDbits.RD0	//0ª saída digital		pino 1 do J1
#define clk_voz		PORTDbits.RD1	//1ª saída digital		pino 2 do J1
#define data_voz	PORTDbits.RD2	//2ª saída digital		pino 3 do J1
#define L1			PORTDbits.RD3	//3ª saída digital		pino 4 do J1
#define L2			PORTDbits.RD4	//4ª saída digital		pino 5 do J1
#define L3			PORTDbits.RD5	//5ª saída digital		pino 6 do J1
#define L4			PORTDbits.RD6	//6ª saída digital		pino 7 do J1
#define L5			PORTDbits.RD7	//7ª saída digital		pino 8 do J1

//*******************************
//Port E, do RE0 até RE3 o RE3 é só entrada pois é o MCLR
//*******************************
#define DOWN		PORTEbits.RE0	//Entrada	Tecla Down					Tem pull down externo
#define _ESC		PORTEbits.RE1	//Entrada	Tecla Esc					Tem pull down externo
#define ENTER		PORTEbits.RE2	//Entrada	Tecla Enter					Tem pull down externo
//#define MCLR		PORTEbits.RE3	//MCLR'

//=================================================================================
//				DEFINIÇÕES DE CONSTANTES
//=================================================================================                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    

#define LIGA 0
#define DESL 1
#define CONST_T_TIMER0 (65536-416)
#define CONST_T_TIMER1 (65536-416)

#define carta1_d0 	0x02
#define carta1_d1 	0x31
#define carta1_d2 	0x36
#define carta1_d3 	0x30
#define carta1_d4 	0x30
#define carta1_d5 	0x37
#define carta1_d6 	0x42
#define carta1_d7 	0x45
#define carta1_d8 	0x37
#define carta1_d9 	0x36
#define carta1_d10 	0x31
#define carta1_d11 	0x45
#define carta1_d12 	0x42
#define carta1_d13 	0x03

#define carta2_d0 	0x02
#define carta2_d1 	0x31
#define carta2_d2 	0x36
#define carta2_d3 	0x30
#define carta2_d4 	0x30
#define carta2_d5 	0x37
#define carta2_d6 	0x42
#define carta2_d7 	0x45
#define carta2_d8 	0x38
#define carta2_d9 	0x44
#define carta2_d10	0x36
#define carta2_d11 	0x35
#define carta2_d12 	0x33
#define carta2_d13 	0x03

#define carta3_d0 	0x02
#define carta3_d1 	0x31
#define carta3_d2 	0x36
#define carta3_d3 	0x30
#define carta3_d4 	0x30
#define carta3_d5 	0x37
#define carta3_d6 	0x42
#define carta3_d7 	0x45
#define carta3_d8 	0x42
#define carta3_d9 	0x33
#define carta3_d10	0x42
#define carta3_d11 	0x42
#define carta3_d12 	0x44
#define carta3_d13 	0x03

#define carta4_d0 	0x02
#define carta4_d1 	0x31
#define carta4_d2 	0x36
#define carta4_d3 	0x30
#define carta4_d4 	0x30
#define carta4_d5 	0x37
#define carta4_d6 	0x42
#define carta4_d7 	0x45
#define carta4_d8 	0x33
#define carta4_d9 	0x45
#define carta4_d10	0x41
#define carta4_d11 	0x36
#define carta4_d12 	0x34
#define carta4_d13 	0x03

#define carta5_d0 	0x02
#define carta5_d1 	0x31
#define carta5_d2 	0x36
#define carta5_d3 	0x30
#define carta5_d4 	0x30
#define carta5_d5 	0x37
#define carta5_d6 	0x42
#define carta5_d7 	0x45
#define carta5_d8	0x36
#define carta5_d9 	0x30
#define carta5_d10	0x36
#define carta5_d11 	0x38
#define carta5_d12 	0x44
#define carta5_d13 	0x03

//=================================================================================
//				DEFINIÇÕES DE DELAY
//=================================================================================

//#define delay_us_250(n)  	Delay1TCYx(n)
#define delay_us_2500(n)  	Delay10TCYx(n/10)
#define delay_ms_25(n)  	Delay100TCYx(n*10)
#define delay_ms_250(n)  	Delay1KTCYx(n)
#define delay_ms_2500(n) 	Delay10KTCYx(n/10)

//=================================================================================
//				DECLARAÇÃO DAS VARIÁVEIS
//=================================================================================

// 1bit

struct
{
	unsigned char f_00:1;
	unsigned char f_01:1;
	unsigned char f_02:1;
	unsigned char f_03:1;
	unsigned char f_04:1;
	unsigned char f_05:1;
	unsigned char f_06:1;
	unsigned char f_07:1;
	unsigned char f_08:1;
	unsigned char f_09:1;
	unsigned char f_10:1;
	unsigned char f_11:1;
	unsigned char f_12:1;
	unsigned char f_13:1;
	unsigned char f_14:1;
	unsigned char f_15:1;
	unsigned char f_16:1;
	unsigned char f_17:1;
	unsigned char f_18:1;
	unsigned char f_19:1;
	unsigned char f_20:1;
	unsigned char f_21:1;
	unsigned char f_22:1;
	unsigned char f_23:1;
	unsigned char f_24:1;
	unsigned char f_25:1;
	unsigned char f_26:1;
	unsigned char f_27:1;
	
}Flags;

#define f_sensor 			Flags.f_00
#define f_tempo	 			Flags.f_01
#define f_a		 			Flags.f_02
#define f_b		 			Flags.f_03
#define f_c		 			Flags.f_04
#define f_Check				Flags.f_05
#define f_play				Flags.f_06
#define f_tamanho_palavra	Flags.f_07
#define f_linha1			Flags.f_08
#define f_linha2			Flags.f_09
#define f_linha3			Flags.f_10
#define f_linha4			Flags.f_11
#define f_inicializando		Flags.f_12
#define f_on_off			Flags.f_13
#define f_d					Flags.f_14
#define f_e					Flags.f_15
#define f_chegou_dado		Flags.f_16
#define f_carta1			Flags.f_17
#define f_carta2			Flags.f_18
#define f_carta3			Flags.f_19
#define f_carta4			Flags.f_20
#define f_cartao_valido		Flags.f_21
#define	f_checa_dado		Flags.f_22
#define f_cartao_invalido	Flags.f_23

#define	Liga_rx				RCSTAbits.CREN

// 8bits
unsigned char	c_teste		= 0x00;
unsigned char	tecla		= 0x00;
unsigned char	dado		= 0x00;
unsigned char	dado1		= 0x00;
unsigned char	cont_250ms	= 0x00;
unsigned char	cont_100ms	= 0x00;
unsigned char	dado_ERRO		= 0x00;
unsigned char	dado_RX			= 0x00;
unsigned char	Controle_dado	= 0X00;
unsigned char	RH_byte1 	= 0x00;
unsigned char	RH_byte2 	= 0x00;
unsigned char	T_byte1 	= 0x00;
unsigned char	T_byte2 	= 0x00;
unsigned char	Temperatura = 0x00;
unsigned char	RH 			= 0x00;
unsigned char	Sum 		= 0x00;
unsigned char	z 			= 0x00;
unsigned char	result 		= 0x00;
unsigned char	k 			= 0x00;
unsigned char	Tecla		= 0x00;
unsigned char	palavra		= 0x00;
unsigned char	passo		= 0x00;
unsigned char	linha		= 0x00;

unsigned char	dado_0		= 0x00;
unsigned char	dado_1		= 0x00;
unsigned char	dado_2		= 0x00;
unsigned char	dado_3		= 0x00;
unsigned char	dado_4		= 0x00;
unsigned char	dado_5		= 0x00;
unsigned char	dado_6		= 0x00;
unsigned char	dado_7		= 0x00;
unsigned char	dado_8		= 0x00;
unsigned char	dado_9		= 0x00;
unsigned char	dado_10		= 0x00;
unsigned char	dado_11		= 0x00;
unsigned char	dado_12		= 0x00;
unsigned char	dado_13		= 0x00;

//Matrix de 8 bits
//unsigned char	linha[5] 	= {0,0,0,0,0};

//16 bits
unsigned int	c_teste1 	= 0x0000;
unsigned int	in_ad 		= 0x0000;

unsigned int 	stopped 	= 0x0000;
unsigned int 	scan 		= 0x0000;
unsigned int 	volume	 	= 0xFFF7;

unsigned int 	cont_3S		= 0x0000;

//=================================================================================
//				DECLARAÇÃO DAS ROTINAS
//=================================================================================
void Int_Alta (void);
void Trata_Alta (void);
void Trata_Timer0(void);
void Trata_Timer1(void);
void Trata_Serial (void);

void reset_module_voz (void);
void play_voz (unsigned int voiceNumber);
void stop_voz(void);
void volume_voz(void);
void envia_comando_voz (unsigned int comando_voz);

void Cabecalho(void);
unsigned char ReadData();
void Read_DHT11(void);

void posicao_LCD(void);

void Checa_dado(void);
void Armazena_Carta (void);

void SubTeste (void);

//=================================================================================
//					Verificação das INTERRUPÇÕES
//=================================================================================

#pragma code Alta = 0x08	//Como se fosse o ORG0000h do ASSEBLER

void Int_Alta (void)		//Já salto para o endereço onde tem mais espaço
{
	Trata_Alta ();			//Vou para rotina pois aqui não tem espaço
}
#pragma code				//para finalizar o pragma do ORG000h
#pragma interrupt Trata_Alta//para indicar ao compilador que a prioridade da interrupção é alta
///////////////////////////

///////////////////////////


void Trata_Alta (void)
{	
	if (INTCONbits.TMR0IF == 1)
	{
		Trata_Timer0();	
	}
	if (PIR1bits.TMR1IF == 1)
	{
		Trata_Timer1();	
	}	
	if (PIR1bits.RCIF == 1)
	{
		Trata_Serial ();
	}
}

//=================================================================================
//					Tratamento das INTERRUPÇÕES
//=================================================================================
void Trata_Timer0(void)
{
	WriteTimer0(CONST_T_TIMER0);
	INTCONbits.TMR0IF = 0;
	
	//Escreva aqui sua interrupção
	
	++cont_3S;
	if (cont_3S == 3000)
	{
		cont_3S=0;
		f_a=1;
	}
}

void Trata_Timer1(void)
{
	WriteTimer1(CONST_T_TIMER1);
	PIR1bits.TMR1IF = 0;
	
	//Escreva aqui sua interrupção
	
	++cont_100ms;
	if (cont_100ms == 100)
	{
		cont_100ms=0;
		
	}
}
void Trata_Serial (void)
{
	dado_RX = ReadUSART();
	f_chegou_dado=1;
}


//=================================================================================
//					SUBROTINAS
//=================================================================================
void zera_dado (void)
{
		Controle_dado=0x00;
		dado_0 		=0;
		dado_1 		=0;
		dado_2 		=0;
		dado_3 		=0;
		dado_4 		=0;
		dado_5 		=0;
		dado_6 		=0;
		dado_7 		=0;
		dado_8 		=0;
		dado_9 		=0;
		dado_10 	=0;
		dado_11 	=0;
		dado_12 	=0;
		dado_13 	=0;
	
}	




void Armazena_Carta(void)
{
	if (f_chegou_dado == 1)
	{
		f_chegou_dado=0;
		if(RCSTAbits.OERR==1)
		{
			dado_ERRO = ReadUSART();
			delay_ms_2500(100);
			dado_ERRO = ReadUSART();
			
			zera_dado ();
			
			f_chegou_dado=0;
			Liga_rx=1;
		}
		else
		{
			
			if (Controle_dado==0)
			{
				dado_0 = dado_RX;	//guarda o dado da recepção no dado_0
				++Controle_dado;
			}
			
			else if (Controle_dado==1)
			{
				dado_1 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==2)
			{
				dado_2 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==3)
			{
				dado_3 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==4)
			{
				dado_4 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==5)
			{
				dado_5 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==6)
			{
				dado_6 = dado_RX;
				++Controle_dado;
			}
			
			else if (Controle_dado==7)
			{
				dado_7 = dado_RX;
				++Controle_dado;

			}
			else if (Controle_dado==8)
			{
				dado_8 = dado_RX;
				++Controle_dado;

			}
			else if (Controle_dado==9)
			{
				dado_9 = dado_RX;
				++Controle_dado;

			}
			else if (Controle_dado==10)
			{
				dado_10 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==11)
			{
				dado_11 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==12)
			{
				dado_12 = dado_RX;
				++Controle_dado;
			}
			else if (Controle_dado==13)
			{
				dado_13 = dado_RX;	
				Liga_rx = 0;
				Controle_dado=0x00;
				f_checa_dado=1;			
			}	
		}	
	}
}

void Checa_dado(void)
{
	if (f_checa_dado==1)
	{
		f_checa_dado=0;
		
		if ((dado_0 == carta1_d0)&&
			(dado_1 == carta1_d1)&&
			(dado_2 == carta1_d2)&&
			(dado_3 == carta1_d3)&&
			(dado_4 == carta1_d4)&&
			(dado_5 == carta1_d5)&&
			(dado_6 == carta1_d6)&&
			(dado_7 == carta1_d7)&&
			(dado_8 == carta1_d8)&&
			(dado_9 == carta1_d9)&&
			(dado_10 == carta1_d10)&&
			(dado_11 == carta1_d11)&&
			(dado_12 == carta1_d12)&&
			(dado_13 == carta1_d13))
		{
			f_carta1=1;
		
					
		}
		if ((dado_0 == carta2_d0)&&
			(dado_1 == carta2_d1)&&
			(dado_2 == carta2_d2)&&
			(dado_3 == carta2_d3)&&
			(dado_4 == carta2_d4)&&
			(dado_5 == carta2_d5)&&
			(dado_6 == carta2_d6)&&
			(dado_7 == carta2_d7)&&
			(dado_8 == carta2_d8)&&
			(dado_9 == carta2_d9)&&
			(dado_10 == carta2_d10)&&
			(dado_11 == carta2_d11)&&
			(dado_12 == carta2_d12)&&
			(dado_13 == carta2_d13))
		{
			f_carta2=1;
		}
		if ((dado_0 == carta3_d0)&&
			(dado_1 == carta3_d1)&&
			(dado_2 == carta3_d2)&&
			(dado_3 == carta3_d3)&&
			(dado_4 == carta3_d4)&&
			(dado_5 == carta3_d5)&&
			(dado_6 == carta3_d6)&&
			(dado_7 == carta3_d7)&&
			(dado_8 == carta3_d8)&&
			(dado_9 == carta3_d9)&&
			(dado_10 == carta3_d10)&&
			(dado_11 == carta3_d11)&&
			(dado_12 == carta3_d12)&&
			(dado_13 == carta3_d13))
		{
			f_carta3=1;	
		}
		if ((dado_0 == carta4_d0)&&
			(dado_1 == carta4_d1)&&
			(dado_2 == carta4_d2)&&
			(dado_3 == carta4_d3)&&
			(dado_4 == carta4_d4)&&
			(dado_5 == carta4_d5)&&
			(dado_6 == carta4_d6)&&
			(dado_7 == carta4_d7)&&
			(dado_8 == carta4_d8)&&
			(dado_9 == carta4_d9)&&
			(dado_10 == carta4_d10)&&
			(dado_11 == carta4_d11)&&
			(dado_12 == carta4_d12)&&
			(dado_13 == carta4_d13))
		{
			f_carta4=1;
		}
		if ((dado_0 == carta5_d0)&&
			(dado_1 == carta5_d1)&&
			(dado_2 == carta5_d2)&&
			(dado_3 == carta5_d3)&&
			(dado_4 == carta5_d4)&&
			(dado_5 == carta5_d5)&&
			(dado_6 == carta5_d6)&&
			(dado_7 == carta5_d7)&&
			(dado_8 == carta5_d8)&&
			(dado_9 == carta5_d9)&&
			(dado_10 == carta5_d10)&&
			(dado_11 == carta5_d11)&&
			(dado_12 == carta5_d12)&&
			(dado_13 == carta5_d13))
		{
			f_cartao_invalido = 1;	
		}
		zera_dado ();
	}
}	

void reset_module_voz (void)
{
	//pinos do modulo de voz
	//audio_L - vai direto em um almplificador de audio para ligar no auto falante
	
	//reset_voz pino RD0
	//clk_voz	pino RD1
	//data_voz	pino RD2
	//todos os pinos acima devem ser saidas digitais

	//busy	pino RC5 para monitorarmos quando o audio está sendo reproduzido
	
	clk_voz=1;
	reset_voz=1;
	Delay1KTCYx(1);	//dei o tempo pedido no datasheet para start bit 1ms
	reset_voz=0;
	Delay1KTCYx(10);	//dei o tempo pedido no datasheet para start bit 5ms
	reset_voz=1;
	Delay1KTCYx(150);	// ERA 150 dei o tempo pedido no datasheet para start bit 300ms
	Delay1KTCYx(150);	// ERA 150 dei o tempo pedido no datasheet para start bit 300ms
}	
void SubTeste (void)
{
	Liga_rx=0;
	reset_module_voz();
	play_voz(86);//I-02
	delay_ms_250(30);
	while(tocando==0);
	Liga_rx=1;
}	
//=================================================================================
void play_voz (unsigned int voiceNumber)
{
	envia_comando_voz (voiceNumber);
		
}	
//=================================================================================
void stop_voz(void)
{
	envia_comando_voz (0xFFFF);	//0xFFFF é o que precisa ser enviado para parar a comunicação
}
//=================================================================================
void volume_voz(void)
{
	envia_comando_voz (volume);
}	
//=================================================================================
void envia_comando_voz (unsigned int comando_voz)
{
	unsigned int i;
	clk_voz = 0;
	Delay1KTCYx(1);						//datasheet 2ms, aqui mandamos 1950ms e os outros 50ms estão no for
	Delay10TCYx(25);
	Delay10TCYx(25);
	Delay10TCYx(25);
	Delay10TCYx(20);

/*

#define reset_voz	PORTDbits.RD0	//0ª saída digital		pino 1 do J1
#define clk_voz		PORTDbits.RD1	//1ª saída digital		pino 2 do J1
#define data_voz	PORTDbits.RD2	//2ª saída digital		pino 3 do J1
*/

	
	for(i=0b1000000000000000;i>0;i=i>>1)		//0x8000 é i=0b1000 0000 0000 0000 = 16 bits
	{										//i>>=1 é igual a i=i>>1 (rotação para a direita)
			clk_voz = 0;
			Delay10TCYx(4);
			if ((comando_voz & i)!=0)	//testa bit a bit os 16 que serão enviados, se o bit for 1
			{
				data_voz=1;			//manda nível alto no pino de dados
			}	
			else
			{
				data_voz=0;			//manda nível baixo no pino de dados
			}	
			Delay10TCYx(4);
			clk_voz = 1;
			Delay10TCYx(8);
	}

	Delay1KTCYx(1);						//datasheet 2ms, aqui mandamos 1950ms e os outros 50ms estão no for
	Delay10TCYx(25);
	Delay10TCYx(25);
	Delay10TCYx(20);
	Delay10TCYx(20);

}	
//=================================================================================
//
//=================================================================================

void Cabecalho(void)
{
		//Serial_Lcd_Out(1, 1, "     Temp:   C ");
		PosicionaLCD(1,1);
		StringLCD("Temp:    C      ");
		Delay10KTCYx(10);
		
		//Serial_Lcd_Out(2, 1, " Humidity:   % ");
		PosicionaLCD(2,1);
		StringLCD("RH:    %        ");
		Delay10KTCYx(10);
}


unsigned char ReadData()
{
	unsigned char i=0, j=0;
	
	for(j = 0; j < 8; j++)
	{
		TMR0L=0x00;
		TMR0H=0x00;		
		while(I2C_SDA==0); 		//Espera o pino subir
		T0CONbits.TMR0ON = 1;   //liga o timer
		while(I2C_SDA==1); 		//Espera o pino descer
		T0CONbits.TMR0ON = 0;   //desliga o timer
		if(TMR0L<=35)
		{
		   i&= ~(1<<(7 - j));  //Clear bit (7-b)
		}
		else 
		{
			i|= (1 << (7 - j)); //Set bit (7-b)
						
		}  
	}

	return i;
}



void Read_DHT11(void)
{
	//Start
	TRISB = 0b10000010;	//coloquei meu pino RB0 como saída
	I2C_SDA = 0;		//coloquei 0 na saida
	Delay1KTCYx(18);	//dei o tempo pedido no datasheet para start bit 18ms
	I2C_SDA = 1;		//voltei o pino para 1
	Delay10TCYx(3);
	TRISB = 0b10000011;	//coloquei meu pino como entrada
	
	//Check
	f_Check = 0;
	Delay10TCYx(4);
	if (I2C_SDA == 0)
	{
		Delay10TCYx(8);
		if (I2C_SDA == 1)
		{
		   f_Check = 1;
		   while(I2C_SDA==1);
		}
		
	
		if (f_Check == 1)
		{
			RH_byte1 = ReadData();					
			RH_byte2 = ReadData();		
			T_byte1 = ReadData();	
			T_byte2 = ReadData();		
			Sum = ReadData();
				
			if(Sum == ((RH_byte1+RH_byte2+T_byte1+T_byte2) & 0XFF))
			{
				Temperatura = T_byte1;
				RH = RH_byte1;
				
				Cabecalho();
				
				//Serial_LCD_Chr(1, 12, 48 + ((Temp / 10) % 10));
				PosicionaLCD(1,6);
				EscreveLCD(48 + ((Temperatura / 10) % 10));
				Delay10KTCYx(10);
				
				//Serial_LCD_Chr(1, 13, 48 + (Temp % 10));
				EscreveLCD(48 + (Temperatura % 10));
				Delay10KTCYx(10);
				
				//Serial_LCD_Chr(2, 12, 48 + ((RH / 10) % 10));
				PosicionaLCD(2,4);
				EscreveLCD(48 + ((RH / 10) % 10));
				Delay10KTCYx(10);
				
				//Serial_LCD_Chr(2, 13, 48 + (RH % 10));
				EscreveLCD(48 + (RH % 10));
				Delay10KTCYx(10);
				
			}
			else
			{
				LimpaLCD();             // clear LCD
				PosicionaLCD(1,1);
				StringLCD("Check sum error ");
			}
		}
	}
	else 
	{
		PosicionaLCD(1,1);
		StringLCD("   No response  ");
		PosicionaLCD(2,1);
		StringLCD("from the sensor ");
	}
	Delay10KTCYx(100);
}
void posicao_LCD(void)
{
	if (f_linha1==0)
	{
		linha =1;
		f_linha1=1;
	}	
	else if (f_linha2==0)
	{
		linha =2;
		f_linha2=1;
	}	
	else if (f_linha3==0)
	{
		linha =3;
		f_linha3=1;
	}	
	else if (f_linha4==0)
	{
		linha =4;
		f_linha4=1;
	}
	else 
	{
		BUZZER=1;
		delay_ms_250(50);	
		BUZZER=0;
		delay_ms_250(50);
		BUZZER=1;
		delay_ms_250(50);	
		BUZZER=0;
		delay_ms_250(50);
		BUZZER=1;
		delay_ms_250(50);	
		BUZZER=0;
		delay_ms_250(50);
	}		
}	
//=================================================================================
//				PROGRAMA PRINCIPAL
//=================================================================================
void main (void)
{	
// Configurações Iniciais
	
//********************************************************************
//Configuração dos pinos
//********************************************************************
	//AD Essa configuração de AD é comum para todos os pinos de AD
	ADCON1 = 0b00001110;

	//Port A, do RA0 até RA5
	TRISA =  0b110001;

	//Port B, do RB0 até RB7
	TRISB = 0b10000011;		//configuro se o pino é entrada ou saida
							// 1 = IN
							// 0 = OUT

	INTCON2bits.RBPU = 1;	// 0 = ligou os PULL UP's
							// 1 = desligou os PULL UP's
	
	//Port C, do RC0 até RC7 menos o RC3 que foi dedicado para a USB
	TRISC = 0b11111111;
	UCONbits.USBEN = 0;		//Configuração necessária para deixar os pinos RC4 e RC5
	UCFGbits.UTRDIS = 1;	//como entradas digitais, desabilitando a USB.
	
	//Port D, do RD0 até RD7
	TRISD = 0b00000000;

	//Port E, do RE0 até RE3 o RE3 é só entrada pois é o MCLR
	TRISE = 0b1111;
	
//********************************************************************
//Condição inicial de cada pino
//********************************************************************
	//Port A, do RA0 até RA5
	PORTA = 0b000000;	

	//Port B, do RB0 até RB7
	PORTB = 0b00000000;
	
	//Port C, do RC0 até RC7 menos o RC3 que foi dedicado para a USB
	PORTC = 0b00000000;
	
	//Port B, do RD0 até RD7
	PORTD = 0b00000000;

	//Port E, do RE0 até RE3 o RE3 é só entrada pois é o MCLR
	PORTE = 0b00000000;

//********************************************************************
//Configuração do AD
//********************************************************************
//C:\Program Files\Microchip\mplabc18\v3.47\doc
//	OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_8_TAD, ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS, ADC_1ANA);
	
//********************************************************************
//Configuração da Serial (USART)
//********************************************************************
//C:\Program Files\Microchip\mplabc18\v3.47\doc

	BAUDCON = 0x00;		// 
 	OpenUSART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH & USART_ADDEN_OFF, 25);
    
    PIR1bits.TXIF = 0;		//Limpa flag de transmissão
    PIE1bits.TXIE = 0;		//Desabilita a interrupção de transmissão
    IPR1bits.TXIP = 0;		//Baixa prioridade
    
    PIR1bits.RCIF = 0;		//Limpa flag de recepção
    PIE1bits.RCIE = 1;		//Habilita a interrupção de recepção
    IPR1bits.RCIP = 1;		//Alta prioridade
    
//********************************************************************
//Configuração dos Timers
//********************************************************************
//Tempo do estouro do timer = t cliclo de máquina * (carga do registrador)*prescaler
//no caso, (carga do registrador)=65536-500 poi sé 16 bits
//Timer0//
OpenTimer0(TIMER_INT_ON&T0_16BIT&T0_SOURCE_INT&T0_EDGE_FALL&T0_PS_1_2);
WriteTimer0(CONST_T_TIMER0);
//T0CONbits.TMR0ON = 1;    //bit que liga e desliga o timer
	
//Timer1//
//OpenTimer1(TIMER_INT_ON&T1_16BIT_RW&T1_SOURCE_INT&T1_PS_1_2&T1_OSC1EN_OFF&T1_SYNC_EXT_OFF);
//WriteTimer1(CONST_T_TIMER1);
//Timer2//    

//Timer3//
    
    
    
//********************************************************************
//		Configuração das Interrupções
//********************************************************************
    
    RCONbits.IPEN = 0;		//Definindo para o PIC que todas as interrupçãoe são de alta prioridade
    INTCONbits.GIE = 1;		//habilita a interrupção global
    INTCONbits.PEIE = 1;	//habilita as interrupções de periférico

//********************************************************************
//		Inicio do programa
//********************************************************************
	IniciaLCD ();
	LimpaLCD();

	f_chegou_dado=0;
	
	k=0x00;
	
//=================================================================================
//		CABEÇALHO
//=================================================================================
	f_tamanho_palavra=0;
	
	clk_voz=1;
	reset_voz=1;
	f_play=0;
	reset_module_voz();
	
	f_linha1=0;
	f_linha2=0;
	f_linha3=0;
	f_linha4=0;
	
	f_inicializando=0;
	f_on_off=0;
	f_a=0;
	f_c=0;
	f_d=0;
	f_e=0;
	f_carta1=0;
	f_carta2=0;
	f_carta3=0;
	f_carta4=0;
	f_cartao_invalido=0;
	f_checa_dado=0;
	zera_dado();
	stop_voz();
	Liga_rx=1;
//=================================================================================
//		BUZZER INDICADOR DE INICIALIZAÇÃO + LCD	
//=================================================================================
	for (k=0;k<3;++k)
	{	
		BUZZER=1;
		delay_ms_250(50);
		delay_ms_250(50);
		delay_ms_250(50);
		delay_ms_250(50);
		BUZZER=0;
		delay_ms_250(50);
	}

	PosicionaLCD (1,1);
	StringLCD (" SMART BUS STOP ");
	PosicionaLCD (2,1);
	StringLCD ("                                 ");

//=================================================================================
//		INICIALIZAÇÃO
//=================================================================================
	while (f_on_off == 0)
	{
		if(Botao_S1 == 0)
		{
			while(Botao_S1 == 0);	
			f_on_off = 1;
		}			
		if(Botao_S3 == 0)
		{
			while(Botao_S3 == 0);
			SubTeste();
		}
	}
	PosicionaLCD (1,1);
	StringLCD ("inside on/off   ");
	PosicionaLCD (2,1);
	StringLCD ("                         ");
//=================================================================================
//		IDENTIFICAÇÃO E REPRODUÇÃO DA LINHA
//=================================================================================
	while(f_on_off == 1)	
{
		Armazena_Carta();
		Checa_dado();
		
		if(Botao_S2 == 0)
		{
			while(Botao_S2 == 0);
			f_on_off = 0;
		}
		if(Botao_S3 == 0)
		{
			while(Botao_S3 == 0);
			SubTeste();
		}
		if (f_carta1==1)
		{
			f_carta1=0;
			
			PosicionaLCD (1,1);
			StringLCD ("      I-02      ");
			PosicionaLCD (2,1);
			StringLCD ("                ");
			Delay10KTCYx(200);
			Liga_rx=0;
			reset_module_voz();
			play_voz(86);//I-02
			delay_ms_250(50);
			while(tocando==0);
			Liga_rx=1;
		}
		if (f_carta2==1)
		{
			f_carta2=0;
			
			PosicionaLCD (1,1);
			StringLCD ("      B-11      ");
			PosicionaLCD (2,1);
			StringLCD ("                ");			
			Delay10KTCYx(200);			
			Liga_rx=0;
			reset_module_voz();
			play_voz(68);//B-11
			delay_ms_250(50);
			while(tocando==0);
			Liga_rx=1;
		}
		if (f_carta3==1)
		{
			f_carta3=0;
			
			PosicionaLCD (1,1);
			StringLCD ("      I-05      ");
			PosicionaLCD (2,1);
			StringLCD ("                ");	
			Delay10KTCYx(200);
			
			Liga_rx=0;
			reset_module_voz();
			play_voz(93);//I-05
			delay_ms_250(50);
			while(tocando==0);
			Liga_rx=1;
		}
		if (f_carta4==1)
		{
			f_carta4=0;
			
			PosicionaLCD (1,1);
			StringLCD ("      T-17      ");
			PosicionaLCD (2,1);
			StringLCD ("                ");	
			Delay10KTCYx(200);
			
			Liga_rx=0;
			reset_module_voz();
			play_voz(112);//T-17
			delay_ms_250(50);
			while(tocando==0);
			Liga_rx=1;
		}
		if (f_cartao_invalido==1)
		{
			f_cartao_invalido=0;
			
			PosicionaLCD (1,1);
			StringLCD ("      I-04      ");
			PosicionaLCD (2,1);
			StringLCD ("                ");	
			Delay10KTCYx(200);
			
			Liga_rx=0;
			reset_module_voz();
			play_voz(91);//I-04
			delay_ms_250(50);
			while(tocando==0);
			Liga_rx=1;
		}
	}
}
