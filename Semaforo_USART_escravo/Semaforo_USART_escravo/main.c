
/*
 * Semaforo_USART_escravo.c
 *
 * Created: 24/09/2021 16:53:33
 * Author : ruana
 */ 

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Defini��es e inclus�o de bibliotecas

#define F_CPU 16000000UL		//Frequ�ncia de trabalho da CPU
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Prototipos das fun��es

void trem();
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);
void pwm_servo(float porcentagem);

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Variaveis globais

int16_t liberado = 0, cont = 0, resultado;		// Variaveis da recep��o de dados
uint8_t t_vermelho = 1, t_amarelo = 1, t_verde = 1;
unsigned char t_0[2], t_1[2], t_2[2], freq[4], lux_LDR[4];
int estado = 0;
int cont_vermelho = 1, cont_verde = 1;
uint32_t tempo_ms = 0, tempo_freq_ms = 0, tempo_LUX_ms = 0;
uint32_t tempo_ms_anterior = 0, tempo_freq_anterior_ms = 0;
uint32_t freq_automatico = 0;
int flag = 0;
int conta_carro = 0, cont_freq = 0;

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Interrup��es

ISR(USART_RX_vect) // Interrup��o para a recep��o de dados
{
	char RX_dado;
	RX_dado = UDR0;
	
	if(!(PINC & (1<<6)))
	{
		PORTB = 0b11110000; //Pinos PB4...7 em nivel alto
	}else{
	switch(RX_dado)
	{
		// Ligando os leds vermelhos do sinal-escravo
		case 0:
			PORTD  &= ~(1<<7);  //Pinos PB7 em nivel baixo;
			PORTB = 0b11110000; //Pinos PB4...7 em nivel alto
			break;
		case 1:
			PORTB = 0b01110000; //Pinos PB4...6 em nivel alto
			break;
		case 2:
			PORTB = 0b00110000; //Pinos PB4...5 em nivel alto
			break;
		case 3:
			PORTB = 0b00010000; //Pinos PB4     em nivel alto
			estado =1;
			break;
		case 4:
			PORTB = 0b00010000; //Pinos PB4     em nivel alto
			break;
		case 5:
			PORTB = 0b00010000; //Pinos PB4     em nivel alto
			break;
		// Ligando os leds verdes do sinal-escravo
		case 6:
			PORTB = 0b00001111; //Pinos PB0...3 em nivel alto
			break;
		case 7:
			PORTB = 0b00000111; //Pinos PB0...2 em nivel alto
			break;
		case 8:
			PORTB = 0b00000001; //Pinos PB0     em nivel alto
			break;
		case 9:
			PORTB = 0b00000000; //Pinos PB0...3 em nivel baixo
			PORTD  |= (1<<7);  //Pinos PD7 em nivel alto
			break;
		default:
		
			break;
	}
	}
}

ISR(TIMER0_COMPA_vect)	// Interrup��o do TC0 a cada 1 ms = (64 * (249 + 1)) / 16 MHz
{
	tempo_ms++;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Fun��o main

int main(void)
{
	// GPIO
	DDRB	= 0b11111111;		// Habilita os pinos PB0...7 como saida
	DDRD	= ((1<<4)|(1<<2));			// Habilita o  pino  PD2, PD4 como saida
	PORTD  |= (1<<2);
	DDRC	= ~(1<<6);
	PORTC  |= (1<<6);
	
	// Configura��o  da USART
	UBRR0H = (unsigned char)(MYUBRR>>8);				// Ajusta a taxa de transmiss�o
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);			// Habilita o transmissor e o receptor e a interrup��o de recep��o da USART
	UCSR0C = (3<<UCSZ00);								// Ajusta o formato do frame: 8 bits de dados e 1 de parada
	
	// Configura��o do PWM
	//Fast PWM, OC2A e OC2B habilitados
	ICR1 = 39999;
	TCCR2A = 0b10100010;	// PWM n�o invertido no pino OC2B
	TCCR2B = 0b00011010;	// Orescaler = 8
	OCR2B = 2000;
	
	// Interrup��es do timer
	TCCR0A = 0b00000010;		// Habilita modo Ctc do TC0
	TCCR0B = 0b00000011;		// Liga TC0 com prescaler = 64
	OCR0A  = 249;				// Ajusta o comparador para o TC0 contar at� 249
	TIMSK0 = 0b00000010;		// Habilita a interrup��o na igualdade de compara��o com OCR0A.
	// A interrup��o ocorre a cada 1 ms = (64 * (249 + 1)) / 16 MHz
	sei();						// Habilita interrup��es globais, ativando o bit I do SREG
	
	while (1)
	{
		trem();
	}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Implementa��o das fun��es

void trem()
{
	if(!(PINC & (1<<6)))
	{
		PORTD  |= (1<<4);  //Pinos PD4 em nivel alto
		pwm_servo(7.5);
	}
	else
	{
		PORTD  &= ~(1<<4);  //Pinos PB7 em nivel baixo
		pwm_servo(5);
	}
}

void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0))); // Espera a limpeza do registrador de transmiss�o
	UDR0 = data;					// Coloca o dado no registrador e o envia
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));	//Espera o dado ser recebido
	return UDR0;					//L� o dado recebido e retorna
}

void pwm_servo(float porcentagem)
{
	float alto = (porcentagem/100)*20, baixo = 20 - alto;
	
	if((tempo_ms - tempo_ms_anterior) >= alto && flag == 0){
		PORTD  &= ~(1<<2);  //Pinos PB7 em nivel baixo
		flag ++;
		tempo_ms_anterior = tempo_ms;
	}
	
	if((tempo_ms - tempo_ms_anterior) >= baixo && flag == 1){
		PORTD  |= (1<<2);  //Pinos PD3 em nivel alto
		flag = 0;
		tempo_ms_anterior = tempo_ms;	
	}	
}

