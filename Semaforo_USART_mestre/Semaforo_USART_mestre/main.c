/*
 * Semaforo_USART_mestre.c
 *
 * Created: 23/09/2021 15:05:31
 * Author : ruana
 */ 

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Definições e inclusão de bibliotecas

#define F_CPU 16000000UL		//Frequência de trabalho da CPU
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "PCD8544\nokia5110.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Prototipos das funções

void semaforo();
void liga_amarelo();
void liga_verde();
void liga_vermelho();
void anima_LCD(uint8_t t_vermelho, uint8_t t_amarelo, uint8_t t_verde);
void IA_carros();
void leitura_LDR();
void USART_Transmit(unsigned char data);
unsigned char USART_Receive();

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Variaveis globais

uint8_t t_vermelho = 1, t_amarelo = 1, t_verde = 1;
unsigned char t_0[2], t_1[2], t_2[2], freq[4], lux_LDR[4];
int estado = 0;
int cont_vermelho = 1, cont_verde = 1;
uint32_t tempo_ms = 0, tempo_freq_ms = 0, tempo_LUX_ms = 0;
uint32_t tempo_ms_anterior = 0, tempo_freq_anterior_ms = 0;
uint32_t freq_automatico = 0;
int flag = 1;
int conta_carro = 0, cont_freq = 0;

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Interrupções

ISR(INT0_vect) // Interrupção da porta PD2 que representa a entrada do clock, que simula os carros
{
	conta_carro++;
	tempo_freq_ms++;
	IA_carros();
}

ISR(PCINT2_vect) // Interrupção das portas PD4, PD5 e PD6 que representa os botoes de seleção
{
	// Botao de seleção de estado
	if(!(PIND & (1<<6)))
	{
		estado ++;
		
		// Estrategia para atualizar o LCD caso algo mude
		if (estado == 0)
		{
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		if (estado == 1)
		{
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		if (estado == 2)
		{
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		if (estado == 3)
		{
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
	}
	
	if (estado > 3)
	{
		estado = 0;
		anima_LCD(t_vermelho, t_amarelo, t_verde);
	}
	
	
	// Botão de mudança de tempo para mais
	if(!(PIND & (1<<4)))
	{
		if (estado == 0)	// Controla o tempo do vermelho para mais
		{
			if (t_vermelho >= 9)
			{
				t_vermelho = 9;
				}else {
				t_vermelho ++;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 1)	// Controla o tempo do amarelo para mais
		{
			if (t_amarelo >= 9)
			{
				t_amarelo = 9;
				}else {
				t_amarelo ++;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 2)	// Controla o tempo do verde para mais
		{
			if (t_verde >= 9)
			{
				t_verde = 9;
				}else {
				t_verde ++;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 3)	// Controla o tempo das cores caso esteja no modo automatico
		{
			cont_freq = 1;
			t_verde = (freq_automatico/60) + 1;
			t_vermelho = 10 - t_verde;
			t_amarelo = 1;
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
	}
	
	// Botão de mudança de tempo para menos
	if(!(PIND & (1<<5)))
	{
		if (estado == 0)	// Controla o tempo do vermelho para menos
		{
			if (t_vermelho <= 1)
			{
				t_vermelho = 1;
				}else {
				t_vermelho --;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 1)	// Controla o tempo do amarelo para menos
		{
			if (t_amarelo <= 1)
			{
				t_amarelo = 1;
				}else {
				t_amarelo --;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 2)	// Controla o tempo do verde para menos
		{
			if (t_verde <= 1)
			{
				t_verde = 1;
				}else {
				t_verde --;
			}
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
		
		if (estado == 3)	// Controla o tempo das cores caso esteja no modo automatico
		{
			cont_freq = 0;
			t_verde = (freq_automatico/60) + 1;
			t_vermelho = 10 - t_verde;
			t_amarelo = 1;
			
			anima_LCD(t_vermelho, t_amarelo, t_verde);
		}
	}
}

ISR(TIMER0_COMPA_vect)	// Interrupção do TC0 a cada 1 ms = (64 * (249 + 1)) / 16 MHz
{
	tempo_ms++;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Função main

int main(void)
{
	// GPIO
	DDRB	= 0b11111111;							// Habilita os pinos PB0...7 como saida
	DDRD	= ~((1<<2)|(1<<4)|(1<<5)|(1<<6));		// Habilita os  pinos  PD2, PD4, PD5 e PD6 como entrada
	PORTD  |= ((1<<2)|(1<<4)|(1<<5)|(1<<6));		// Habilita os pull-up de PD2, PD4, PD5 e PD6
	DDRC	= ~((1<<6)|(1<<0));						// Habilita os pinos PC0 e PC6 como entrada
	PORTC  |= (1<<6);								// Habilita o pull-up de PC6

	// Configuração das interrupções
	EICRA  = 0b00000010;		// Interrrupçãp externa INT0 na borda de descida
	EIMSK  = 0b00000001;		// Habilita a interrupção externa INT0
	PCICR  = 0b00000100;		// Habilita a interrupção externa PCINT2
	PCMSK2 = 0b01110000;		// Habilita individualmente as portas PD4, PD5 e PD6
	PCIFR  = 0b00000100;		// Indica se alguma interrupção ocorreu em PCINT2
	
	// Configuração do ADC
	ADMUX  = 0b01000000;		// Vcc como referência, canal 0
	ADCSRA = 0b11100111;		// Habilita o AD, modo de conversão continua, prescaler = 128
	ADCSRB = 0b00000000;		// Modo de conversão continua
	DIDR0  = 0b00111110;		// Habilita apenas o pino PC0 como entrada do ADC0
	
	// Configuração do PWM
	//Fast PWM, OC2A e OC2B habilitados
	TCCR2A = 0b00100011;	// PWM não invertido no pino OC2B
	TCCR2B = 0b00000100;	// Liga TC2, prescaler = 64, fpwm =	f0sc/(256*prescaler) = 16MHz/(256*64) = 976Hz
	OCR2B = 256;
	
	// Configuração  da USART
	UBRR0H = (unsigned char)(MYUBRR>>8);				// Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);			// Habilita o transmissor e o receptor e a interrupção de recepção da USART
	UCSR0C = (3<<UCSZ00);								// Ajusta o formato do frame: 8 bits de dados e 1 de parada
	
	// Interrupções do timer
	TCCR0A = 0b00000010;		// Habilita modo Ctc do TC0
	TCCR0B = 0b00000011;		// Liga TC0 com prescaler = 64
	OCR0A  = 249;				// Ajusta o comparador para o TC0 contar até 249
	TIMSK0 = 0b00000010;		// Habilita a interrupção na igualdade de comparação com OCR0A.
	// A interrupção ocorre a cada 1 ms = (64 * (249 + 1)) / 16 MHz
	sei();						// Habilita interrupções globais, ativando o bit I do SREG
	
	// Configuração do LCD
	nokia_lcd_init();
	anima_LCD(t_vermelho, t_amarelo, t_verde);
	
	while (1)
	{
		semaforo();
		IA_carros();
		leitura_LDR();
	}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

// Implementação das funções

void semaforo() // Função geral do semaforo
{
	liga_amarelo();
	liga_vermelho();
	liga_verde();
	
}

void liga_vermelho() // Função para habilitação da cor vermelha
{
	if(flag == 1){
		PORTB = 0b11110000; //Pinos PB4...7 em nivel alto
		USART_Transmit(6);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= 250*t_vermelho && flag == 2){
		PORTB = 0b01110000; //Pinos PB4...6 em nivel alto
		USART_Transmit(7);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= 250*t_vermelho && flag == 3){
		PORTB = 0b00110000; //Pinos PB4...5 em nivel alto
		USART_Transmit(7);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= 250*t_vermelho && flag == 4){
		PORTB = 0b00010000; //Pinos PB4     em nivel alto
		USART_Transmit(8);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= 250*t_vermelho && flag == 5){
		PORTB = 0b00000000; //Pinos PB4...7 em nivel baixo
		USART_Transmit(9);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	
}

void liga_amarelo() // Função para habilitação da cor amarela
{
	if(flag == 11){
		PORTD  |= (1<<7);  //Pinos PD7 em nivel alto
		USART_Transmit(5);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= t_amarelo*1000 && flag == 12){
		PORTD  &= ~(1<<7);  //Pinos PB7 em nivel baixo;
		//USART_Transmit(6);
		tempo_ms_anterior = tempo_ms;
		flag = 1;
	}
}

void liga_verde() // Função para habilitação da cor verde
{
	if(flag == 6){
		PORTB = 0b00001111; //Pinos PB0...3 em nivel alto
		USART_Transmit(0);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= t_verde*250 && flag == 7){
		PORTB = 0b00000111; //Pinos PB0...2 em nivel alto
		USART_Transmit(1);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= t_verde*250 && flag == 8){
		PORTB = 0b00000011; //Pinos PB0...1 em nivel alto
		USART_Transmit(2);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= t_verde*250 && flag == 9){
		PORTB = 0b00000001; //Pinos PB0     em nivel alto
		USART_Transmit(3);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	if((tempo_ms - tempo_ms_anterior) >= t_verde*250 && flag == 10){
		PORTB = 0b00000000; //Pinos PB0...3 em nivel baixo
		USART_Transmit(4);
		tempo_ms_anterior = tempo_ms;
		flag ++;
	}
	
}

void anima_LCD(uint8_t t_vermelho, uint8_t t_amarelo, uint8_t t_verde) // Função para animação do Nokia LCD PCD8544
{
	sprintf(t_0, "%u", t_vermelho);
	sprintf(t_1, "%u", t_amarelo);
	sprintf(t_2, "%u", t_verde);
	sprintf(freq, "%u", freq_automatico);
	sprintf(lux_LDR, "%u", (1023000/ADC) - 1000);
	
	nokia_lcd_clear();
	
	nokia_lcd_set_cursor(48, 0);
	nokia_lcd_write_string("|", 2);
	nokia_lcd_set_cursor(48, 10);
	nokia_lcd_write_string("|", 2);
	nokia_lcd_set_cursor(48, 20);
	nokia_lcd_write_string("|", 2);
	nokia_lcd_set_cursor(48, 30);
	nokia_lcd_write_string("|", 2);
	nokia_lcd_set_cursor(48, 40);
	nokia_lcd_write_string("|", 2);
	
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("T. Vm:", 1);
	nokia_lcd_set_cursor(37, 0);
	nokia_lcd_write_string(t_0, 1);
	
	nokia_lcd_set_cursor(0, 13);
	nokia_lcd_write_string("T. Am:", 1);
	nokia_lcd_set_cursor(37, 13);
	nokia_lcd_write_string(t_1, 1);
	
	nokia_lcd_set_cursor(0, 27);
	nokia_lcd_write_string("T. Vd:", 1);
	nokia_lcd_set_cursor(37, 27);
	nokia_lcd_write_string(t_2, 1);
	
	if(estado == 3 && cont_freq == 1)
	{
		nokia_lcd_set_cursor(0, 40);
		nokia_lcd_write_string("Modo A:", 1);
		
		}else {
		nokia_lcd_set_cursor(0, 40);
		nokia_lcd_write_string("Modo M:", 1);
		
	}
	
	nokia_lcd_set_cursor(62, 8);
	nokia_lcd_write_string(lux_LDR, 1);
	nokia_lcd_set_cursor(62, 18);
	nokia_lcd_write_string("LUX", 1);
	
	nokia_lcd_set_cursor(62, 30);
	nokia_lcd_write_string(freq, 1);
	nokia_lcd_set_cursor(62, 38);
	nokia_lcd_write_string("c/m", 1);
	
	nokia_lcd_set_cursor(45, 0 + estado * 13);
	nokia_lcd_write_string("<", 1);
	
	nokia_lcd_render();
}

void IA_carros() // Função que faz a "contagem" dos carros
{
	if((tempo_ms - tempo_freq_anterior_ms) >= 5000)
	{
		freq_automatico = conta_carro * 12;
		tempo_freq_anterior_ms = tempo_ms;
		conta_carro = 0;
		anima_LCD(t_vermelho, t_amarelo, t_verde);
	}
}

void leitura_LDR() // Função que faz a leitura do LDR a cada 500ms e aciona a lampada
{
	if((tempo_ms - tempo_LUX_ms) >= 500)
	{
		if((((1023000/ADC) - 1000) <= 300) && ((!(PINC & (1<<6))) || (freq_automatico > 0)))
		OCR2B = 250;
		
		else if((((1023000/ADC) - 1000) <= 300) &&  !(!(PINC & (1<<6)) || (freq_automatico > 0)))
		OCR2B = 80;
		
		else
		OCR2B = 1;
		
		tempo_LUX_ms = tempo_ms;
	}
	anima_LCD(t_vermelho, t_amarelo, t_verde);
}

void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0))); // Espera a limpeza do registrador de transmissão 
	UDR0 = data;					// Coloca o dado no registrador e o envia
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));	//Espera o dado ser recebido
	return UDR0;					//Lê o dado recebido e retorna
}

