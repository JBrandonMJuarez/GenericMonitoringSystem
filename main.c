#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*
 * Medidor de Consumo de Energía - PIC18F4550
 * Sensor: ACS712 20A en AN0
 * LCD I2C en RB0 (SDA), RB1 (SCL)
 * UART para transmisión de datos
 * Oscilador externo de 20 MHz
 * Autor: ChatGPT
*/

#pragma config FOSC = HSPLL_HS  // Oscilador externo de alta velocidad con PLL habilitado
#pragma config PLLDIV = 5       // (20 MHz entrada / 5 = 4 MHz para PLL)
#pragma config CPUDIV = OSC1_PLL2
#pragma config USBDIV = 2
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = ON
#pragma config BOR = OFF
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LVP = OFF
#pragma config PBADEN = OFF
#pragma config DEBUG = OFF
#define _XTAL_FREQ 48000000UL

#define N 240  // número de muestras
#define OFFSET_ADC 512  // corresponde a ~2.5V con Vref=5V
#define SENSIBILIDAD 0.1  // ejemplo: 100 mV/A para ACS712-20A

// ================= UART ==================
void UART_Init(unsigned long baud) {
    unsigned int x;
    x = (_XTAL_FREQ - baud*64)/(baud*64);
    if(x>255){
        x = (_XTAL_FREQ - baud*16)/(baud*16);
        BRGH = 1;
    } else {
        BRGH = 0;
    }
    SPBRG = x;
    SYNC = 0;
    SPEN = 1;
    TRISC6 = 1;
    TRISC7 = 1;
    TXEN = 1;
    CREN = 1;
}

void UART_Write(char data) {
    while(!TRMT);
    TXREG = data;
}

void UART_Write_Text(const char* text) {
    while(*text != '\0') {
        UART_Write(*text++);
    }
}

// ================= ADC ==================
void ADC_Init() {
    ADCON1 = 0x0E;  // AN0 analógico, resto digital
    ADCON2 = 0xAE; 
    ADCON0 = 0x01;  // Canal 0 (AN0), ADC on
}

uint16_t ADC_Read(uint8_t ch) {
    ADCON0 = (ADCON0 & 0xC5) | (ch << 2);
    __delay_us(20);
    GO_nDONE = 1;
    while(GO_nDONE);
    return ((ADRESH << 8) + ADRESL);
}

// ================= I2C ==================
void I2C_Init(void) {
    SSPCON1 = 0x28;
    SSPCON2 = 0x00;
    SSPADD = (_XTAL_FREQ/(4*100000))-1;
    SSPSTAT = 0;
    TRISB0 = 1; // SDA
    TRISB1 = 1; // SCL
}

void I2C_Wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));
}

void I2C_Start(void) {
    I2C_Wait();
    SEN = 1;
}

void I2C_Stop(void) {
    I2C_Wait();
    PEN = 1;
}

void I2C_Write(uint8_t data) {
    I2C_Wait();
    SSPBUF = data;
}

// ================= LCD I2C ==================
#define LCD_ADDR 0x40

void LCD_Cmd(uint8_t cmd) {
    uint8_t hi = cmd & 0xF0;
    uint8_t lo = (cmd << 4) & 0xF0;
    I2C_Start();
    I2C_Write(LCD_ADDR);
    I2C_Write(hi | 0x0C);
    I2C_Write(hi | 0x08);
    I2C_Write(lo | 0x0C);
    I2C_Write(lo | 0x08);
    I2C_Stop();
    __delay_ms(2);
}

void LCD_Char(char data) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    I2C_Start();
    I2C_Write(LCD_ADDR);
    I2C_Write(hi | 0x0D);
    I2C_Write(hi | 0x09);
    I2C_Write(lo | 0x0D);
    I2C_Write(lo | 0x09);
    I2C_Stop();
    __delay_us(50);
}

void LCD_Init() {
    __delay_ms(50);
    LCD_Cmd(0x33);
    LCD_Cmd(0x32);
    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x06);
    LCD_Cmd(0x01);
    __delay_ms(2);
}

void LCD_Text(const char* txt) {
    while(*txt) LCD_Char(*txt++);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 1) ? 0x00 : 0x40;
    LCD_Cmd(0x80 | (pos + col));
}

// ================= MAIN ==================
void main(void) {
    ADC_Init();
    UART_Init(9600);
    I2C_Init();
    LCD_Init();

    char buffer[20];
    uint16_t adc;
    float voltage, current;

    while(1) {
        //adc = ADC_Read(0);
        //voltage = (adc * 5.0) / 1023.0;
        //current = (voltage - 2.5) / 0.1;
	
	float suma = 0;
	for (int i = 0; i < N; i++) {
	 int lectura = ADC_Read(0); // lectura del ADC
	 float volt = lectura - OFFSET_ADC;
	 suma += volt * volt;
	}
	float promedio = suma / N;
	float corriente_rms = sqrt(promedio) * (5.0 / 1023.0) / SENSIBILIDAD;

        // LCD
        LCD_SetCursor(1, 0);
        LCD_Text("I=");
        sprintf(buffer, "%.2fA   ", corriente_rms);
        LCD_Text(buffer);

        // UART
        sprintf(buffer, "\r\nCorriente: %.2fA\r\n", corriente_rms);
        UART_Write_Text(buffer);

        __delay_ms(500);
    }
}

