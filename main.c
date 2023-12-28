#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define F_CPU 8000000UL         /* Define CPU Frequency e.g. here 8MHz */
#include <avr/io.h> /* Include AVR std. library file */
#include <util/delay.h>         /* Include Delay header file */
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#define LCD_Dir  DDRC           /* Define LCD data port direction */
#define LCD_Port PORTC          /* Define LCD data port */
#define RS PC0                  /* Define Register Select pin */
#define EN PC1
#define Buzzer PB0                  /* Define Enable signal pin */
#define ADC_CHANNEL 4

#define MIN_SENSOR_VALUE 0
#define MAX_SENSOR_VALUE 1023
#define MIN_OUTPUT_VALUE 0
#define MAX_OUTPUT_VALUE 1000

void UART_init(long USART_BAUDRATE);
unsigned char UART_RxChar(void);
void UART_TxChar(char ch);
void UART_SendString(char *str);

void UART_init(long USART_BAUDRATE){
	UCSRB |= (1 << RXEN) | (1 << TXEN);/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
	UBRRL = BAUD_PRESCALE;		/* Load lower 8-bits of the baud rate value */
	UBRRH = (BAUD_PRESCALE >> 8);	/* Load upper 8-bits*/
}

unsigned char UART_RxChar(void){
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);		/* Return the byte */
}

void UART_TxChar(char ch){
	while (! (UCSRA & (1<<UDRE)));  /* Wait for empty transmit buffer */
	UDR = ch ;
}

void UART_SendString(char *str){
	unsigned char j=0;
	while (str[j]!=0){		/* Send string till null */
		UART_TxChar(str[j]);	
		j++;
	}
}

void ADC_Init()
{
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);    /* Set ADC prescaler to 64 for an 8MHz clock, which gives ADC clock of 125kHz */
    ADMUX |= (1 << REFS0);                    /* Set reference voltage to AVCC */
    ADCSRA |= (1 << ADEN);                    /* Enable ADC */
}

uint16_t ADC_Read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); /* Select the ADC channel */
    ADCSRA |= (1 << ADSC);                      /* Start ADC conversion */
    while (ADCSRA & (1 << ADSC));               /* Wait for ADC conversion to complete */
    return ADC;                                 /* Return the ADC result */
}

void LCD_Command(unsigned char cmnd)
{
    LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
    LCD_Port &= ~(1 << RS); /* RS=0, command reg. */
    LCD_Port |= (1 << EN); /* Enable pulse */
    _delay_us(1);
    LCD_Port &= ~(1 << EN);

    _delay_us(200);

    LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4); /* sending lower nibble */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);
    _delay_ms(2);
}

void LCD_Char(unsigned char data)
{
    LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
    LCD_Port |= (1 << RS); /* RS=1, data reg. */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);

    _delay_us(200);

    LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
    LCD_Port |= (1 << EN);
    _delay_us(1);
    LCD_Port &= ~(1 << EN);
    _delay_ms(2);
}
void LCD_Init(void) {
    LCD_Dir = 0xFF; /* Make LCD port direction as o/p */
    _delay_ms(20); /* LCD Power ON delay always >15ms */

    LCD_Command(0x02); /* send for 4-bit initialization of LCD */
    _delay_us(1);
    LCD_Command(0x28); /* 2 line, 5*7 matrix in 4-bit mode */
    _delay_us(1);
    LCD_Command(0x0C); /* Display on, cursor off */
    _delay_us(1);
    LCD_Command(0x06); /* Increment cursor (shift cursor to right) */
    _delay_us(1);
    LCD_Command(0x01); /* Clear display screen */
    _delay_ms(2);
}


void LCD_String(char *str) /* Send string to LCD function */
{
    int i;
    for (i = 0; str[i] != 0; i++) /* Send each char of string till the NULL */
    {
        LCD_Char(str[i]);
    }
}

void LCD_String_xy(char row, char pos, char *str) /* Send string to LCD with xy position */
{
    if (row == 0 && pos < 16)
        LCD_Command((pos & 0x0F) | 0x80); /* Command of first row and required position<16 */
    else if (row == 1 && pos < 16)
        LCD_Command((pos & 0x0F) | 0xC0); /* Command of first row and required position<16 */
    LCD_String(str); /* Call LCD string function */
}

void LCD_Clear()
{
    LCD_Command(0x01); /* Clear display */
    _delay_ms(2);
    LCD_Command(0x80); /* Cursor at home position */
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

void Buzzer_On()
{
    PORTB |= (1 << Buzzer); /* Turn ON the buzzer by setting the corresponding bit in PORTC */
}

void Buzzer_Off()
{
    PORTB &= ~(1 << Buzzer); /* Turn OFF the buzzer by clearing the corresponding bit in PORTC */
}


int main()
{
    char sensorValueStr[16];
    uint16_t sensorValue;
    uint16_t scaledValue;

    LCD_Init();            // Initialization of LCD */
    ADC_Init();            // Initialization of ADC */

	// Set Buzzer pin as output
    DDRB |= (1 << Buzzer);
	
    while (1)
    {
        sensorValue = ADC_Read(ADC_CHANNEL); // Read sensor value from ADC channel */

        // Scale the sensor value to the desired range
        scaledValue = map(sensorValue, MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, MIN_OUTPUT_VALUE, MAX_OUTPUT_VALUE);

        sprintf(sensorValueStr, "Sensor:%2d", scaledValue);
		
		//sprintf(sensorValueStr, "Sensor:%2d", scaledValue);
        LCD_Clear();
        LCD_String_xy(0, 0, sensorValueStr); // Display scaled sensor value on the LCD */
		//LCD_String_xy(1, 0, "Sending text");

if (scaledValue > 20) {
           // LCD_Clear();
            LCD_String_xy(1, 0, "Alert!");
			Buzzer_On();
			
			    _delay_ms(1000); // Delay for a while before reading again 
		 		UART_init(9600);
				UART_SendString("AT\r\n");
				_delay_ms(200);
				UART_SendString("ATE0\r\n");
				_delay_ms(200);
				UART_SendString("AT+CMGF=1\r\n");
				_delay_ms(200);
				UART_SendString("AT+CMGS=\"+94772809811\"\r\n");
				_delay_ms(200);
				UART_SendString("Danger! low quality air");
				UART_TxChar(26);
				
				_delay_ms(1);
        } else {
            LCD_String_xy(1, 0, "Okay");
			Buzzer_Off();

        }

    }
    return 0;
}
