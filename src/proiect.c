#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "ssd1306.h"
#include "pff.h"
#include "adc.h"

#define PM_BAUD 9600
#define DEBOUNCE_TIME_MS 400

volatile uint8_t next_debounce_ms_counter = 0;
volatile uint8_t play_debounce_ms_counter = 0;
volatile uint8_t stop_debounce_ms_counter = 0;

int next_state = 0;
int play_state = 0;
int stop_state = 0;

volatile uint8_t playing = 0;
volatile uint16_t value = 0;

volatile uint8_t hours = 0;
volatile uint8_t minutes = 0;
volatile uint8_t seconds = 0;

volatile uint32_t next_mill = 0;
volatile uint32_t play_mill = 0;
volatile uint32_t stop_mill = 0;

FATFS fs;					// sistemul de fisiere

/*---------------------------------------------------------------------------*/
/* Player audio                                                              */
/*---------------------------------------------------------------------------*/

/*
 * Four-Character Code - folosit pentru a indentifica formate de date
 */
#define FCC(c1, c2, c3, c4) \
	(((DWORD)(c4) << 24) + \
	 ((DWORD)(c3) << 16) + \
	 (( WORD)(c2) <<  8) + \
	 (( BYTE)(c1) <<  0))


uint8_t	buf[2][256];	// wave output buffers (double buffering)
const	 uint16_t	buf_size = 256;	// front and back buffer sizes
volatile uint8_t	buf_front = 0;	// front buffer index (current buffer used)
volatile uint8_t	buf_pos = 0;	// current buffer position
volatile uint8_t	buf_sync = 0;

volatile uint8_t next_pressed = 0;
volatile uint8_t play_pressed = 0;
volatile uint8_t stop_pressed = 0;

#define BUF_FRONT	(buf[buf_front])
#define BUF_BACK	(buf[1 - buf_front])

// ----------------------------------------
volatile uint32_t mills = 0;

uint32_t milliseconds() {
    return mills;
}

void my_delay(uint32_t time) {
    uint32_t start = milliseconds();
    uint32_t end = start + time;

    // wait for the specified amount of time
    while(end > milliseconds());

    return;
}

/*
	display message on screen
*/
void displayMessage(char* str) {
	SSD1306_ClearScreen (); // clear screen
	SSD1306_DrawLine (0, MAX_X, 4, 4); // draw line
	SSD1306_SetPosition (7, 1); // set position
	SSD1306_DrawString (str); // draw string
	SSD1306_DrawLine (0, MAX_X, 18, 18); // draw line
	SSD1306_UpdateScreen (SSD1306_ADDR); // update
	SSD1306_UpdateScreen (SSD1306_ADDR); // update
}

/*
	display title on screen
*/
void displayTitle() {
	SSD1306_ClearScreen (); // clear screen
	SSD1306_DrawLine (0, MAX_X, 4, 4); // draw line
	SSD1306_SetPosition (7, 1); // set position
	SSD1306_DrawString ("SOUND MIXER"); // draw string
	SSD1306_DrawLine (0, MAX_X, 18, 18); // draw line
	SSD1306_UpdateScreen (SSD1306_ADDR); // update

	my_delay (1000);
	SSD1306_InverseScreen (SSD1306_ADDR);

	my_delay (1000);
	SSD1306_NormalScreen (SSD1306_ADDR);

	SSD1306_ClearScreen ();
}

/*
	timer0 interrupt - used for advancing song bytes
*/
ISR(TIMER0_COMPA_vect)
{
	OCR1A = BUF_FRONT[buf_pos++];

	// swap buffers when end is reached (end is 256 <=> overflow to 0)
	if(buf_pos == 0)
		buf_front = 1 - buf_front;
}

/*
	timer0 config - used for songs
*/
void timer0_start(void)
{
	// interrupt on compare A
	TIMSK0 |= (1 << OCIE0A);
	// CTC, top OCRA
	TCCR0B |= (0 << WGM02);
	TCCR0A |= (1 << WGM01) | (0 << WGM00);
	// prescaler 8
	TCCR0B |= (2 << CS00);
}

/*
	timer0 stop - used when song stops playing
*/
void timer0_stop(void)
{
	TCCR0B = 0;
	TCCR0A = 0;
	TIMSK0 = 0;
	OCR0A = 0;
	TCNT0 = 0;
}

/*
	timer1 start - used for 8-bit PWM song output
*/
void timer1_start(void)
{
	// 8-bit FastPWM
	TCCR1B |= (1 << WGM12);
	TCCR1A |= (1 << WGM10);
	// channel A inverted
	TCCR1A |= (1 << COM1A0) | (1 << COM1A1);
	// prescaler 1
	TCCR1B |= (1 << CS10);
}

void timer1_stop(void)
{
	TCCR1B = 0;
	TCCR1A = 0;
	OCR1A = 0;
	TCNT1 = 0;
}

/*
	verifica daca se cere oprirea melodiei (prin apasarea STOP)
*/
bool continue_play()
{
	if (stop_pressed) {
		stop_pressed = 0;
		return false;
	}

	return true;
}

/*
 * Incarca header-ul unui fisier WAVE
 *
 * @return DWORD
 * 	0 => format invalid
 * 	1 => eroare I/O
 * 	>1 => numarul de sample-uri
 */
DWORD load_header(void)
{
	DWORD size;
	WORD ret;

	// citeste header-ul (12 octeti)
	if(pf_read(BUF_FRONT, 12, &ret))
		return 1;

	if(ret != 12 || LD_DWORD(BUF_FRONT + 8) != FCC('W','A','V','E'))
		return 0;

	for(;;)
	{
		// citeste chunk ID si size
		pf_read(BUF_FRONT, 8, &ret);
		if(ret != 8)
			return 0;

		size = LD_DWORD(&BUF_FRONT[4]);

		// verifica FCC
		switch(LD_DWORD(&BUF_FRONT[0]))
		{
			// 'fmt ' chunk
			case FCC('f','m','t',' '):
				// verifica size
				if(size > 100 || size < 16) return 0;

				// citeste continutul
				pf_read(BUF_FRONT, size, &ret);
				// verifica codificarea
				if(ret != size || BUF_FRONT[0] != 1) return 0;
				// verifica numarul de canale
				if(BUF_FRONT[2] != 1 && BUF_FRONT[2] != 2) return 0;
				// verifica rezolutia
				if(BUF_FRONT[14] != 8 && BUF_FRONT[14] != 16) return 0;

				// seteaza sampling rate-ul
				OCR0A = (BYTE)(F_CPU / (8 + value) / LD_WORD(&BUF_FRONT[4])) - 1;
				break;

			// 'data' chunk => incepe redarea
			case FCC('d','a','t','a'):
				return size;

			// 'LIST' chunk => skip
			case FCC('L','I','S','T'):
			// 'fact' chunk => skip
			case FCC('f','a','c','t'):
				pf_lseek(fs.fptr + size);
				break;

			// chunk necunoscut => eroare
			default:
				return 0;
		}
	}

	return 0;
}

/*
 * Functie care reda un fisier audio
 *
 * path - calea absoluta a fisierului
 *
 * @return UINT
 *	FR_OK daca a rulat cu succes fisierul
 */
UINT play(const char *path)
{
	FRESULT ret;

	if((ret = pf_open(path)) == FR_OK)
	{
		WORD bytes_read;

		// incarca header-ul fisierului
		DWORD current_size = load_header();
		if(current_size < buf_size)
			return FR_NO_FILE;

		// align to sector boundary
		ret = pf_lseek((fs.fptr + 511) & ~511);
		if(ret != FR_OK)
			return ret;

		// fill front buffer
		ret = pf_read(BUF_FRONT, buf_size, &bytes_read);
		if(ret != FR_OK)
			return ret;
		if(bytes_read < buf_size)
			return ret;

		// reset front buffer index
		buf_pos = 0;

		// start output
		timer0_start();
		timer1_start();
		DDRB |= (1 << PB1);

		// set playing state
		playing = 1;

		// reset time length of played song
		hours = 0;
		minutes = 0;
		seconds = 0;

		while(continue_play())
		{
			printf("Playing...\n");
			uint8_t old_buf_front = buf_front;
			
			// fill back buffer
			ret = pf_read(BUF_BACK, buf_size, &bytes_read);
			if(ret != FR_OK)
				break;
			if(bytes_read < buf_size)
				break;

			// wait for buffer swap
			while(old_buf_front == buf_front) ;
		}

		// stop output
		DDRB &= ~(1 << PB1);
		timer1_stop();
		timer0_stop();
	}

	playing = 0;
	return ret;
}


/*---------------------------------------------------------------------------*/
/* Ceas                                                                      */
/*---------------------------------------------------------------------------*/

ISR(TIMER2_COMPA_vect)
{	
	/* increment milliseconds used for debouncing and delay */
	++mills;

	static uint16_t miliseconds = 0;

	miliseconds += 1;

	/* change number of milliseconds for one second based on current song speed */
	uint16_t val = 500 + (value + 1) * 500 / 8;
	if (val > 500) {
		val = 500;
	}

	if(miliseconds < val)
		return;
	miliseconds = 0;

	if(++seconds == 60)
	{
		seconds = 0;

		if(++minutes == 60)
		{
			minutes = 0;

			if(++hours == 24)
				hours = 0;
		}
	}

	/* if a song is currently playing, display time */
	if (playing == 1) {
		char buf[9];
		snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hours, minutes, seconds);
		displayMessage(buf);
	}
}

void timer2_init(void)
{
	// set compare at each milisecond
	OCR2A = 124;
	// interrupt on compare A
	TIMSK2 |= (1 << OCIE2A);
	// CTC, top OCRA
	TCCR2A |= (1 << WGM21);
	// prescaler 128
	TCCR2B |= (1 << CS20) | (1 << CS22);
}

DIR directory;
FILINFO file;

void init_directory() {
	/* open `/music`, using the `directory` structure declared above */
	pf_opendir(&directory, "/music");

	/* find the first file whose name doesn't start with a `_`; use the `directory` and `file` structures declared above */
	do
	{
		pf_readdir (&directory, &file);
	} while (file.fname[0] == '_');

	/* display the name of the first file on the LCD */
	displayMessage(file.fname);
	printf("%s\n", file.fname);
}

void next_file() {
	/* go to the next file whose name doesn't start with a `_`; after the last file, go back to the first file */
	do{
		pf_readdir(&directory, &file);
		if( strcmp (file.fname, "") == 0)
		init_directory();
	} while (file.fname[0] == '_');
	
	/* display the file name on the LCD */
	displayMessage(file.fname);
	printf("%s\n", file.fname);
}

void buttons_init() {
	/* configure button PD2 */
	DDRD &= ~(1 << PD2); // set as INPUT
	PORTD |= (1 << PD2); // enable pull-up resistor

	/* configure button PD3 */
	DDRD &= ~(1 << PD3);
	PORTD |= (1 << PD3);

	DDRD &= ~(1 << PD4);
	PORTD |= (1 << PD4);
}

void interrupt_config(){
	cli();

	/* enable pin change interrupt for PD2 and PD3 - buttons */
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20);

	sei();
}

/* interrupt handle for buttons */
ISR(PCINT2_vect) {
	if ((PIND & (1 << PD2)) == 0) {
		next_pressed = 1;
	}

	if ((PIND & (1 << PD3)) == 0){
		play_pressed = 1;
	}

	if ((PIND & (1 << PD4)) == 0){
		stop_pressed = 1;
	}
}

int main (void)
{
	char path[200];

	/* init USART */
	USART0_init(CALC_USART_UBRR(PM_BAUD));
  	USART0_use_stdio();
	/* config interrupts */
	interrupt_config();
	/* set button pins as input */
	buttons_init();
	/* init timer 2 (counts milliseconds) */
	timer2_init();
	/* init I2C display */
	SSD1306_Init (SSD1306_ADDR); // 0x3C
	/* init ADC */
	adc_init();

	/* display splash screen */
	displayTitle();

	/* enable interrupts */
	sei();

	for(;;)
	{
		displayMessage("MOUNTING...");

		// mount filesystem
		int mount = pf_mount(&fs);
		if(mount != FR_OK)
		{
			// wait a while and retry
			my_delay(1000);
			continue;
		}

		my_delay(2000);
		displayMessage("MOUNTED");
		printf("Mounted!\n");
		my_delay(2000);

		init_directory();

		for(;;)
		{
			uint16_t new_value = myAnalogRead(0);
			new_value = new_value >> 7;
			if (value != new_value) {
				value = new_value;
				printf("ADC: %d\n", value);
				char buf[12];
				snprintf(buf, sizeof(buf), "SPEED: %d", value);
				displayMessage(buf);
			}

			if (next_pressed) {
				if (mills - next_mill > DEBOUNCE_TIME_MS) {
					next_file();
				}
				
				next_mill = milliseconds();
				next_pressed = 0;
			}

			/* play current song */
			if (play_pressed) {
				if (mills - play_mill > DEBOUNCE_TIME_MS) {
					play_pressed = 0;
					sprintf(path, "/music/%s", file.fname);
					printf(path);
					play(path);
				}
				
				play_mill = milliseconds();
				play_pressed = 0;
			}

			if (stop_pressed) {
				stop_pressed = 0;
			}
		}
	}

	return 0;
}
