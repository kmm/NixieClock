/*
 * Nixie Clock Firmware
 *
 * Created: 11/18/2012 11:57:31 PM
 *  Author: kmm
 */ 

// Internal 8MHz osc
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "clock.h"

volatile uint8_t g_ticks; // a tick is 1/100th of a second from timer1 compare interrupt A
volatile uint8_t g_cur_digit;
volatile uint8_t g_debounce_timer;
volatile uint8_t g_debounce_port;

uint32_t g_gps_time;
uint32_t g_sys_time;
uint8_t g_mode;

uint8_t g_rx_state, g_rx_ctr;
uint8_t g_rx_buf[RX_BUF_SZ];

uint8_t g_scan_order[] = {3,0,1,2}; // because why not?
int8_t g_settings[] = {MODE_TIME_24,         // SET_DISP_MODE  Time display mode
					   MODE_SYNC_GPS_UPDATE, // SET_SYNC_MODE  Time sync mode
					   DEFAULT_GMT_OFFSET,   // SET_GMT_OFFSET GMT offset (hours)
					   0,                    // SET_HOUR       manual hours (MODE_SYNC_GPS_OFF only)
					   0,                    // SET_MIN        manual minutes
					   0,                    // SET_SEC        manual seconds
					   DEFAULT_GPS_RATE		 // SET_GPS_RATE   GPS sync rate (MODE_SYNC_GPS_UPDATE only)
}; 

uint32_t bin_to_bcd(uint8_t bin) {
	// there are faster more bitshifty ways, but i'm lazy
	uint8_t hund = bin / 100;
	uint8_t tens = (bin - (hund * 100)) / 10;
	uint8_t ones = bin - (hund * 100) - (tens * 10);
	
	return hund << 8 | tens << 4 | ones;
}

/************************************************************************/
/* Init USART0 for 19200,N,8,1                                          */
/************************************************************************/
void init_uart() {
	UBRR0 = 25;
	UCSR0A = 0x00;
	UCSR0B = 0b10010000;
	UCSR0C = 0b00001110;
	UCSR0D = 0x00;
}

void set_clock_gps(uint32_t gps_tow) {
	g_gps_time = ((uint32_t)(gps_tow / 100.0) - DEFAULT_GPS_OFFSET) + (3600UL * g_settings[SET_GMT_OFFSET]);
}

/************************************************************************/
/* Grab stuff coming out of the GPS module.                             */
/* This module (Royaltek REB-21R with random firmware) spits out data   */
/* in SIRF binary format by default. The "measured navigation data"     */
/* message (id 0x02) has Week and TOW, so we can just grab it.          */
/* http://gpsd.googlecode.com/files/SiRF-Royaltek.pdf                   */  
/************************************************************************/
void handle_uart_rx() {
	cli();
	uint8_t rx_byte = UDR0;
	switch(g_rx_state) {
		case RX_IDLE:
			// SIRF binary messages start with 0xA0 0xA2 ...
			if(rx_byte == 0xA0) { g_rx_state = RX_START; }
			break;
		case RX_START:
			if(rx_byte == 0xA2) { g_rx_state = RX_READ; }
			break;
		case RX_READ:
			g_rx_buf[g_rx_ctr] = rx_byte;
			// Packet bytes 0:1 have the message length, read until we have that many bytes
			// or we run out of buffer. If we run out of buffer, just truncate.
			if(g_rx_ctr > 2 && g_rx_ctr == g_rx_buf[1]) { g_rx_state = RX_END;  }
			if(g_rx_ctr < RX_BUF_SZ) { g_rx_ctr++; } else { g_rx_state = RX_END; }
			break;
		case RX_END:
			if(g_rx_buf[2] == 0x02) {
				// We have a full buffer and it's a nav data packet! Yay!
				// GPS TOW is bytes 26:29 in the stream, unpack em and update the GPS time global
				set_clock_gps(((uint32_t)g_rx_buf[26] << 24) | ((uint32_t)g_rx_buf[27] << 16) | ((uint32_t)g_rx_buf[28] << 8) | g_rx_buf[29]);			
			}				
			g_rx_state = RX_IDLE;
			g_rx_ctr = 0;
			break;
		default:
			break;
	}
	sei();
}

void step_clock() {
	if(g_ticks >= 100) {
		g_ticks = 0;
		g_sys_time += 1;
	}
}

uint16_t render_time(uint32_t time, uint8_t flags) {
	uint8_t h, m, s;
	// don't really like all these mods and divs; there's almost certainly a faster way
	if(flags & _BV(FLAG_RENDER_12)) {
		h = ((time / 3600) % 12);
		if(h == 0) { h += 12; }
	}
	else {
		h = (time / 3600) % 24;
	}		
	m = (time / 60) % 60;
	s = time % 60;
	return (h / 10) << 12 | (h % 10) << 8 | (m / 10) << 4 | m % 10;
}

void display(uint8_t selected, uint16_t digits)
{
	uint32_t bits;
	uint16_t digit;
	uint8_t shift[] = {0, 2, 12, 22};
	
	digit = (digits & (0xF << (3 - selected) * 4)) >> (3 - selected) * 4;
	digit = digit == 0 ? 0x09 : --digit;
	if(selected == 0) {
		if(digit == 0x09) {
			bits = 0;
		}
		else {			
			bits = (1 << digit) & 0x3;
		}		
	}
	else {
		bits = (1 << digit) & 0x3FF;
	}
	PORTC &= ~(_BV(HV_OE));
	shift_hv(bits << shift[selected]);
	PORTC |= _BV(HV_OE);
}

void shift_hv(uint32_t data)
{
	for(uint8_t i = 0; i < 32; i++)
	{
		PORTC |= _BV(HV_CK);
		if(data << i & 0x80000000) {
			PORTB |= _BV(HV_DI);
		} 
		else {
			PORTB &= ~(_BV(HV_DI));
		}
		// 1.255 flicker glitch
		_delay_us(1.75);
		PORTC &= ~(_BV(HV_CK));
		_delay_us(1.75);
	}
}

ISR(USART0__RX_vect) {
	handle_uart_rx();
}

ISR(TIMER0_COMPA_vect) {
	switch(g_mode) {
		case OPR_MODE_RUN:
			display(g_scan_order[g_cur_digit++], render_time(g_gps_time, _BV(FLAG_RENDER_12)));
			//display(g_scan_order[g_cur_digit++], g_debounce_port);
			break;
		case OPR_MODE_CONF:
			//display_conf(g_scan_order[g_cur_digit++], (uint16_t g_conf_item << 8) | g_conf_val);
			break;
		default:
			g_mode = OPR_MODE_RUN;
			break;
	}	
	if(g_cur_digit > 3) { g_cur_digit = 0; }
}

ISR(TIMER1_COMPA_vect) {

}

ISR(PCINT0_vect) {
	uint8_t switches = ~((PINA & _BV(SW_DECR)) << 3) | ~((PINA & _BV(SW_INCR)) << 2) | ~((PINB & _BV(SW_MODE)) << 1) | ~(PINC & _BV(SW_ENTR));
	g_debounce_port = switches;
	
}

int main(void)
{
	DDRA = _BV(LED_RED) | _BV(LED_YEL);
	DDRB = _BV(HV_DI);
	DDRC = _BV(HV_CK) | _BV(HV_OE) | _BV(HV_ST);
	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = _BV(HV_ST);
	PUEA = _BV(SW_INCR) | _BV(SW_DECR);
	PUEB = _BV(SW_MODE);
	PUEC = _BV(SW_ENTR);
	
	TCCR0A = 0x00;
	TCCR0B = 0x0B;
	OCR0A = 0x00;
	
	TCCR1A = 0x00;
	TCCR1B = 0x0C;
	OCR1A = 0x0C35;
	
	TIMSK |= _BV(OCF1A) | _BV(OCF0A);

	PCMSK0 |= _BV(PCINT2) | _BV(PCINT3);
	PCMSK1 |= _BV(PCINT11);
	PCMSK2 |= _BV(PCINT12);
	
	GIMSK |= _BV(PCIE0) | _BV(PCIE1) | _BV(PCIE2);
	
	init_uart();
	PORTA |= _BV(PA5);
	_delay_ms(1000);
	PORTA &= ~(_BV(PA5));
	sei();
	
    while(1) { /* wait for interrupts! */ }
}