/*
 * main.h
 *
 * Created: 12/3/2012 9:47:44 PM
 *  Author: kmm
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#define HV_CK PC1
#define HV_DI PB2
#define HV_ST PC4
#define HV_OE PC5

#define LED_RED PA0
#define LED_YEL PA1

#define SW_MODE PB3
#define SW_ENTR PC0
#define SW_INCR PA3
#define SW_DECR PA2

#define DEFAULT_GPS_OFFSET 28

#define DEFAULT_GMT_OFFSET -8
#define DEFAULT_DST_OFFSET 0
#define DEFAULT_GPS_RATE 60

#define RX_BUF_SZ 64

#define RX_IDLE 0
#define RX_START 1
#define RX_READ 3
#define RX_HAVE_MSG 4
#define RX_END 5

#define FLAG_RENDER_12 0

#define OPR_MODE_RUN 0
#define OPR_MODE_CONF 1

#define MODE_TIME_24 0
#define MODE_TIME_12 1
#define MODE_TIME_24_M_D 2
#define MODE_TIME_12_M_D 3
#define MODE_TIME_24_M_D_Y 4
#define MODE_TIME_12_M_D_Y 5

#define MODE_SYNC_GPS_UPDATE 0
#define MODE_SYNC_GPS_ONLY 1
#define MODE_SYNC_GPS_OFF 2

#define SET_DISP_MODE 0
#define SET_SYNC_MODE 1
#define SET_GMT_OFFSET 2
#define SET_HOUR 3
#define SET_MIN 4
#define SET_SEC 5
#define SET_GPS_RATE 6

void handle_uart_rx();
void display(uint8_t, uint16_t);
void shift_hv(uint32_t);
void step_clock();
uint32_t bin_to_bcd(uint8_t);
void init_uart();
uint16_t render_time(uint32_t time, uint8_t flags);

#endif /* MAIN_H_ */
