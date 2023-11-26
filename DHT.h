/*#ifndef _DHT_h
#define _DHT_h

//#define F_CPU 8000000UL

#define DHT_PORT_OUT PORTB
#define DHT_PORT_IN PINB
#define DHT_PIN PB5

#define SET_BIT(byte, bit) ((byte) |= (1UL << (bit)))
#define CLEAR_BIT(byte,bit) ((byte) &= ~(1UL << (bit)))
#define IS_SET(byte,bit) (((byte) & (1UL << (bit))) >> (bit))

#include <stdint.h>

//uint8_t dht_data;

void initDHT(void);

uint8_t dht_fetchData(uint8_t* arr);

#endif
*/
