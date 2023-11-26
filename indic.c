#include <avr/io.h>
#include <util/delay.h>

#define SET_BIT(byte, bit) ((byte) |= (1UL << (bit)))
#define CLEAR_BIT(byte,bit) ((byte) &= ~(1UL << (bit)))
#define IS_SET(byte,bit) (((byte) & (1UL << (bit))) >> (bit))
//#define IS_SET(byte,bit) (((byte) >> (bit)) & 1)


unsigned char display_number[] =
{
  0x3f, //0
  0x06, //1
  0x5b, //2
  0x4f, //3
  0x66, //4
  0x6d, //5
  0x7d, //6
  0x07, //7
  0x7f, //8
  0x6f  //9
};

unsigned int display_digit = 0;
unsigned int* display_digits[4];

void display_change(unsigned int display_value) {
    if (display_digit == 0) display_digits[0] = ~(display_number[display_value / 1000]);
    else if (display_digit == 1) display_digits[1] = ~(display_number[(display_value / 100) % 10]);
    else if (display_digit == 2) display_digits[2] = ~(display_number[(display_value / 10) % 10]);
    else if (display_digit == 3) display_digits[3] = ~(display_number[display_value % 10]);
}

void display_show() {
	PORTD = display_digits[display_digit];

    //PINB.1 = (digit >> 3) && 1;
    //PORTB2 = (digit >> 2) && 1;
    //PORTB3 = (digit >> 1) && 1;
    //PORTB4 = (digit >> 0) && 1;
    PORTB &= 0b11110000;
    SET_BIT(PORTB, display_digit);

    display_digit += 1;
    if (display_digit > 3) display_digit = 0;
}
