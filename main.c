#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <string.h>

#define GET_UBRR_VAL(baudrate) ( F_CPU/(16UL*(baudrate)) - 1 )
#define UBRR_VAL GET_UBRR_VAL(9600)
#define UBRRL_VAL (uint8_t)( (UBRR_VAL) & 0xff )
#define UBRRH_VAL (uint8_t)((UBRR_VAL) >> 8)

#define GET_TWBR_VAL(clkrate, prsc) ( (F_CPU/(clkrate) - 16UL)/(2*(prsc)))
#define TWBR_VAL GET_TWBR_VAL(100000UL, 1)

#define TM1637_CLK PC5
#define TM1637_DIO PC4
#define TM1637_DDR DDRC
#define TM1637_PORT PORTC
#define TM1637_PIN PINC

#define TM1637_DATA_FIXED_ADDR_COMMAND 0b01000100
#define TM1637_DATA_AUTO_ADDR_COMMAND 0b01000000
#define TM1637_SET_ADDR_COMMAND(addr) (0b11000000 + ( ( addr )&0b111 ))
#define TM1637_DISPLAY_COMMAND(on_off, brightness) (0b10000000 + ( ( (on_off)&1 ) << 3 ) + ( (brightness)&0b111 ))
#define TM1637_DELAY_US 1

#define HEATER_DDR DDRD
#define HEATER_PORT PORTD
#define HEATER_PIN PIND
#define HEATER_IO PD7

#define TEMP_HARD_LIMIT 48 // If this limit is reached the heater is turned off till a delta has been
                           // achieved
#define TEMP_LIMIT_DELTA 5 // Delta for heater to turn back on
#define TEMP_TARGET_DELTA 1 // Once the target temperature is reached, the heater is turned off
                            // till the temperature drops below the target by this number

#define UI_UP_DDR DDRC
#define UI_UP_PORT PORTC
#define UI_UP_PIN PINC
#define UI_UP_IO PC3

#define UI_DOWN_DDR DDRC
#define UI_DOWN_PORT PORTC
#define UI_DOWN_PIN PINC
#define UI_DOWN_IO PC2

#define UI_SET_DDR DDRD
#define UI_SET_PORT PORTD
#define UI_SET_PIN PIND
#define UI_SET_IO PD2

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PIN PIND
#define LED_IO PD3

#define DISABLE_INT0 EIMSK &= ~(1 << INT0)
#define ENABLE_INT0 EIMSK |= (1 << INT0)


typedef struct {
  char buf[100]; // Buffer where UART data is stored
  uint8_t i; // Index of the next character to send
  uint8_t sent; // If the data in the buffer has been sent or not
} uart_tx_buf_t;
volatile uart_tx_buf_t uart_tx_buf;

uint8_t sev_seg_map[] = {
  0b00111111, // For 0
  0b00000110, // For 1
  0b01011011, // For 2
  0b01001111, // For 3
  0b01100110, // For 4
  0b01101101, // For 5
  0b01111101, // For 6
  0b00000111, // For 7
  0b01111111, // For 8
  0b01101111, // For 9
  0x00, //For all off, used for blinking purposes
};

#define NUM_STATES 3
typedef enum {
  STATE_NORMAL, // Temp control loop and send measured temp via UART
  STATE_SET_TEMP, // Turn off heater and set temperature
  STATE_LIMIT_REACHED // Turn off heater if limit reached and wait for the specified delta
                      // before turning the heater on again
} state_t;
volatile state_t state = STATE_NORMAL;
volatile state_t next_state = STATE_NORMAL;
volatile state_t state_isr_bu = STATE_NORMAL;
volatile uint8_t target_temp = 35;


void init_uart() {
  UBRR0L = UBRRL_VAL;
  UBRR0H = UBRRH_VAL;
  UCSR0C |= (1 << 3); 
}

void init_uart_int() {
  init_uart();
  UCSR0B |= (1 << UDRIE0);
  uart_tx_buf.sent = 1;
}

void uart_send_byte(char c) {
  while(!( UCSR0A & (1 << 5) ));
  UDR0 = c;
}


void uart_send_string(const char* s) {
  while(*s != 0) {
    uart_send_byte(*s);
    s++;
  }
}

void uart_int_send_string(const char* s) {
  while(uart_tx_buf.sent != 1);
  while(!(UCSR0A & (1 << UDRE0)));
  uint8_t len = strlen(s);
  if ( len < 100 ){
    for(uint8_t i = 0; i <= len; i++) {
      uart_tx_buf.buf[i] = s[i];
    }
    uart_tx_buf.i = 1;
    UDR0 = uart_tx_buf.buf[0];
    uart_tx_buf.sent = 0;
    UCSR0B |= (1 << UDRIE0);
  }
}

void uart_send_num(uint16_t num, int skip_lzero) {
  if(num == 0){
    uart_send_byte('0');
    return;
  }
  char st[6];
  st[5] = 0;
  for(int i = 4; i >= 0; i--) {
    st[i] = num % 10 + '0';
    num = num/10;
  }
  if(skip_lzero) {
  int leading_zero_skip = 0;
  while(st[leading_zero_skip] == '0') {
    leading_zero_skip++;
  }
  uart_send_string(st + leading_zero_skip);
  }
  else uart_send_string(st);
}


void uart_int_send_num(uint16_t num, int skip_lzero) {
  if(num == 0){
    uart_int_send_string("0");
    return;
  }
  char st[6];
  st[5] = 0;
  for(int i = 4; i >= 0; i--) {
    st[i] = num % 10 + '0';
    num = num/10;
  }
  if(skip_lzero) {
  int leading_zero_skip = 0;
  while(st[leading_zero_skip] == '0') {
    leading_zero_skip++;
  }
  uart_int_send_string(st + leading_zero_skip);
  }
  else uart_int_send_string(st);
}

void init_spi() {
  SPCR |= (1 << MSTR) | (1 << CPHA);
  SPCR |= 1 | (1 << SPE);
}

uint8_t read_spi() {
  SPDR = 0x00;
  while(!(SPSR & (1 << SPIF)));
  uint8_t data = SPDR;
  return data;
}

void write_spi(char c) {
  SPDR = c;
  while(!(SPSR & (1 << SPIF)));
}

uint16_t read_6675() {
  PORTB &= ~(1 << PB0);
  PORTB &= ~(1 << PB2);
  //_delay_ms(100);
  uint8_t ret_high = read_spi();
  uint8_t ret_low = read_spi();
  //_delay_ms(100);
  PORTB |= (1 << PB2);
  PORTB |= (1 << PB0);
  return ((uint16_t)ret_high << 8) | ret_low;
}


void tm1637_send_start() {
  TM1637_PORT |= (1 << TM1637_CLK) | (1 << TM1637_DIO);
  TM1637_DDR |= ( 1 << TM1637_CLK ) | (1 << TM1637_DIO);
  TM1637_PORT &= ~(1 << TM1637_DIO);
  _delay_us(TM1637_DELAY_US);
  TM1637_PORT &= ~(1 << TM1637_CLK);
  _delay_us(TM1637_DELAY_US);
}

int tm1637_send_byte(uint8_t c, int send_stop) {
  for(int i = 0; i <= 7; i++) {
    if((c>>i) & 1) {
      TM1637_PORT |= (1 << TM1637_DIO);
    } else {
      TM1637_PORT &= ~(1 << TM1637_DIO);
    }
    _delay_us(TM1637_DELAY_US);
    TM1637_PORT |= (1 << TM1637_CLK);
    _delay_us(TM1637_DELAY_US);
    TM1637_PORT &= ~(1 << TM1637_CLK);
    _delay_us(TM1637_DELAY_US);
  }
  TM1637_DDR &= ~(1 << TM1637_DIO);
  _delay_us(TM1637_DELAY_US);
  TM1637_PORT |= (1 << TM1637_CLK);
  _delay_us(TM1637_DELAY_US);
  int nack = TM1637_PIN & (1 << TM1637_DIO);
  TM1637_PORT &= ~(1 << TM1637_CLK);
  _delay_us(TM1637_DELAY_US);
  TM1637_PORT &= ~(1 << TM1637_DIO);
  TM1637_DDR |= (1 << TM1637_DIO);
  _delay_us(TM1637_DELAY_US);
  if(send_stop) {
    TM1637_PORT |= (1 << TM1637_CLK);
    _delay_us(TM1637_DELAY_US);
    TM1637_PORT |= (1 << TM1637_DIO);
  }
  return !nack;
}

void tm1637_set_first2_digits(uint8_t num) {
  tm1637_send_start();
  tm1637_send_byte(TM1637_DATA_AUTO_ADDR_COMMAND, 1);
  tm1637_send_start();
  tm1637_send_byte(TM1637_SET_ADDR_COMMAND(0), 0);
  tm1637_send_byte(sev_seg_map[num/10], 0);
  tm1637_send_byte(sev_seg_map[num%10], 1);
}

void tm1637_set_last2_digits(uint8_t num) {
  tm1637_send_start();
  tm1637_send_byte(TM1637_DATA_AUTO_ADDR_COMMAND, 1);
  tm1637_send_start();
  tm1637_send_byte(TM1637_SET_ADDR_COMMAND(2), 0);
  tm1637_send_byte(sev_seg_map[num/10], 0);
  tm1637_send_byte(sev_seg_map[num%10], 1);
}


void init_ui() {
  // Enable Falling edge detection on INT0
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  ENABLE_INT0;
}

void debounce_timer_init() {
  TCCR0B = (0b101 << CS00); // Divide timer clk by 1024
}



// State Transition functions----
void normal_to_set_temp() {
  HEATER_PORT &= ~(1 << HEATER_IO);
  //ENABLE_INT0;
}

void set_temp_to_normal() {
  LED_PORT &= ~(1 << LED_IO);
}

void normal_to_limit_reached() {
  HEATER_PORT &= ~(1 << HEATER_IO);
  LED_PORT |= (1 << LED_IO);
}

void limit_reached_to_normal() {
  LED_PORT &= ~(1 << LED_IO);
}

void limit_reached_to_set_temp() {
}

void set_temp_to_limit_reached() {
  LED_PORT |= (1 << LED_IO);
}

typedef void(*func_ptr)();

func_ptr state_transition_map[NUM_STATES][NUM_STATES] = {
  {NULL, normal_to_set_temp, normal_to_limit_reached}, // normal to others
  {set_temp_to_normal, NULL, set_temp_to_limit_reached}, // set_temp to others
  {limit_reached_to_normal, limit_reached_to_set_temp, NULL} // limit_reached to others
};

//---------------
//
int main(void) {
  // Set TX as output
  DDRD = (1 << PD1);
  //DDRC = (1 << PC5) | ( 1 << PC4 );
  TM1637_PORT |= (1 << TM1637_CLK) | (1 << TM1637_DIO);
  TM1637_DDR |= (1 << TM1637_CLK) | (1 << TM1637_DIO);
  // Set the heater pin and turn it off
  HEATER_DDR |= (1 << HEATER_IO);
  HEATER_PORT &= ~(1 << HEATER_IO);
  // Set the UI push button pins as an input
  // and enable the internal pull up
  UI_UP_DDR &= ~(1 << UI_UP_IO);
  UI_UP_PORT |= (1 << UI_UP_IO);
  UI_DOWN_DDR &= ~(1 << UI_DOWN_IO);
  UI_DOWN_PORT |= (1 << UI_DOWN_IO);
  UI_SET_DDR &= ~(1 << UI_SET_IO);
  UI_SET_PORT |= (1 << UI_SET_IO);
  // Set the UI LED as an output
  LED_DDR |= (1 << LED_IO);
  // Set the SPI pins
  DDRB = (1 << PB5) | (1 << PB0) | (1 << PB2) | (1 << PB3);
  PORTB = 1 << PB0 | 1 << PB2;
  sei();
  debounce_timer_init();
  init_uart_int();
  init_spi();
  init_ui();
  // Initialize Display
  tm1637_send_start();
  tm1637_send_byte(TM1637_DATA_AUTO_ADDR_COMMAND, 1);
  tm1637_send_start();
  tm1637_send_byte(TM1637_SET_ADDR_COMMAND(0), 0);
  for(int i = 0; i < 6; i++) {
    if(i < 5) tm1637_send_byte(sev_seg_map[9], 0);
    else tm1637_send_byte(sev_seg_map[1], 1);
  }
  tm1637_send_start();
  tm1637_send_byte(TM1637_DISPLAY_COMMAND(1, 3), 1);
  tm1637_send_start();
  tm1637_send_byte(TM1637_DATA_FIXED_ADDR_COMMAND, 1);
  while(1){
    if(state != next_state) {
      state_transition_map[state][next_state]();
      state = next_state;
    }
    if(state == STATE_NORMAL){
      uint16_t temp = (read_6675() >> 3) & 0xfff;
      temp *= 25;
      DISABLE_INT0;
      if(temp/100 >= TEMP_HARD_LIMIT) {
        next_state = STATE_LIMIT_REACHED;
      } else {
        ENABLE_INT0;
      }
      if(temp/100 >= target_temp) {
        HEATER_PORT &= ~(1 << HEATER_IO);
      }
      if(temp/100 + TEMP_TARGET_DELTA <= target_temp) {
        HEATER_PORT |= (1 << HEATER_IO);
      }
      uart_int_send_string("Temperature: ");
      tm1637_set_first2_digits(temp/100);
      uart_int_send_num(temp/100, 1);
      uart_int_send_string(".");
      uart_int_send_num(temp%100, 1);
      uart_int_send_string("\r\n");
      tm1637_set_last2_digits(target_temp);
      PORTB ^= (1 << PB5);
      _delay_ms(300);
    } else if (state == STATE_SET_TEMP) {
      static uint8_t cnt = 0;
      if(!cnt) {
        LED_PORT ^= (1 << LED_IO);
      } else if(cnt == 3) {
        cnt = 0;
      }
      else {
        cnt++;
      }
      if(!( UI_UP_PIN & (1 << UI_UP_IO ) )) {
        target_temp++;
        tm1637_set_last2_digits(target_temp);
      } else if (!(UI_DOWN_PIN & (1 << UI_DOWN_IO))) {
        target_temp--;
        tm1637_set_last2_digits(target_temp);
      }
      _delay_ms(150); // A delay for debouncing
    }
    else if (state == STATE_LIMIT_REACHED) {
      uint16_t temp = ( read_6675() >> 3 ) & 0xfff;
      temp /= 4;
      DISABLE_INT0;
      if(TEMP_HARD_LIMIT  >= TEMP_LIMIT_DELTA + temp) {
        next_state = STATE_NORMAL;
      } else {
        ENABLE_INT0;
      }
      _delay_ms(300);
    }
  }
}


// Interrupts

ISR(USART_UDRE_vect)
{
  if(uart_tx_buf.sent == 0) {
    while(!(UCSR0A & (1 << UDRE0)));
    if(uart_tx_buf.buf[uart_tx_buf.i] != 0) {
      UDR0 = uart_tx_buf.buf[uart_tx_buf.i];
      uart_tx_buf.i++;
    } else {
      uart_tx_buf.sent = 1;
    }
  } else {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

ISR(INT0_vect)
{
  if(state != STATE_SET_TEMP){
    state_isr_bu = state;
    next_state = STATE_SET_TEMP;
  }
  else
    next_state = state_isr_bu;

}

