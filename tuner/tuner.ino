#include "trig8.h"
#include <limits.h>

#define SEMI 1.059463
#define CENT 1.0005777895

// CPU_F/prescaler/ADC_cycles
// 160000000/128/13
// Adjust to tune the tuner
#define ADCFREQ 9575.0
// scales to the range of sin8: 0-255
#define OMEGA (255/ADCFREQ)

#define E4 (329.63*OMEGA)
#define B3 (246.94*OMEGA)
#define G3 (196.00*OMEGA)
#define D3 (146.83*OMEGA)
#define A2 (110.00*OMEGA)
#define E2 (82.41*OMEGA)

#define BUZZER 3
#define ENVELOPE 11
#define CLICKER 2

// PORTB
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SCROLL_PHASE A8
#define SCROLL_COUNTER A9
#else
#define SCROLL_PHASE 6
#define SCROLL_COUNTER 7
#endif

#define BUFSIZE (1<<8)
#define BUFMASK (BUFSIZE-1)
volatile uint16_t buf[BUFSIZE];
volatile unsigned long long counter = 0;

#define ENVSIZE (1<<8)
#define ENVMASK (BUFSIZE-1)
uint16_t env_buf[ENVSIZE];
uint8_t env_counter = 0;

//const float strings[6] = {E2, A2, D3, G3, B3, E4};
//const char names[6][2] = {"E", "A", "D", "G", "B", "E"};
//int note_idx = 5;
volatile float note = D3;

void setup(){
  
  //Serial.begin(9600);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(ENVELOPE, OUTPUT);
  pinMode(CLICKER, OUTPUT);

  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);

    // rotary encoder
  pinMode(SCROLL_PHASE, INPUT);
  pinMode(SCROLL_COUNTER, INPUT);
  //attachInterrupt(SCROLL_PHASE, read_encoder, CHANGE);
  //attachInterrupt(SCROLL_COUNTER, read_encoder, CHANGE);

  PCICR = (1 << PCIE2);  //Enable PCI2 interupt
  PCMSK2 = (1 << PCINT22) | (1 << PCINT23);  // Mask for encoder pins
    
  //set up continuous sampling of analog pin 0 at 10kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0) //set reference voltage
        |  (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1)| (1 << ADPS0) //set ADC clock with 128 prescaler- 16mHz/128=125kHz
         |  (1 << ADATE) //enabble auto trigger
         |  (1 << ADIE) //enable interrupts when measurement complete
         |  (1 << ADEN) //enable ADC
         |  (1 << ADSC); //start ADC measurements

  // Enable output A, B, fast PWM
  TCCR2A =  _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  // No PWM prescaler
  TCCR2B = _BV(CS20);
  // Duty cycle
  OCR2A = 127;
  OCR2B = 127;

  // SPI slave for debugging via USBTinyISP
  SPCR = _BV(SPE) | _BV(SPIE);
}

SIGNAL(ADC_vect) {//when new ADC value ready
  //uint16_t val = ADCL;
  //val |= ADCH << 8;
  uint8_t val = ADCH;
  uint8_t sine = sin8(note*counter);
  //int16_t prod = (val*sine)-0x7fff;
  //int16_t enve = IIR2(&lpf, prod);
  OCR2B = sine;
  //digitalWrite(CLICKER, enve>0);
  int idx = counter & BUFMASK;
  buf[idx] = val*sine;
  counter++;
}

//https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
SIGNAL(PCINT2_vect) {
  //const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  // 00 00 -> --
  // 00 01 -> CW
  // 00 10 -> CCW
  // 00 11 -> --
  // 01 00 -> CCW
  // 01 01 -> --
  // 01 10 -> --
  // 01 11 -> CW
  // 10 00 -> CW
  // 10 01 -> --
  // 10 10 -> --
  // 10 11 -> CCW
  // 11 00 -> --
  // 11 01 -> CCW
  // 11 10 -> CW
  // 11 11 -> --
  const float enc_states[] = {1,1,1,1,1,1,1,1/SEMI,1,1,1,SEMI,1,1,1,1};
  static uint8_t old_AB = 0;

  old_AB <<= 2; //remember previous state
  old_AB |= (PIND >> 6) & 0x03; //add current state
  note *= enc_states[old_AB & 0x0f];
}

SIGNAL(SPI_STC_vect) {
  SPDR = OCR2A;
}

void loop() {
    uint16_t m = 0;
    for(int i=0; i<BUFSIZE; i++) {
      if (buf[i] > m) {
        m = buf[i];
      }
    }
    //OCR2A = m>>8;
    env_buf[env_counter & ENVMASK] = m;
    env_counter++;
    uint16_t env_min = 0xffff;
    uint16_t env_max = 0;
    for(int i=0; i<ENVSIZE; i++) {
      if (env_buf[i] > env_max) {
        env_max = env_buf[i];
      }
      if (env_buf[i] < env_min) {
        env_min= env_buf[i];
      }
    }

    if (env_max-env_min > 5000) {
      uint16_t threshold = (env_min/2) + (env_max/2);
      digitalWrite(CLICKER, m>threshold);
    }
    //Serial.println(PINK, BIN);
}
