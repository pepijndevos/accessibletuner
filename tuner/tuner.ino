#include <SPI.h>

#include "trig8.h"
#include <limits.h>

#define SEMI 1.059463
#define CENT 1.0005777895

// CPU_F/prescaler/ADC_cycles
// 160000000/128/13
// Adjust to tune the tuner
#define ADCFREQ 1953.128
// scales to the range of sin8: 0-255
#define OMEGA (256/ADCFREQ)

#define E4 (329.63*OMEGA)
#define B3 (246.94*OMEGA)
#define G3 (196.00*OMEGA)
#define D3 (146.83*OMEGA)
#define A2 (110.00*OMEGA)
#define E2 (82.41*OMEGA)

#define BUZZER 9
#define EN_OUT 3
#define ADC_CS A1

// PORTB
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SCROLL_PHASE A8
#define SCROLL_COUNTER A9
#else
#define SCROLL_PHASE 6
#define SCROLL_COUNTER 7
#endif

#define BUFSIZE (1<<6)
#define BUFMASK (BUFSIZE-1)
volatile uint16_t buf[BUFSIZE];
volatile unsigned long long counter = 0;

#define ENVSIZE (1<<8)
#define ENVMASK (ENVSIZE-1)
uint16_t env_buf[ENVSIZE];
uint8_t env_counter = 0;

const float strings[6] = {E2, A2, D3, G3, B3, E4};
//const char names[6][2] = {"E", "A", "D", "G", "B", "E"};
int note_idx = 4;
//volatile float note = D3;
volatile uint8_t phase = 0;

void setup(){
  Serial.begin(115200);
  SPI.begin();
  
  pinMode(ADC_CS, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(EN_OUT, OUTPUT);
    // rotary encoder
  pinMode(SCROLL_PHASE, INPUT);
  pinMode(SCROLL_COUNTER, INPUT);

  PCICR = (1 << PCIE2);  //Enable PCI2 interupt
  PCMSK2 = (1 << PCINT22) | (1 << PCINT23);  // Mask for encoder pins
    
  //set up continuous sampling of analog pin 0 at 10kHz
  // Timer0 is already used for millis() this breaks everything
  TCCR0A =  _BV(WGM01);
  TCCR0B = _BV(CS01) | _BV(CS00);
  OCR0A = 127;
  TIMSK0 |= _BV(OCIE0A);
      
  // Enable output A, B, fast PWM
  TCCR1A =  _BV(COM1A1) | _BV(WGM10);
  // No PWM prescaler
  TCCR1B = _BV(CS10) | _BV(WGM12);
  // Duty cycle
  OCR1A = 127;
  digitalWrite(EN_OUT, HIGH);

}

SIGNAL(TIMER0_COMPA_vect)  {//when new ADC value ready
  SPI.beginTransaction(SPISettings(F_CPU/2, MSBFIRST, SPI_MODE0));
  digitalWrite(A1, LOW);
  delayMicroseconds(1);
  uint16_t val = SPI.transfer16(0)<<4;
  digitalWrite(A1, HIGH);
  SPI.endTransaction();
  
  float note = strings[(note_idx/4) % 6];
  uint16_t sine = sin16(note*256*counter)^(1<<15);
  OCR1A = sin8(note*counter+phase);
  //digitalWrite(CLICKER, enve>0);
  int idx = counter & BUFMASK;
  uint32_t prod = (uint32_t)(uint16_t)val*(uint32_t)(uint16_t)sine;
  buf[idx] = prod>>16;
  counter++;
}

//https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
SIGNAL(PCINT2_vect) {
  const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
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
  //const float enc_states[] = {1,1,1,1,1,1,1,1/SEMI,1,1,1,SEMI,1,1,1,1};
  static uint8_t old_AB = 0;

  old_AB <<= 2; //remember previous state
  old_AB |= (PIND >> 6) & 0x03; //add current state
  //note_idx *= enc_states[old_AB & 0x0f];
  note_idx += enc_states[old_AB & 0x0f];
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

    uint16_t threshold = (env_min/2) + (env_max/2);

    if (env_max-env_min > 5000) {
      phase = (m>threshold)*128;
    }
    int idx = counter & BUFMASK;
    Serial.print(m, DEC);
    Serial.print("\t");
    Serial.print(buf[idx], DEC);
    Serial.print("\t");
    Serial.print(env_min, DEC);
    Serial.print("\t");
    Serial.print(env_max, DEC);
    Serial.print("\t");
    Serial.println(threshold, DEC);
}
