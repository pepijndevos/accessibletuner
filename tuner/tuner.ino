#include <SPI.h>

#include "trig8.h"
#include "iir.h"
#include <limits.h>

#define SEMI 1.059463
#define CENT 1.0005777895

// Adjust to tune the tuner
#define ADCFREQ 1982.0
// scales to the range of sin8: 0-255
#define OMEGA (256/ADCFREQ)

#define E4 329.63
#define B3 246.94
#define G3 196.00
#define D3 146.83
#define A2 110.00
#define E2 82.41

#define BUZZER 9
#define EN_OUT 3
#define ADC_CS A1

// PORTB
#define SCROLL_PHASE 6
#define SCROLL_COUNTER 7

volatile unsigned long long counter = 0;
unsigned long sample_interval = 0;
unsigned long click_interval = 0;
int phase = 0;

const float strings[6] = {E2, A2, D3, G3, B3, E4};
int note_idx = 0;
float note = strings[0]*OMEGA;
IIR_filter bpf = iirpeak(strings[0]*2/ADCFREQ, 10*2/ADCFREQ);
IIR_filter lpf = {0, 0, 0, 0, 2, 4, 2, 3910, -1870};

void setup(){
  Serial.begin(250000);
  SPI.begin();
  
  pinMode(ADC_CS, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(EN_OUT, OUTPUT);
    // rotary encoder
  pinMode(SCROLL_PHASE, INPUT);
  pinMode(SCROLL_COUNTER, INPUT);

  PCICR = (1 << PCIE2);  //Enable PCI2 interupt
  PCMSK2 = (1 << PCINT22) | (1 << PCINT23);  // Mask for encoder pins
      
  // Enable output A, B, fast PWM
  TCCR1A =  _BV(COM1A1) | _BV(WGM10);
  // No PWM prescaler
  TCCR1B = _BV(CS10) | _BV(WGM12);
  // Duty cycle
  OCR1A = 127;
  digitalWrite(EN_OUT, HIGH);
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

  note = strings[(((note_idx/4) % 6) + 6) % 6]*OMEGA;
  IIR_filter bpf = iirpeak(strings[(note_idx/4) % 6]*2/ADCFREQ, 10*2/ADCFREQ);
}

void loop() {
  while(micros()-sample_interval < 500 || micros() < sample_interval);
  sample_interval = micros();

  // Do this first to reduce jitter
  OCR1A = sin8(note*counter+phase);

  SPI.beginTransaction(SPISettings(F_CPU/2, MSBFIRST, SPI_MODE0));
  digitalWrite(ADC_CS, LOW);
  delayMicroseconds(1);
  int16_t val = (int16_t)SPI.transfer16(0) - (1<<11);
  digitalWrite(ADC_CS, HIGH);
  SPI.endTransaction();
    
  int16_t sine = (int16_t)sin8(note*counter)-(int16_t)127;

  int16_t fval = IIR2(&bpf, val);
  int16_t prod = fval*sine;
  int16_t lpval = IIR2(&lpf, prod);
  // timeout on clicker for anti-madness
  if(millis()-click_interval > 20) {
    click_interval = millis();
    phase = (lpval>0)*128;
  }

if(counter % 64 == 0) {
  Serial.print(fval, DEC);
  Serial.print("\t");
//  Serial.print(note_idx, DEC);
//  Serial.print("\t");
  Serial.println(lpval, DEC);
//  Serial.print("\t");
//  Serial.println(micros()-sample_interval, DEC);
}

  counter++;
}
