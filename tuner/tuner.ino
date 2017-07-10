#include "trig8.h"
#include "iir.h"
#include <limits.h>

#define SEMITONE 1.059463
#define CENT 1.0005777895

// CPU_F/prescaler/ADC_cycles
// 160000000/128/13
#define ADCFREQ 9615.384615384615
// scales to the range of sin8: 0-255
#define OMEGA (255/ADCFREQ)

#define E4 (329.63*OMEGA)
#define B3 (246.94*OMEGA)
#define G3 (196.00*OMEGA)
#define D3 (146.83*OMEGA)
#define A2 (110.00*OMEGA)
#define E2 (82.41*OMEGA)

// how tuned is tuned? freq/MARGIN
#define MARGIN 300

#define BUZZER 10
#define CLICKER 9
#define BUTTON 3

volatile uint16_t buf[256];
volatile unsigned int counter = 0;

const float strings[6] = {E2, A2, D3, G3, B3, E4};
const char names[6][2] = {"E", "A", "D", "G", "B", "E"};
int note_idx = 3;


/**********************************************************/
// Filter coefficients
/**********************************************************/
/* These coefficients will give the response of a low pass filter.
 * A0 is not defined since it is the downscaling factor.
 *
 * Note that the sign is changed on the A*-coefficients, relative to the ones
 * presented in the appplication note. This is because only additions are used
 * in the filtering algorithm, while the difference equation requires
 * subtractions for the feedback-part.
 */
#define FB0    2072   //!< First feed-forward coefficient.
#define FB1   -4143   //!< Second feed-forward coefficient.
#define FB2    2072   //!< Third feed-forward coefficient.
#define FA1    130961 //!< Second feedback coefficient.
#define FA2   -65425 //!< Third feedback coefficient.

// Initialize the filter struct.
struct IIR_filter lpf = {0,0,0,0, FB0, FB1, FB2, FA1, FA2};

void setup(){
  
  Serial.begin(9600);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(CLICKER, OUTPUT);
  pinMode(BUTTON, INPUT);
    
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
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  // No PWM prescaler
  TCCR2B = _BV(CS20);
  // Duty cycle
  OCR2A = 127;
}

SIGNAL(ADC_vect) {//when new ADC value ready
  //uint16_t val = ADCL;
  //val |= ADCH << 8;
  uint8_t val = ADCH;
  float f = strings[note_idx];
  uint8_t sine = sin8(f*counter);
  OCR2A = sine;
  int16_t prod = (val*sine)-0x7fff;
  int16_t enve = IIR2(&lpf, prod);
  digitalWrite(CLICKER, enve>0);
  //int idx = counter & 0xff;
  //buf[idx] = val*sine;
  counter++;
}

void loop() {
    Serial.println("hello");
}
