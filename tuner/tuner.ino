#include <limits.h>

#define E4 329.63
#define B3 246.94
#define G3 196.00
#define D3 146.83
#define A2 110.00
#define E2 82.41
#define SEMITONE 1.059463
#define CENT 1.0005777895
// CPU_F/prescaler/ADC_cycles
// 160000000/32/13
#define ADCFREQ 38461.0
// how tuned is tuned? freq/MARGIN
#define MARGIN 300

#define BUZZER 2
#define BUTTON 3

volatile uint8_t buf[1024];
volatile unsigned int counter = 0;

const float strings[6] = {E2, A2, D3, G3, B3, E4};
const char names[6][2] = {"E", "A", "D", "G", "B", "E"};
int note_idx = 0;

void setup(){
  
  Serial.begin(9600);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
    
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0) //set reference voltage
        |  (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0) //set ADC clock with 32 prescaler- 16mHz/32=500kHz
         |  (1 << ADATE) //enabble auto trigger
         |  (1 << ADIE) //enable interrupts when measurement complete
         |  (1 << ADEN) //enable ADC
         |  (1 << ADSC); //start ADC measurements
  
  //tone(2, note);
}

ISR(ADC_vect) {//when new ADC value ready
  buf[counter] = ADCH;
  counter = (counter + 1) & 1023;
}

// get frequency between given bounds
float get_frequency(const float low, const float high) {
  int best_cycle = 0;
  int best_difference = INT_MAX;
  for (int offset = ADCFREQ/high; offset<ADCFREQ/low; offset++) {
    int difference = 0;
    for (int i = 0; i<ADCFREQ/low; i++) {
      difference += max(buf[i], buf[i+offset])-min(buf[i], buf[i+offset]);
    }
    if (difference < best_difference) {
      best_cycle = offset;
      best_difference = difference;
    }
  }
  return ADCFREQ/best_cycle;
}

void loop() {
  float note = strings[note_idx];
  while (counter != 0);
  ADCSRA &= ~(1 << ADIE); //disable ADC ISR
  float fr = get_frequency(note*0.9, note*1.1);
  float detune = fr-note;

  if (detune > (note/MARGIN)) {
    tone(BUZZER, note*SEMITONE*SEMITONE);
    delay(100);
  } else if (detune < -(note/MARGIN)) {
    tone(BUZZER, note/SEMITONE/SEMITONE);
    delay(100);
  }
  tone(BUZZER, note);
  delay(100);
  noTone(BUZZER);

  if(digitalRead(BUTTON)) {
    delay(500);
    note_idx++;
    if (note_idx == 6) note_idx = 0;
    
    for (int i = 0; i<note_idx+1; i++) {
      tone(BUZZER, strings[i]);
      delay(200);
    }
    noTone(BUZZER);
  }

  delay(100);
  
  ADCSRA |= (1 << ADIE); // reenable ADC ISR
  Serial.print(fr); Serial.print(" "); Serial.println(detune);
}
