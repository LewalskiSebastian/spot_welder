#include <EEPROM.h>

long times = 0;
const byte interruptPin = 2;
const byte encoderPin = 3;
const byte encoderPinB = 4;
const byte outPin = 13;
unsigned long last_time = 0;
uint16_t milis = 999;
uint16_t def_milis = 500;
uint16_t new_milis;
unsigned long start_time = 0;
volatile int analogVal;

int address_low = 1;  // Adres EEPROM do któego zapisywana jest aktualna wartość milis
int address_high = 0;
// TODO: Przetestuj i dobierz te czasy tak aby dobrze zapisywało, ewentualnie daj większy kondensator na zasilanie
float min_voltage = 4.4;  //Napięcie przy którym zapisywana jest aktualna wartość milis do EEPROM
float normal_voltage = 4.41; // Jeżeli napięcie jest większe od tego to anulowane jest czekania na wyłączenie (waitTurnOff)
bool waitTurnOff = false; //Prawda jeżeli napięcie jest mniejsze od min_voltage, nie zapisuj wtedy do EEPROM tylko czekaj aż się wyłączy lub zwiększy napięcie

#define Dig1    10
#define Dig2    9
#define Dig3    8
#define Dig4    5

#define clockPin  7
#define dataPin   6

char text[5] = "HExO";  // Powitanie "HEllO"

bool is_text = true;
byte current_digit;
int  count = 0;
void disp(byte number, bool dec_point = false);
void disp_char(char text, bool dec_point = false);

void setup() {
  Serial.begin(9600);   // Inicjalizacja USART
  byte milis_low = 0;
  byte milis_high = 0;
  EEPROM.get(address_low, milis_low);
  EEPROM.get(address_high, milis_high);
  milis = ((uint16_t)milis_high << 8) + milis_low;
  Serial.print("load_milis = ");
  Serial.println(milis);
  if (milis < 0 || milis > 9999) {
    milis = def_milis;
  }
  
  noInterrupts();   // Wyłączeniue przerwań cli()

  pinMode(Dig1, OUTPUT);
  pinMode(Dig2, OUTPUT);
  pinMode(Dig3, OUTPUT);
  pinMode(Dig4, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  disp_off();  // Wyłączenie wyświetlacza
  
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPin), INT, CHANGE);  // Inicjalizacja przerwania zewnętrznego
  attachInterrupt(digitalPinToInterrupt(encoderPin), INT_encoder, RISING);  // Inicjalizacja przerwania zewnętrznego
  timer2_init();   // Inicjalizacja timera
  adc_init(); // Inicjalizacja ADC
  interrupts();   // Włączenie przerwań sei();
  start_time = millis();
  ADCSRA |=B01000000;
}

void loop() {
  if (millis() - start_time > 1000){  // Po wyświetleniu powitania znawsze na początku wyłącz is_text aby wyświetlić liczbę
    is_text = false;
  }
  
  if (waitTurnOff) {    // Gdy niskie napięcie zasilania pożegnaj się
    char on_text[5] = "BYE!";
    for(int i = 0; i < 5; i++)
    {
      text[i] = on_text[i];
    }
    is_text = true;
  }

  if (TIMSK1 & ((1 << OCIE1A)|(1 << TOIE1))) {  // Gdy zgrzewanie jest aktywne
    char on_text[5] = " ON ";
    for(int i = 0; i < 5; i++)
    {
      text[i] = on_text[i];
    }
    is_text = true;
  }
  
  while (Serial.available() > 0) {
    unsigned long received = Serial.parseInt();
    if (received < 10000 && received > 0) {
      milis = received;
      Serial.flush();
      Serial.print("I received: ");
      Serial.println(milis);
//      EEPROM.put(address_low, lowByte(milis));
//      EEPROM.put(address_high, highByte(milis));
    }
  }
}

void set_timer(unsigned long shot_time) {
  if (TIMSK1 & ((1 << OCIE1A)|(1 << TOIE1))) {
    return;
  }
  noInterrupts();
  unsigned long ticks = shot_time * 250;
  unsigned long max_ticks = 65535;
  times = ticks / max_ticks;
  unsigned long rest = ticks % max_ticks;
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  TCCR1B |= (1 << CS11) | (1 << CS10);    // 64 prescaler 

  OCR1A = rest;            // Rejestr do którego porównywany jest rejestr timera 16MHz/1024/0.25s
  
  if (times) {
    TIMSK1 |= (1 << TOIE1);  // Przerwanie po przepełnieniu licznika
  }
  else
  {
    TCCR1B |= (1 << WGM12);   // Tryb CTC (czyści licznik po porównaniu)
    TIMSK1 |= (1 << OCIE1A);  // Przerwanie po porównaniu rejestrów
  }
  digitalWrite(outPin, HIGH);
  interrupts();
}

void timer2_init() {   // Inicjalizacja timera1
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  TCCR2A |= (1 << WGM21); // Tryb CTC

  OCR2A = 40;  // Rejestr do którego porównywany jest rejestr timera 16MHz/1024/400Hz

  TIMSK2 |= (1 << OCIE2A);    // Przerwanie po porównaniu rejestrów

  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);    // 1024 prescaler 
}

void adc_init() {
  ADMUX = (0x01 << REFS0) /* AVCC with external capacitor at AREF pin */
          | (0 << ADLAR) /* Left Adjust Result: disabled*/
          | (0x0e << MUX0) /* Internal Reference (VBG) */;
  
  // Ustawia bit ADEN w ADCSRA (0x7A) aby włączyć przetwornik ADC.
  ADCSRA |= B10000000;
 
  // Ustawia bit ADATE w ADCSRA (0x7A) aby włączyć automatyczne wyzwalanie przetwornika
  ADCSRA |= B00100000;
 
  // Zeruje bity ADTS2-0 w ADCSRB (0x7B) aby włączyć tryb ciągłego przetwarzania
  ADCSRB &= B11111000;
 
  // Ustawia prescaler na 128 (16MHz/128 = 125kHz)
  ADCSRA |= B00000111;
 
  // Ustawia bit ADIE w ADCSRA (0x7A) aby aktywować przerwania wewnętrzne ADC
  ADCSRA |= B00001000;
}

ISR(TIMER1_COMPA_vect)    // Przerwanie timera1 (porównanie rejestrów)
{
  digitalWrite(outPin, LOW);
  TIMSK1 &= ~(1 << OCIE1A); // turn off the timer interrupt
  TCCR1B &= ~(1 << WGM12);   // Tryb CTC (czyści licznik po porównaniu)
}

ISR(TIMER1_OVF_vect)
{
  times--;
  if (times < 0) {
    TIMSK1 &= ~(1 << TOIE1); // turn off the timer interrupt
    TCCR1B |= (1 << WGM12);   // Tryb CTC (czyści licznik po porównaniu)
    TIMSK1 |= (1 << OCIE1A);  // Przerwanie po porównaniu rejestrów
  }
}

ISR(TIMER2_COMPA_vect)  // Przerwanie timera0 (porównanie rejestrów)
{
  disp_off();  // Wyłączenie wyświetlacza
 
  switch (current_digit)
  {
    case 1:
      if (is_text) {
        disp_char(text[0]);
      }
      else {
        disp(count / 1000);   // Przygotowanie do wyświetlenia cyfry 1
      }
      digitalWrite(Dig1, LOW);  // Włączenie cyfry 1
      break;
 
    case 2:
      if (is_text) {
        disp_char(text[1]);
      }
      else {
        disp( (count / 100) % 10 );   // Przygotowanie do wyświetlenia cyfry 2
      }
      digitalWrite(Dig2, LOW);     // Włączenie cyfry 2
      break;
 
    case 3:
      if (is_text) {
        disp_char(text[2]);
      }
      else {
        disp( (count / 10) % 10 );   // Przygotowanie do wyświetlenia cyfry 3
      }
      digitalWrite(Dig3, LOW);    // Włączenie cyfry 3
      break;
 
    case 4:
      if (is_text) {
        disp_char(text[3]);
      }
      else {
        disp(count % 10);   // Przygotowanie do wyświetlenia cyfry 4
      }
      digitalWrite(Dig4, LOW);  // Włączenie cyfry 4
  }
 
  current_digit = (current_digit % 4) + 1;
}

// Przerwanie po odczycie ADC
ISR(ADC_vect){
  uint16_t ADC_RES_L = ADCL;
  uint16_t ADC_RES_H = ADCH;
  float Vcc_value = ( 0x400 * 1.1 ) / (ADC_RES_L + ADC_RES_H * 0x100) /* calculatethe Vcc value */;
  if (Vcc_value < min_voltage and !waitTurnOff) {
    EEPROM.put(address_high, highByte(milis));
    EEPROM.put(address_low, lowByte(milis));
    waitTurnOff = true;
    Serial.println(Vcc_value);
  } else if (Vcc_value > normal_voltage and waitTurnOff) {
    waitTurnOff = false;
    Serial.println(Vcc_value);
  }
}

void INT() {
  delay(10);
  if (millis() - last_time < 200) return;
  last_time = millis();
  if(digitalRead(interruptPin) == LOW)
  {
    set_timer(milis);
  }
}

void INT_encoder() {
  if(digitalRead(encoderPinB) == LOW)
  {
    new_milis = milis - 1;
  }
  else
  {
    new_milis = milis + 1;
  }
  milis = new_milis;
  if (milis < 1) {
    milis = 1;
  } else if (milis > 2000) {
    milis = 2000;
  }
  Serial.println(milis);
}

void disp(byte number, bool dec_point)
{
  switch (number)
  {
    case 0:  // print 0
      shiftOut(dataPin, clockPin, MSBFIRST, 0x02 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 1:  // print 1
      shiftOut(dataPin, clockPin, MSBFIRST, 0x9E | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 2:  // print 2
      shiftOut(dataPin, clockPin, MSBFIRST, 0x24 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 3:  // print 3
      shiftOut(dataPin, clockPin, MSBFIRST, 0x0C | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 4:  // print 4
      shiftOut(dataPin, clockPin, MSBFIRST, 0x98 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 5:  // print 5
      shiftOut(dataPin, clockPin, MSBFIRST, 0x48 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 6:  // print 6
      shiftOut(dataPin, clockPin, MSBFIRST, 0x40 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
    
    case 7:  // print 7
      shiftOut(dataPin, clockPin, MSBFIRST, 0x1E | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 8:  // print 8
      shiftOut(dataPin, clockPin, MSBFIRST, !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 9:  // print 9
      shiftOut(dataPin, clockPin, MSBFIRST, 0x08 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
  }
}

void disp_char(char ch, bool dec_point)
{
  switch (ch)
  {
    case 'O':  // Pisz O
      shiftOut(dataPin, clockPin, MSBFIRST, 0x02 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'I':  // Pisz I
      shiftOut(dataPin, clockPin, MSBFIRST, 0x9E | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'Z':  // Pisz Z
      shiftOut(dataPin, clockPin, MSBFIRST, 0x24 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'l':  // Pisz l
      shiftOut(dataPin, clockPin, MSBFIRST, 0xF2 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'x':  // Pisz ll
      shiftOut(dataPin, clockPin, MSBFIRST, 0x92 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'S':  // Pisz S
      shiftOut(dataPin, clockPin, MSBFIRST, 0x48 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'G':  // Pisz G
      shiftOut(dataPin, clockPin, MSBFIRST, 0x42 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
    
    case 'H':  // Pisz H
      shiftOut(dataPin, clockPin, MSBFIRST, 0x90 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
      break;
 
    case 'E':  // Pisz E
      shiftOut(dataPin, clockPin, MSBFIRST, 0x60 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case 'B':  // Pisz B
      shiftOut(dataPin, clockPin, MSBFIRST, 0x00 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case 'Y':  // Pisz Y
      shiftOut(dataPin, clockPin, MSBFIRST, 0x88 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case 'o':  // Pisz o
      shiftOut(dataPin, clockPin, MSBFIRST, 0xC4 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case 'N':  // Pisz N
      shiftOut(dataPin, clockPin, MSBFIRST, 0x12 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case '!':  // wykrzyknik
      shiftOut(dataPin, clockPin, MSBFIRST, 0x9E | dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case ' ':  // Pisz spację
      shiftOut(dataPin, clockPin, MSBFIRST, 0xFE | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    default:
      shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);
  }
}
 
void disp_off()
{
   digitalWrite(Dig1, HIGH);
   digitalWrite(Dig2, HIGH);
   digitalWrite(Dig3, HIGH);
   digitalWrite(Dig4, HIGH);
}
