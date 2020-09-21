long times = 0;
const byte interruptPin = 2;
const byte outPin = 13;
unsigned long last_time = 0;
long milis = 999;
unsigned long start_time = 0;
volatile int analogVal;

#define Dig1    8
#define Dig2    5
#define Dig3    4
#define Dig4    3

#define clockPin  7
#define dataPin   6

char text[5] = "HExO";

bool is_text = true;
byte current_digit;
int  count = 0;
void disp(byte number, bool dec_point = false);
void disp_char(char text, bool dec_point = false);

void setup() {
  noInterrupts();   // Wyłączeniue przerwań cli()

  pinMode(Dig1, OUTPUT);
  pinMode(Dig2, OUTPUT);
  pinMode(Dig3, OUTPUT);
  pinMode(Dig4, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  disp_off();  // Wyłączenie wyświetlacza
  
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPin), INT, CHANGE);  // Inicjalizacja przerwania zewnętrznego
  timer2_init();   // Inicjalizacja timera
  adc_init(); // Inicjalizacja ADC
  Serial.begin(9600);   // Inicjalizacja USART
  interrupts();   // Włączenie przerwań sei();
  start_time = millis();
  ADCSRA |=B01000000;
}

void loop() {
  if (millis() - start_time > 1000){
    is_text = false;
  }
  if (TIMSK1 & ((1 << OCIE1A)|(1 << TOIE1))) {
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
  // Zeruj bit ADLAR w ADMUX (0x7C)
  // ADCL będzie zawierał 8 młodszych bitów, a ADCH 2 starsze
  ADMUX &= B11011111;
 
  // Ustawia bity REFS1 i REFS0 w ADMUX (0x7C) w celu wyboru napięcia referencyjnego z nóżki AREF (5V)
  ADMUX |= B01000000;
 
  // Zeruje bity MUX3-0 w ADMUX (0x7C) w celu ustawienia wejścia A0
  ADMUX &= B11110000;
  
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
  analogVal = ADCL | (ADCH << 8); // Odczyt zmierzonej wartości
  long new_milis = (analogVal - 12)*2;
  if (new_milis - milis < 20 && milis - new_milis < 20){
    return;
  }
  milis = new_milis;
  if (milis < 1) {
    milis = 1;
  } else if (milis > 2000) {
    milis = 2000;
  }
  Serial.println(milis);
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

    case 'o':  // Pisz o
      shiftOut(dataPin, clockPin, MSBFIRST, 0xC4 | !dec_point);
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);

    case 'N':  // Pisz N
      shiftOut(dataPin, clockPin, MSBFIRST, 0x12 | !dec_point);
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
