
/////////////////////////////////////////////////////////////////////////////////////////
////////////////     Luxmetre Oregon basé sur UVN800 ////////////////////////////////////
////////////////         Programme pour ATTiny85 ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//                 _______
//                |°      |
//             ---|1     8|--- VCC+
// resetbouton ---|2     7|--- photoresistance
//    TX DATA  ---|3     6|--- LED_PIN
//         GND ---|4     5|--- VCC sur transistor pour photoresistance, TX 433, LED
//                |_______|
//
//
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

//set bit macro

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define LED_PIN 1
#define TX_PIN 4
#define Oregon_FRAME_LENGTH 19
#define Oregon_SHORT 500
#define Oregon_LONG 1000
#define LUX_PIN 1
#define LED_ON digitalWrite(LED_PIN, HIGH)
#define LED_OFF  digitalWrite(LED_PIN, LOW)
#define toggleTX digitalWrite(TX_PIN, !digitalRead(TX_PIN))
#define VCC 0
const unsigned int frequence = 3;//MINUTES  -----!!MAX 30!!-------environ le temps entre 2 envois (en minute)
const unsigned int resetbouton = 3;
volatile bool stateBt = 0;
volatile unsigned int f_wdt = 0;
const unsigned int RC = 10;  //position du rollingcode sur EEPROM
volatile byte code[19] = { 0xA, //   >
                           0xD, //     > <------- code [0 - 3] = Device ID
                           0x8, //   >
                           0x7, // >
                           0x4, //  <------------ code[4] = Channel 1,2 or 4
                           0x1, //  <------------ code[5] = Batterie ???
                           0xA, //  <------------ code[6] = rolling code
                           0xC, //  <------------ code[7] = rolling code
                           0x8, //  <------------ code[8]
                           0x5, //  <------------ code[9] = mesure UV  MAX : 0xA
                           0x0, //  <------------ code[10]
                           0x0, //  <------------ code[11]
                           0x7, //  <------------ code[12]
                           0x0, //  <------------ code[13]
                           0x0, //  <------------ code[14]= checksum code[1 à 13]
                           0x0, //  <------------ code[15]= checksum code[1 à 13]
                           0x0, //
                           0xF, //
                           0x2
                         };//



void sendOregon() {
  LED_ON;
  byte provisoir;
  bool currentbit;
  unsigned long periode = micros();
  digitalWrite(TX_PIN, HIGH);
  int i = 0;
  while (  i < 48 ) {
    if ((micros() - periode) > Oregon_SHORT) {
      periode = micros();
      toggleTX;
      ++i;
    }
  }
  // Code //////////////////////////////////////////
  i = 0;
  while (  i < Oregon_FRAME_LENGTH ) {
    provisoir = code[i];
    int j = 0;
    while (j < 4) {
      if ((micros() - periode) > Oregon_SHORT ) {
        toggleTX;
        currentbit = ( provisoir  & ( 0x1 << j ));
        while ( (micros() - periode) < Oregon_LONG ) {};
        periode = micros();
        if  (currentbit == 0) digitalWrite(TX_PIN, LOW );
        if  (currentbit == 1) digitalWrite(TX_PIN, HIGH );
        ++j;
      }
    }
    ++i;
  }
  while ( (micros() - periode) < Oregon_SHORT ) {};
  toggleTX;
  while ( (micros() - periode) < Oregon_SHORT ) {};
  digitalWrite(TX_PIN, HIGH);
  LED_OFF;
}










void checksum() {
  byte s = 0;
  for (int i = 1; i < 14; ++i) {
    s += code[i];
  }
  code[14] = s & 0xF;
  code[15] = (s & 0xF0) >> 4 ;
}

void resetRollingcode() {

  randomSeed(micros());
  byte newcode = random(0 , 0xFF);
  EEPROM.put(RC, newcode);
  code[6] = ((newcode & 0xF) & 0xF);
  code[7] = (((newcode & 0xF0) >> 4) & 0xF);

  for (int i = 0; i < 10; ++i) {
    LED_ON;
    delay(50);
    LED_OFF;
    delay(500);
  }

  delay(500);
}

void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here, waiting for interrupt
 
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}




void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;

  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt++;  // set global flag
}
ISR(PCINT0_vect) {     //All interrupts direct to this ISR function
    f_wdt = 255;
    stateBt = 1;
}
void setup(){ 
  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  setup_watchdog(9);
  pinMode(TX_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(VCC, OUTPUT);
  pinMode(resetbouton, INPUT);
  digitalWrite(resetbouton, HIGH);    // Activate internal pullup resistor
  PCMSK  |= bit (PCINT3);  //                                   Pin Change Mask Register
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts  General Interrupt Flag Register
  GIMSK  |= bit (PCIE);    // enable pin change interrupts      General Interrupt Mask Register

  byte getcode;
  EEPROM.get(RC, getcode); // recuperation du rolling code en memoire eeprom
  code[6] = ((getcode & 0xF) & 0xF);
  code[7] = (((getcode & 0xF0) >> 4) & 0xF);
  sei();                   // enable interrupts
}

void loop() {

  if (stateBt) {
    resetRollingcode();
    stateBt = 0;
  }


  digitalWrite(VCC, HIGH);
  unsigned int  reading  = analogRead(LUX_PIN) ;
  if (reading < 20) reading = 0;
  else if (reading < 50) reading = 1;
  else if (reading < 110) reading = 2;
  else if (reading < 200) reading = 3;
  else if (reading < 300) reading = 4;
  else  reading = 5 + ((reading - 300) / 120) ;
  if (reading > 10 ) reading = 10;
  code[9] = reading;
  checksum();
  sendOregon();
  digitalWrite(VCC, LOW);
  while (f_wdt < (60 / 8 * frequence)) {// Wait  watchdog interupts (~10secs)
    system_sleep();  
  }
  f_wdt = 0;     // reset flag
  
}

/*  code[]:
  - 0-3: Device ID. The ID for THGN132N sensors is  1A2D .
  - 4: Channel. This corresponds to the channel slider on the back of the sensor ( 1, 2, or 4 for channel 1, 2 or 3).
  - 5: Battery? All of my readings have 0 for this nibble. I'm half-expecting it to become non-zero on low battery.
  - 6-7: Rolling code. This is a unique identifier for the sensor. It resets when the battery is replaced.
  - 8: The tenths digit of the temperature.
  - 9: The unit of UVI
  - 10: The tens digit of the temperature.
  - 11: The unit digit of the temperature.
  - 12: The unit digit of the humidity.
  - 13: The sign for the temperature. This nibble will be 0 for a +ve temp, and non-zero for -ve. During my testing with the sensor in the freezer, I've only seen this return 0 or 8.
  - 15: The tens digit of the humidity.

  --------------------------------------------------------
  |  Sensor Name      |  Code   | Type                   |
  --------------------------------------------------------
  |  Oregon-THR128    |         |                        |
  |  Oregon-THR138    | 0x0A4D  | Inside Temperature     |A0D4
  |  Oregon-THC138    |         |                        |
  --------------------------------------------------------
  | Oregon-THC238     |         |                        |
  | Oregon-THC268     |         |                        |
  | Oregon-THN132N    |         |                        |
  | Oregon-THWR288A   | 0xEA4C  | Outside/Water Temp     |AEC4
  | Oregon-THRN122N   |         |                        |
  | Oregon-THN122N    |         |                        |
  | Oregon-AW129      |         |                        |
  | Oregon-AW131      |         |                        |
  --------------------------------------------------------
  | Oregon-THWR800    | 0xCA48  | Water Temp             |AC84
  --------------------------------------------------------
  | Oregon-THGN122N   |         |                        |
  | Oregon-THGN123N   |         |                        |
  | Oregon-THGR122NX  | 0x1A2D  | Inside Temp-Hygro      |A1D2
  | Oregon-THGR228N   |         |                        |
  | Oregon-THGR238    |         |                        |
  | Oregon-THGR268    |         |                        |
  --------------------------------------------------------
  | Oregon-THGR810    | 0xFA28  | Inside Temp-Hygro      |AF82
  | Oregon-RTGR328N   | 0x*ACC  | Outside Temp-Hygro     |
  | Oregon-THGR328N   | 0xCA2C  | Outside Temp-Hygro     |ACC2
  | Oregon-WTGR800    | 0xFAB8  | Outside Temp-Hygro     |AF8B
  --------------------------------------------------------
  | Oregon-THGR918    |         |                        |
  | Oregon-THGRN228NX | 0x1A3D  | Outside Temp-Hygro     |A1D3
  | Oregon-THGN500    |         |                        |
  --------------------------------------------------------
  | Huger - BTHR918   | 0x5A5D  | Inside Temp-Hygro-Baro |A5D5
  --------------------------------------------------------
  | Oregon-BTHR918N   |         |                        |
  | Oregon-BTHR968    | 0x5A6D  | Inside Temp-Hygro-Baro |A6D6
  --------------------------------------------------------
  | Oregon-RGR126     |         |                        |
  | Oregon-RGR682     | 0x2A1D  | Rain Gauge             |A2D1
  | Oregon-RGR918     |         |                        |
  --------------------------------------------------------
  | Oregon-PCR800     | 0x2A19  | Rain Gauge             |A291
  | Oregon-WTGR800    | 0x1A99  | Anemometer             |A199
  | Oregon-WGR800     | 0x1A89  | Anemometer             |A189
  --------------------------------------------------------
  | Huger-STR918      |         |                        |
  | Oregon-WGR918     | 0x3A0D  | Anemometer             |A3D0
  --------------------------------------------------------
  | Oregon-UVN128     |         |                        |
  | Oregon-UV138      | 0xEA7C  | UV sensor              |AEC7
  --------------------------------------------------------
  | Oregon-UVN800     | 0xDA78  | UV sensor              |DA87
  | Oregon-RTGR328N   | 0x*AEC  | Date & RF_BIT_TIME            |
  --------------------------------------------------------
  | cent-a-meter      |         |                        |
  | OWL CM113         | 0xEAC0  | Ampere meter           |AE0C
  | Electrisave       |         |                        |
  --------------------------------------------------------
  | OWL CM119         | 0x1A*                            |
  |                   | 0x2A*   | Power meter            |A1**
  |                   | 0x3A**  |                        |
  --------------------------------------------------------*/
