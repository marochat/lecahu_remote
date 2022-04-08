// Remote control for room LED light LEC-AHxxU

#include <Arduino.h>

#define RAW_BUFFER_LENGTH 256

#include "PinDefinitionsAndMore.h"

#include <IRremote.hpp>

#define BLUE_LED digitalRead(A0)
#define RED_LED digitalRead(A1)
#define SET_BLUE_LED(x) digitalWrite(A0, x)
#define SET_RED_LED(x) digitalWrite(A1, x)

#define UPPER_SENS digitalRead(A2)
#define LOWER_SENS digitalRead(A3)

#define ANALOGIN A5

#define HIGH_TH 0x180
#define LOW_TH 0x100

bool readmode = false;
bool light_state = false;
bool upper_state = false;
bool lower_state = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, OUTPUT);  // LED BLUE
  pinMode(A1, OUTPUT);  // LED RED
  pinMode(A2, INPUT);   // Upper Sensor
  pinMode(A3, INPUT);   // Lower Sendor
  pinMode(A4, INPUT);   // Remo Mode
  //pinMode(A5, INPUT);   // Analog input

  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);    // IR RECEIVE
  pinMode(3, OUTPUT);   // IR SEND

    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  if(digitalRead(A4)) readmode = true;
  if(readmode){
    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED    
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.print(F("at pin "));
    Serial.println(IR_RECEIVE_PIN);
  } else {
    IrSender.begin(IR_SEND_PIN);
    Serial.print(F("Sensor mode!\n"));
    //delay(2000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(readmode){
    if (IrReceiver.decode()) {  // Grab an IR code
        // Check if the buffer overflowed
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            Serial.println(F("Overflow detected"));
            Serial.println(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
            // see also https://github.com/Arduino-IRremote/Arduino-IRremote#modifying-compile-options-with-sloeber-ide
        } else {
            Serial.println();                               // 2 blank lines between entries
            Serial.println();
            IrReceiver.printIRResultShort(&Serial);
            Serial.println();
            Serial.println(F("Raw result in internal ticks (50 us) - with leading gap"));
            IrReceiver.printIRResultRawFormatted(&Serial, false); // Output the results in RAW format
            Serial.println(F("Raw result in microseconds - with leading gap"));
            IrReceiver.printIRResultRawFormatted(&Serial, true);  // Output the results in RAW format
            Serial.println();                               // blank line between entries
            Serial.print(F("Result as internal ticks (50 us) array - compensated with MARK_EXCESS_MICROS="));
            Serial.println(MARK_EXCESS_MICROS);
            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, false); // Output the results as uint8_t source code array of ticks
            Serial.print(F("Result as microseconds array - compensated with MARK_EXCESS_MICROS="));
            Serial.println(MARK_EXCESS_MICROS);
            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true); // Output the results as uint16_t source code array of micros
            IrReceiver.printIRResultAsCVariables(&Serial);  // Output address and data as source code variables

            IrReceiver.compensateAndPrintIRResultAsPronto(&Serial);
        }
        IrReceiver.resume();                            // Prepare for the next value
    }
  } else {    // Sensor mode.
    char buf[5];
    int lsens = analogRead(ANALOGIN);
    sprintf(buf, "lsens : %04X", lsens);
    Serial.println(buf);
    if(lsens > HIGH_TH) {
      light_state = true;
      delay(500);
    } else if(lsens < LOW_TH) {
      if(light_state){
        Serial.println(F("LIGHT OFF!\n"));
        remote_off();
        light_state = false;
        delay(5000);
      }
      delay(500);
    }
    if(!light_state){
      if(UPPER_SENS) {
        SET_BLUE_LED(HIGH);
        delay(1500);
        if(UPPER_SENS){
          Serial.println(F("LIGHT ON!\n"));
          remote_on();
          delay(1000);
        }
      } else {
        SET_BLUE_LED(LOW);
      }
    } else {
      SET_BLUE_LED(LOW);
    }
    delay(100);
  }
}

void remote_on(){
    uint16_t rawData[179] = {
      3230,1720, 380,1270, 380,470, 380,420, 480,370, 430,420, 430,420,
      430,420, 380,420, 430,420, 430,420, 430,420, 380,470, 380,1270,
      380,470, 380,420, 430,420, 430,420, 430,420, 380,470, 380,420,
      430,1270, 380,1270, 430,420, 380,470, 380,420, 430,420, 430,420,
      430,420, 380,1270, 430,420, 380,1270, 430,420, 430,1220, 430,1270,
      380,1270, 380,1270, 430,420, 380,1270, 430,420, 430,1220, 430,420,
      430,420, 430,420, 380,470, 380,420, 430,420, 430,420, 380,470, 380,1270,
      430,1220, 430,1270, 380,1270, 380,1320, 380,1270, 380,1270, 380,1270,
      430,420, 430,420, 380,1320, 380,420, 380,470, 380,420, 430,420, 430,420,
      430,1270, 380,1270, 380,470, 380,1270, 380,1270, 430,1270, 380,1270,
      380,1320, 380,420, 430,1270, 380,420, 430,420, 380,470, 380,470, 380,1270,
      380,1270, 430,1270, 380,470, 380,1270, 380,1270, 380,1270, 430,1270,
      380,420, 430,420, 380
    };  // Protocol=PULSE_DISTANCE Address=0x0 Command=0x0 Raw-Data=0x3DC2FB 88 bits LSB first
    IrSender.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), 38);
}

void remote_off(){
    uint16_t rawData[179] = {
      3280,1670, 430,1270, 430,370, 430,420, 430,420, 380,470, 380,420,
      380,470, 380,470, 380,470, 380,470, 380,420, 380,470, 380,1270,
      380,470, 380,470, 380,470, 380,420, 430,420, 380,470, 380,470,
      380,1270, 380,1270, 430,420, 380,470, 380,470, 380,420, 430,420,
      430,420, 380,1270, 430,420, 380,1270, 430,420, 430,1220, 430,1270,
      380,1270, 380,1270, 430,420, 380,1270, 430,420, 380,1270, 430,420,
      430,420, 380,470, 380,420, 430,420, 430,420, 380,470, 380,470, 380,1270,
      380,1270, 430,1220, 430,1270, 380,1270, 380,1270, 430,1220, 430,1270,
      380,1270, 430,1220, 430,420, 430,420, 380,470, 380,420, 430,420, 430,420,
      430,420, 380,470, 380,1270, 380,1270, 430,1270, 380,1270, 380,1270, 380,1320,
      380,420, 430,1270, 380,470, 380,420, 380,470, 380,470, 380,1270, 380,1270,
      430,1270, 380,420, 430,1270, 380,1270, 380,1270, 380,1320, 380,420,
      380,470, 380
    };  // Protocol=PULSE_DISTANCE Address=0x0 Command=0x0 Raw-Data=0x3DC2FC 88 bits LSB first
    IrSender.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), 38);
}
