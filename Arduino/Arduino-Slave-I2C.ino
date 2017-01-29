#include <Wire.h>
#include "FastLED.h"
#include "Timer.h"

#define NUM_LEDS 265
#define DATA_PIN 6
#define COLOR_ORDER GRB

Timer t;

char ch;
CRGB leds[NUM_LEDS];
enum systemModeEnum {Off, Red, Blue, Green, Pink, Teal, Rainbow, Blink} currentSystemMode = Rainbow;
systemModeEnum lastSystemMode = Off;
int commandReceivedLedOnboardArduino = 13;

bool blinkOn = false;

void toggleCommandArrivedLed(int LightDuration = 200)
{
  digitalWrite(commandReceivedLedOnboardArduino, HIGH);
  delay(LightDuration);
  digitalWrite(commandReceivedLedOnboardArduino, LOW);
  delay(LightDuration);
}

void processCommand(char theCh) {
  Serial.print("Recieved Command ");
  Serial.print(theCh);
  Serial.print("\n");
  switch (theCh)
  {
    case 'r':
      currentSystemMode = Red;
      Serial.write("Commanded RED. \n");
      break;

    case 'g':
      currentSystemMode = Green;
      Serial.write("Commanded GREEN. \n");
      break;

    case 'b':
      currentSystemMode = Blue;
      Serial.write("Commanded BLUE. \n");
      break;

    case 'p':
      currentSystemMode = Pink;
      Serial.write("Commanded PINK. \n");
      break;

    case 't':
      currentSystemMode = Teal;
      Serial.write("Commanded TEAL. \n");
      break;

    case 'n':
      currentSystemMode = Off;
      Serial.write("Commanded BLANK. \n");
      break;

    case 'a':
      currentSystemMode = Rainbow;
      Serial.write("Commanded RAINBOW. \n");
      break;

    case 'o':
      currentSystemMode = Blink;
      Serial.write("Commanded BLINK. \n");
      break;

    default:
      Serial.write("Commanded not recognized. \n");
      break;

  }
}

void rainbow(uint8_t wait)
{
  uint16_t hue;
  FastLED.clear();
  for (hue = 10; hue < 255 * 3; hue++)
  {
    fill_rainbow( &(leds[0]), NUM_LEDS /*led count*/, hue /*starting hue*/);
    FastLED.show();
  }
  return;
}
void processCurrentMode() {
  switch (currentSystemMode) {

    case Blink:
      blinkOn = !blinkOn;
      if (currentSystemMode == Blink) {
        if (blinkOn) {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 255, 255) );
          FastLED.show();
        }
        else {
          FastLED.clear();
          fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
          FastLED.show();
        }
      }
      break;
      
    case Red:
      if (lastSystemMode != Red) {
        FastLED.clear();
        fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 0, 0) );
        FastLED.show();
        lastSystemMode = Red;
      }
      break;
      
    case Green:
      if (lastSystemMode != Green) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 255, 0) );
      FastLED.show();
      lastSystemMode = Green;
      }
      break;

    case Blue:
      if (lastSystemMode != Blue) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 255) );
      FastLED.show();
      lastSystemMode = Blue;
      }
      break;

    case Pink:
      if (lastSystemMode != Pink) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 255, 20, 147) );
      FastLED.show();
      lastSystemMode = Pink;
      }
      break;

    case Teal:
      if (lastSystemMode != Teal) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 255, 255) );
      FastLED.show();
      lastSystemMode = Teal;
      }
      break;

    case Off:
      if (lastSystemMode != Off) {
      FastLED.clear();
      fill_solid( leds, NUM_LEDS /*number of leds*/, CRGB( 0, 0, 0) );
      FastLED.show();
      lastSystemMode = Off;
      }
      break;

    case Rainbow:
      if (lastSystemMode != Rainbow) {
      FastLED.clear();
      rainbow(0);
      FastLED.show();
      lastSystemMode = Rainbow;
      }
      break;

    default:
      Serial.print("Unrecognized Mode");
      break;

  }
}
void setup()
{
  FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  Wire.begin(1); //communicate on this address
  Wire.onReceive(receiveEvent);//when communication is successful,
  Serial.begin(9600);          //go to the function
  FastLED.show();
  t.every(1000, processCurrentMode);
  pinMode(commandReceivedLedOnboardArduino, OUTPUT);
  toggleCommandArrivedLed();
  Serial.write("Arduino-Slave-I2C-NewLEDStrip Initialization Complete. \n");
}

void loop()
{
  t.update();
  if (Serial.available()) {
    ch = Serial.read();
    processCommand(ch);
  }

}

void receiveEvent(int howMany)
{
  Serial.write("receiveEvent Invoked. \n");
  while (Wire.available()) //when you recieve a byte (through i2c)
  {
    toggleCommandArrivedLed();

    unsigned char state = Wire.read();
    Serial.write("Received Char: <");
    Serial.write(state);
    Serial.write("> \n");
    //not sure if it should be a normal or unsigned char

    processCommand(state);
  }
}
