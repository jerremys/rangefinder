#include <Arduino.h>
#include <FastLED.h>
#include "symboles.h"

#define LED_PIN 13
#define BRIGHTNESS 2
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define INDICATOR_PIN 16

#define MATRIX_HEIGHT 16
#define MATRIX_WIDTH 16
#define NUM_LEDS MATRIX_HEIGHT *MATRIX_WIDTH

#define CM_CONVERSION 29.1
#define NUMBER_OF_SENSORS 2

#define PARK_MARGIN 30

CRGB leds[NUM_LEDS];

enum POSITION
{
  NONE,
  FRONT,
  LEFT
};

struct Sensor
{
  POSITION position;
  int triggerPin;
  int echoPin;
  float distanceCm;
  int parkDistance;
  int parkMaxDistance;
};

const int COLORS[] = {
    0x000000,
    0x70ff70, // Light green
    0x00ff00, // Green
    0xffee80, // Light yellow
    0xc2a800, // Yellow
    0xff4f4d, // Light red
    0xc70300  // Red
};

Sensor allSensors[NUMBER_OF_SENSORS] = {
    {.position = FRONT, .triggerPin = 14, .echoPin = 12, .distanceCm = 0, .parkDistance = 90, .parkMaxDistance = 90 + PARK_MARGIN},
    {.position = LEFT, .triggerPin = 4, .echoPin = 5, .distanceCm = 0, .parkDistance = 90, .parkMaxDistance = 90 + PARK_MARGIN}};

int trigPinX = 14; // Trigger
int echoPinX = 12; // Echo

int trigPinY = 4; // Trigger
int echoPinY = 5; // Echo

long duration, cm;
float inches, feet;
unsigned int annimationCount = 0;

CRGB leftBuffer[MATRIX_HEIGHT];

struct Coordinate
{
  unsigned char x;
  unsigned char y;

  Coordinate(unsigned char xTmp, unsigned char yTmp)
  {
    x = xTmp;
    y = yTmp;
  }
};

//-----------------------------------------------------------------------------
// Forward declarations
void showSymbol(Symbols::SYMBOLS symbol);
void dumpLeds(bool matrix);
//-----------------------------------------------------------------------------

void setup()
{
  // Serial Port begin
  Serial.begin(921600);
  Serial.println("Starting New");

  // Define inputs and outputs
  for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
  {
    pinMode(allSensors[i].triggerPin, OUTPUT);
    pinMode(allSensors[i].echoPin, INPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, HIGH);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  showSymbol(Symbols::SYMBOLS::ARROW_LEFT);
};

int ColorToIndex(CRGB hex)
{
  int arraySize = sizeof(COLORS) / sizeof(COLORS[0]);
  int idx = -1;

  for (int i = 0; i < arraySize; ++i)
  {
    if (COLORS[i] == hex)
    {
      idx = i;
      break;
    }
  }

  return idx;
}

//------------------------------------------------------------------------------
void dumpLeds(bool matrix)
{
  Serial.println("0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F");
  for (int x = 0; x < NUM_LEDS; ++x)
  {
    if (x > 0)
    {
      Serial.print(",");
      if (!(x % MATRIX_WIDTH))
        Serial.println("");
    }
    Serial.print(ColorToIndex(leds[x]));
  }
  Serial.println();
}

const char *toString(POSITION position)
{
  char pos[6];
  switch (position)
  {
  case NONE:
    return "None";
  case FRONT:
    return "Front";
  case LEFT:
    return "Left";
  default:
    return "Error";
  }
}

bool IsCarPresent()
{
  bool parked = true;
  for (int i = 0; parked && i < NUMBER_OF_SENSORS; ++i)
  {
    if (false)
    {
      Serial.print("test[");
      Serial.print(toString(allSensors[i].position));
      Serial.print(",");
      Serial.print(allSensors[i].distanceCm);
      Serial.print(",");
      Serial.print(allSensors[i].parkMaxDistance);
      Serial.print(",");
      Serial.print(allSensors[i].distanceCm < allSensors[i].parkMaxDistance);
      Serial.println("]");
    }
    parked &= allSensors[i].distanceCm != 0 && allSensors[i].distanceCm < allSensors[i].parkMaxDistance;
  }

  return parked;
}

void SetParkLight(bool lightOn)
{
  digitalWrite(INDICATOR_PIN, lightOn ? HIGH : LOW);
}

void DoMeasure(Sensor *sensor)
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(sensor->triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sensor->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->triggerPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(sensor->echoPin, INPUT);
  sensor->distanceCm = (pulseIn(sensor->echoPin, HIGH) / 2) / CM_CONVERSION;
}

void clearMatrix()
{
  for (int i = 0; i < NUM_LEDS; ++i)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

int toLinearPosition(int x, int y)
{
  if (!!(y % 2))
  {
    return (((y - 1) * MATRIX_WIDTH) + MATRIX_WIDTH - x + MATRIX_WIDTH - 1);
  }
  else
  {
    return (x + (y * MATRIX_HEIGHT));
  }
}

Coordinate toMatrixPosition(int idx)
{
  int y = (int)(idx / 16);
  int x = idx % MATRIX_WIDTH;

  if (y % 2)
  {
    x = MATRIX_WIDTH - x - 1;
  }

  return Coordinate(x, y);
}

void showSymbol(Symbols::SYMBOLS symbol)
{
  auto color = CRGB::Green;
  clearMatrix();

  auto selectedSymbol = Symbols::SYMBOL_MATRIX[(int)Symbols::SYMBOLS::ARROW_LEFT];

  for (int x = 0; x < NUM_LEDS; ++x)
  {
    leds[x] = COLORS[selectedSymbol[x]];
  }

  FastLED.show();
}

void PrintDistance(Sensor sensor)
{
  auto inches = sensor.distanceCm / 2.54;
  feet = inches / 12;

  if (true)
  {
    Serial.print("[");
    Serial.print(toString(sensor.position));
    Serial.print("]=[");
    Serial.print(sensor.distanceCm);
    Serial.print("cm, ");
    Serial.print(inches);
    Serial.print("in, ");
    Serial.print(feet);
    Serial.println("ft]");
  }
}

void scrollUp()
{
  for (int i = 0; i <= NUM_LEDS - 1; i++)
  {
    if (i < MATRIX_WIDTH)
    {
      leftBuffer[i] = leds[i];
    }
    else
    {
      leds[i - MATRIX_WIDTH] = leds[i];
    }
  }
  dumpLeds(true);
}

void scrollLeft()
{
  CRGB leftBuffer[MATRIX_WIDTH];

  for (int i = 0; i <= NUM_LEDS - 1; i++)
  {
    if (i < MATRIX_WIDTH)
    {
      leftBuffer[i] = leds[i];
    }
    else
    {
      leds[i - MATRIX_WIDTH] = leds[i];
    }
  }

  for (int i = 0; i < MATRIX_WIDTH; ++i)
  {
    leds[NUM_LEDS - MATRIX_WIDTH + i] = leftBuffer[i];
  }

  FastLED.show();
}

void TestPattern()
{
  int ct = 0;
  while (1)
  {
    leds[ct++] = CRGB::Red;
    FastLED.show();
    delay(100);
  }
}

void *thread(void *ptr)
{
  int type = (int)ptr;
  fprintf(stderr, "Thread - %d\n", type);
  return ptr;
}
void loop()
{
  if (++annimationCount > 2)
  {
    Serial.println("BEFORE");
    dumpLeds(true);
    scrollLeft();
    Serial.println("AFTER");
    dumpLeds(true);
  }

  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)

  for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
  {
    DoMeasure(&allSensors[i]);
    PrintDistance(allSensors[i]);
  }

  SetParkLight(IsCarPresent());
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW

  delay(1000);
}
