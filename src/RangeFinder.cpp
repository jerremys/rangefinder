#include <Arduino.h>
#include <FastLED.h>
#include "symboles.h"
#include <string>
#include <vector>
#include <stdlib.h>
#include <TaskScheduler.h>

// Callback methods prototypes
void AnimateCallback();

// Tasks
Task animateTask(50, TASK_FOREVER, &AnimateCallback);
Scheduler runner;

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
#define _TASK_SLEEP_ON_IDLE_RUN

const int sleepMs = 50;
Task taskAnimate;

CRGB leds[NUM_LEDS];

enum POSITION
{
  NONE,
  FRONT,
  RIGHT,
  BACK,
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
void ShowSymbol(Symbols::SYMBOLS symbol);
void DumpLeds(bool matrix);
void ReverseColumns();
void ScrollUp();
void ScrollRight();
void ScrollLeft();
void AnimateCallback();
void Scroll(POSITION position);
void DoMeasure(Sensor *sensor);
void SetParkLight(bool lightOn);
bool IsCarPresent();
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

  ShowSymbol(Symbols::SYMBOLS::ARROW_RIGHT);

  runner.init();
  runner.addTask(animateTask);
  animateTask.enable();
};

void loop()
{
  runner.execute();

  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)

  for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
  {
    DoMeasure(&allSensors[i]);
    // PrintDistance(allSensors[i]);
  }

  SetParkLight(IsCarPresent());
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
}

void AnimateCallback()
{
  Scroll(RIGHT);
}

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
void DumpLeds(bool matrix)
{
  Serial.println("    0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F");
  Serial.println("    -------------------------------");
  for (int x = 0; x < NUM_LEDS; ++x)
  {
    if (x == 0)
    {
      Serial.print("00| ");
    }
    if (x > 0)
    {
      Serial.print(",");
      if (!(x % MATRIX_WIDTH))
      {
        Serial.println("");
        int row = x / MATRIX_HEIGHT;
        std::string rowP = row < 10 ? "0" + std::to_string(row) : std::to_string(row);
        rowP += "| ";
        Serial.print(rowP.c_str());
      }
    }
    Serial.print(ColorToIndex(leds[x]));
  }
  Serial.println();
  Serial.println("    -------------------------------");
  Serial.println("    0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F");
  Serial.println();
}

const char *toString(POSITION position)
{
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

void CearMatrix()
{
  for (int i = 0; i < NUM_LEDS; ++i)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

int ToLinearPosition(int x, int y)
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

Coordinate ToMatrixPosition(int idx)
{
  int y = (int)(idx / 16);
  int x = idx % MATRIX_WIDTH;

  if (y % 2)
  {
    x = MATRIX_WIDTH - x - 1;
  }

  return Coordinate(x, y);
}

void ShowSymbol(Symbols::SYMBOLS symbol)
{
  CearMatrix();

  auto selectedSymbol = Symbols::SYMBOL_MATRIX[(int)symbol];

  for (int x = 0; x < NUM_LEDS; ++x)
  {
    int row = x / MATRIX_HEIGHT;
    if (row % 2)
    {
      // Odd row
      // We need to reverse the position within the row for every odd row because
      // or the layout of the matrix
      auto column = x - (row * MATRIX_WIDTH);
      auto reversed = (((row - 1) * MATRIX_WIDTH) + MATRIX_WIDTH - column + MATRIX_WIDTH - 1);
      leds[reversed] = COLORS[selectedSymbol[x]];
    }
    else
    {
      leds[x] = COLORS[selectedSymbol[x]];
    }
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

void Scroll(POSITION position)
{
  switch (position)
  {
  case FRONT:
    ScrollUp();
    break;
  case RIGHT:
    ScrollRight();
    break;
  case BACK:
    break;
  case LEFT:
    ScrollLeft();
    break;
  }
}

void Flash()
{
}
void ScrollUp()
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
  DumpLeds(true);
}

void ScrollLeft()
{
  CRGB buffer[MATRIX_WIDTH];

  for (int i = 0; i <= NUM_LEDS - 1; i++)
  {
    if (i < MATRIX_WIDTH)
    {
      buffer[i] = leds[i];
    }
    else
    {
      leds[i - MATRIX_WIDTH] = leds[i];
    }
  }

  // Append the row that was moved off the left of the matrix to the right-most column
  for (int i = 0; i < MATRIX_WIDTH; ++i)
  {
    leds[NUM_LEDS - MATRIX_WIDTH + i] = buffer[i];
  }

  ReverseColumns();

  FastLED.show();
}

void ScrollRight()
{
  CRGB buffer[MATRIX_WIDTH];

  int lastColumn = NUM_LEDS - MATRIX_WIDTH - 1;
  int bufferCount = 0;

  for (int i = NUM_LEDS - 1; i >= 0 - 1; --i)
  {
    if (i > lastColumn)
    {
      buffer[bufferCount++] = leds[i];
    }
    else
    {
      leds[i + MATRIX_WIDTH] = leds[i];
    }
  }

  // Prepend the row that was moved off the right of the matrix to the left-most column
  for (int i = 0; i < MATRIX_WIDTH; ++i)
  {
    leds[i] = buffer[i];
  }

  ReverseColumns();

  FastLED.show();
}

void ReverseColumns()
{
  for (int i = 0; i < NUM_LEDS; i += MATRIX_WIDTH)
  {
    // Go through each column
    CRGB columnBuffer[MATRIX_WIDTH];

    for (int j = 0; j < MATRIX_WIDTH; ++j)
    {
      // Read the column into a buffer
      columnBuffer[j] = leds[i + j];
    }

    // Re-add the elements in reverse order
    for (int k = MATRIX_WIDTH - 1, count = 0; k >= 0; --k, ++count)
    {
      leds[i + k] = columnBuffer[count];
    }
  }
}

void TestPattern()
{
  int ct = 0;
  while (1)
  {
    leds[ct++] = CRGB::Red;
    FastLED.show();
    delay(1000);
  }
}
