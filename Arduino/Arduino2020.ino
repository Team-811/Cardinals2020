#include <FastLED.h>

/*
   Team 811 2020 LED Code
   This LED code drives 3 separate LED strips of different lengths. There is a strip for the shooter, climber, and intake drum
*/

//define which pins on the Arduino will be used for the 3 strips
#define PIN1    3
#define PIN2    4
#define PIN3    5

//length of each strip
#define intakeLength    157
#define shooterLength   20
#define climberLength   33

//create an LED strip object for each strip
CRGB intake[intakeLength];
CRGB shooter[shooterLength];
CRGB climber[climberLength];

//setup the Serial connection to the RoboRio and initialize the LED strips to their respective pins and lengths
void setup()
{
  delay(200);
  Serial.begin(9600);
  Serial.setTimeout(5);
  delay(2000);
  FastLED.addLeds<NEOPIXEL, PIN1>(climber, climberLength);
  FastLED.addLeds<NEOPIXEL, PIN2>(shooter, shooterLength);
  FastLED.addLeds<NEOPIXEL, PIN3>(intake, intakeLength);

  FastLED.setBrightness(100);
  FastLED.show();
}

//this main loop method deals with setting the LED pattern based on the string sent by the RoboRio
int patternCode = 0;
String sData = "";
String metadata = "";
void loop()
{
  //see if new data has been sent
  while (Serial.available() > 0)
  {
    sData = Serial.readString();
  }

  int firstComma = sData.indexOf(",");

  //the pattern ID is the first number in the string before the comma
  patternCode = sData.substring(0, firstComma).toInt();

  //the rest of the string after the first number contains any values a pattern requires
  metadata = sData.substring(firstComma + 1);

  //set the pattern
  switch (patternCode)
  {
    case 1:
      {
        Rainbow(metadata.toInt());
        break;
      }
    case 2:
      {
        RainbowChase(metadata.toInt());
        break;
      }
    case 3:
      {
        FillUp(metadata.substring(0, metadata.indexOf(",")).toInt(),
               metadata.substring(metadata.indexOf(",") + 1).toInt());
        break;
      }
    case 4:
      {
        Blink(metadata.substring(0, metadata.indexOf(",")).toInt(),
              metadata.substring(metadata.indexOf(",") + 1).toInt());
        break;
      }
    case 5:
      {
        String metadata2 = metadata.substring(metadata.indexOf(",") + 1);
        MovingGradient(metadata.substring(0, metadata.indexOf(",")).toInt(),
                       metadata2.substring(0, metadata2.indexOf(",")).toInt(),
                       metadata2.substring(metadata2.indexOf(",") + 1).toInt());
        break;
      }
    case 6:
      {
        String metadata2 = metadata.substring(metadata.indexOf(",") + 1);
        ColorChase(metadata.substring(0, metadata.indexOf(",")).toInt(),
                   metadata2.substring(0, metadata2.indexOf(",")).toInt(),
                   metadata2.substring(metadata2.indexOf(",") + 1).toInt());
        break;
      }
    default:
      {
        Rainbow(0);
        break;
      }
  }
}

/*
   All integers for colors are for the HSV value. The saturation and brightness are always set to full
*/

//Smooth shifting rainbow cycle with adjustable speed
uint8_t gHue = 0;
void Rainbow(int del)
{
  fill_rainbow(shooter, shooterLength, gHue, 3);
  fill_rainbow(intake, intakeLength, gHue, 3);
  fill_rainbow(climber, climberLength, gHue, 3);
  gHue++;
  if (gHue > 255)
  {
    gHue += 1;
  }
  delay(del);
  FastLED.show();
}

//Two-dot chase that cycles through the rainbow w/adjustable speed
void RainbowChase(int del)
{
  for (int j = 0; j < 256; j += 6)
  {
    for (int q = 0; q < 5; q++)
    {
      if (Serial.available() > 0)
        break;
      for (uint16_t i = 0; i < intakeLength; i = i + 5)
      {
        if (Serial.available() > 0)
          break;
        intake[i + q] = CHSV(j, 255, 255);
        intake[i + q + 1] = CHSV(j, 255, 255);
      }
      for (uint16_t i = 0; i < shooterLength; i = i + 5)
      {
        if (Serial.available() > 0)
          break;
        shooter[i + q] = CHSV(j, 255, 255);
        shooter[i + q + 1] = CHSV(j, 255, 255);
      }
      for (uint16_t i = 0; i < climberLength; i = i + 5)
      {
        if (Serial.available() > 0)
          break;
        climber[i + q] = CHSV(j, 255, 255);
        climber[i + q + 1] = CHSV(j, 255, 255);
      }

      delay(del);

      for (uint16_t i = 0; i < intakeLength; i = i + 5)
      {
        intake[i + q - 1] = CRGB(0, 0, 0);
        if (Serial.available() > 0)
          break;
      }
      for (uint16_t i = 0; i < shooterLength; i = i + 5)
      {
        shooter[i + q - 1] = CRGB(0, 0, 0);
        if (Serial.available() > 0)
          break;
      }
      for (uint16_t i = 0; i < climberLength; i = i + 5)
      {
        climber[i + q - 1] = CRGB(0, 0, 0);
        if (Serial.available() > 0)
          break;
      }
      FastLED.show();
    }
  }
}

//Fill and clear the strips with one color at a specified speed
int s = shooterLength;
int i = intakeLength;
int c = climberLength;
void FillUp(int color, int del)
{
  if (s >= shooterLength)
  {
    s = 0;
    fill_solid(shooter, shooterLength, CRGB(0, 0, 0));
  }
  if (i >= intakeLength)
  {
    i = 0;
    fill_solid(intake, intakeLength, CRGB(0, 0, 0));
  }
  if (c >= climberLength)
  {
    c = 0;
    fill_solid(climber, climberLength, CRGB(0, 0, 0));
  }

  shooter[s] = CHSV(color, 255, 255);
  s++;

  for (int j = 0; j < 3; j++)
  {
    intake[i + j] = CHSV(color, 255, 255);
  }
  i += 3;

  for (int j = 0; j < 1; j++)
  {
    climber[c + j] = CHSV(color, 255, 255);
  }
  c += 1;

  delay(del);
  FastLED.show();
}

//Blink one color at given delay
void Blink(int color, int del)
{
  fill_solid(intake, intakeLength, CRGB(0, 0, 0));
  fill_solid(shooter, shooterLength, CRGB(0, 0, 0));
  fill_solid(climber, climberLength, CRGB(0, 0, 0));
  FastLED.show();

  delay(del);

  fill_solid(intake, intakeLength, CHSV(color, 255, 255));
  fill_solid(shooter, shooterLength, CHSV(color, 255, 255));
  fill_solid(climber, climberLength, CHSV(color, 255, 255));
  FastLED.show();

  delay(del);
}

//Subtle shifting gradient between two colors
int g = 1;
int rev = 2;
void MovingGradient(int c1, int c2, int del)
{
  if ((g + c1) > c2)
  {
    rev = 3;
  }
  if (g <= 0)
  {
    rev = 2;
  }

  fill_gradient_RGB(climber, 0, CHSV(g + c1, 255, 255), climberLength, CHSV(c2 - g, 255, 255));
  fill_gradient_RGB(shooter, 0, CHSV(g + c1, 255, 255), shooterLength, CHSV(c2 - g, 255, 255));
  fill_gradient_RGB(intake, 0, CHSV(g + c1, 255, 255), intakeLength, CHSV(c2 - g, 255, 255));

  if (rev == 3)
  {
    g--;
  }
  else
  {
    g++;
  }

  delay(del);
  FastLED.show();
}

//Two-dot chase pattern w/two colors; set both to the same for single color; adjustable speed
void ColorChase(int color1, int color2, int del)
{
  for (int q = 0; q < 5; q++)
  {
    if (Serial.available() > 0)
      break;
    for (uint16_t i = 0; i < intakeLength; i = i + 5)
    {
      if (Serial.available() > 0)
        break;
      intake[i + q] = CHSV(color1, 255, 255);
      intake[i + q + 1] = CHSV(color2, 255, 255);
    }
    for (uint16_t i = 0; i < shooterLength; i = i + 5)
    {
      if (Serial.available() > 0)
        break;
      shooter[i + q] = CHSV(color1, 255, 255);
      shooter[i + q + 1] = CHSV(color2, 255, 255);
    }
    for (uint16_t i = 0; i < climberLength; i = i + 5)
    {
      if (Serial.available() > 0)
        break;
      climber[i + q] = CHSV(color1, 255, 255);
      climber[i + q + 1] = CHSV(color2, 255, 255);
    }

    delay(del);

    for (uint16_t i = 0; i < intakeLength; i = i + 5)
    {
      intake[i + q - 1] = CRGB(0, 0, 0);
      if (Serial.available() > 0)
        break;
    }
    for (uint16_t i = 0; i < shooterLength; i = i + 5)
    {
      shooter[i + q - 1] = CRGB(0, 0, 0);
      if (Serial.available() > 0)
        break;
    }
    for (uint16_t i = 0; i < climberLength; i = i + 5)
    {
      climber[i + q - 1] = CRGB(0, 0, 0);
      if (Serial.available() > 0)
        break;
    }
    FastLED.show();
  }
}
