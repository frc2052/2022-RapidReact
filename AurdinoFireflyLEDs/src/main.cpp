#include <Arduino.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include <comet.h>
#include <fire.h>

#define NUM_LEDS 60
#define LED_PIN 3

#define TIMES_PER_SECOND(x) EVERY_N_MILLISECONDS(1000/x)
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))   

CRGB g_LEDs[NUM_LEDS] = {0}; //Frame buffer for LEDs
int g_Brightness = 255;           // 0-255 LED brightness scale

//digital in pins
int PIN_ONE = 12;
int PIN_TWO = 11;
int PIN_FOUR = 10;
int PIN_EIGHT = 9;
int PIN_SIXTEEN = 8;

// Color variables
int colorVal;
double redVal;
double greenVal;
double blueVal;

int saturation;
int hue;
int value;

//transition variables
bool areLEDsOn = false;
unsigned long timeVal;
bool isCycleUp = false;
double cycleCount = 0;
int lastCode = 0;
int lastPulseCode = 0;
int brightnessModifier = 0;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(g_LEDs, NUM_LEDS);
  FastLED.setBrightness(255);
//  set_max_power_indicator_LED(LED_BUILTIN);   // FastLED will light LED if power limiting

//  FastLED.setMaxPowerInMilliWatts(5000); //1 amps * 5 volts = 5 watts or 5,000 milliwatts
  FastLED.setMaxPowerInMilliWatts(2500); //.5 amps * 5 volts = 2.5 watts or 2,500 milliwatts

  Serial.begin(9600);
  Serial.println("STARTUP");
}

void setColor(int r, int g, int b)
{
  //keep track of last color in global variable incase we want them next loop
//  redVal = r;
//  greenVal = g;
//  blueVal = b;
  fill_solid(g_LEDs, NUM_LEDS, CRGB(r, g, b));
  FastLED.show(g_Brightness);
}

void runStatusModeInitialActions(int code)
{
  switch (code)
  {
    case 1:
      hue = 0;
      saturation = 1;
      value = 1;
      break;
    case 14:
      redVal = 0.5;
      blueVal = redVal * 0.2;
    default:
      break;
  }
}

void LEDsOnWhite() 
{
  redVal = 1;
  greenVal = 1;
  blueVal = 1;
}

void LEDsOff() 
{
  redVal = 0;
  greenVal = 0;
  blueVal = 0;
}

void blinkColor(int r, int g, int b, int delayMs)
{
  if (millis() - timeVal > delayMs) //time to switch
  {
    areLEDsOn = !areLEDsOn;
    timeVal = millis(); //reset the time of transition
  }

  if (areLEDsOn)
  {
    redVal = r;
    greenVal = g;
    blueVal = b;
  }
  else
  {
    LEDsOff();
  }
}

// void pulseColor(int colorCode, int highR, int highG, int highB, int lowR, int lowG, int lowB, int cycleSteps)
// {
//   //this method will fade in brightnets and return to the max color
//   //this works best if you send in a color, then cut the color values, with the low values being half the high values, or low values being 0 if you want them to all fade out
//   //cycle steps will change how quickly the pulse happens. If you send in red (255,0,0) and off (0,0,0) as colors a cycleSteps of 255 will go through every shade of red.
//   //a step value of 10 will skip roughly every 25 shades of red, a value of 1000 will show every shade of red roughly 4 times before moving to the next one

//   if (lastPulseCode != colorCode)
//   {
//     //the pulse is changing color - start over
//     lastPulseCode = colorCode;
//     isCycleUp = false;
//     cycleCount = 0;
//   }

//   double doubleSteps = cycleSteps; //force to a double so we have factional steps

//   double stepR = (highR - lowR) / doubleSteps; //how much to change the color each step
//   double stepG = (highG - lowG) / doubleSteps;
//   double stepB = (highB - lowB) / doubleSteps;

//   int r = highR;
//   int g = highG;
//   int b = highB;
  
//   if (cycleSteps == cycleCount) //we have reached the maximum cycles for this direction (fade/brighten)
//   {
//     cycleCount = 0; //reset cycleCount
//     isCycleUp = !isCycleUp; //switch directions
//   }

//   if (isCycleUp) //gaining in brightness
//   {
//     r = lowR - (cycleCount * stepR);
//     g = lowG - (cycleCount * stepG);
//     b = lowB - (cycleCount * stepB);    
//   }
//   else  //fading in brightness
//   {
//     r = highR - (cycleCount * stepR);
//     g = highG - (cycleCount * stepG);
//     b = highB - (cycleCount * stepB);
//   }

//   //make sure not less than zero due to rounding and multiplication
//   r = (r < 0) ? 0 : r;
//   g = (g < 0) ? 0 : g;
//   b = (b < 0) ? 0 : b;

//   //make sure not more than 255 due to rounding and multiplication
//   r = (r > 255) ? 255 : r;
//   g = (g > 255) ? 255 : g;
//   b = (b > 255) ? 255 : b;
  
//   setColor(r,g,b);  
// }

void blinkingRedStatusMode()
{
  blinkColor(1, 0, 0, 1000);
}


void setRGBFromHSV() {
    if (hue > 360) {
        hue = 360;
    } else if (hue < 0) {
        hue = 0;
    }

    if (saturation > 1) {
        saturation = 1;
    } else if (saturation < 0) {
        saturation = 0;
    }

    if (value > 1) {
        value = 1;
    } else if (value < 0) {
        value = 0;
    }
  
    double R, G, B;
    double H = hue;
    double S = saturation;
    double V = value;

    if (H < 0) 
    {
      H += 360;
    }
    if (H >= 360) 
    {
      H -= 360;
    }

    if (V <= 0) 
    {
      R = G = B = 0;
    } 
    else if (S <= 0) 
    {
      R = G = B = V;
    } 
    else 
    {
      double hf = H / 60.0;
      int i = (int) floor(hf);
      double f = hf - i;
      double pv = V * (1 - S);
      double qv = V * (1 - S * f);
      double tv = V * (1 - S * (1 - f));
      switch (i) {
        /* Red is dominant color */
        case 0 :
          R = V;
          G = tv;
          B = pv;
          break;
        /* Green is dominant color */
        case 1 :
          R = qv;
          G = V;
          B = pv;
          break;
        case 2 :
          R = pv;
          G = V;
          B = tv;
          break;
        /* Blue is the dominant color */
        case 3 :
          R = pv;
          G = qv;
          B = V;
          break;
        case 4 :
          R = tv;
          G = pv;
          B = V;
          break;
        /* Red is the dominant color */
        case 5 :
          R = V;
          G = pv;
          B = qv;
          break;
        /**
         * Just in case we overshoot on our math by a little, we put
         * these here. Since its a switch it won't slow us down at all
         * to put these here
         */
        case 6 :
          R = V;
          G = tv;
          B = pv;
          break;
        case -1 :
          R = V;
          G = pv;
          B = qv;
          break;
        /* The color is not defined, we should throw an error */
        default :
          /* Just pretend its black/white */
          R = G = B = V;
          break;
      }
    }

    redVal = R;
    blueVal = G;
    greenVal = B;
}

void fireflyStatusMode() {
    if (isCycleUp) {
        cycleCount += 0.4;
    } else {
        cycleCount -= 0.4;
    }

    redVal = 0.0003 * (cycleCount * cycleCount) - 0.2;
    greenVal = redVal * 0.2;

    if (cycleCount >= 60) {
        isCycleUp = false;
    } else if (cycleCount <= 0) {
        isCycleUp = true;
    }

    //Serial.println(redVal);
}

void rainbowStatusMode() 
{
  hue += 1;

  if (hue >= 360) {
    hue = 0;
  }

  setRGBFromHSV();
}

void visionTargetingStatusMode() {
    if (isCycleUp) {
        blueVal += 0.05;
    } else {
        blueVal -= 0.05;
    }

    if (blueVal >= 1) {
        isCycleUp = false;
    } else if (blueVal <= 0.25) {
        isCycleUp = true;
    }

    /*evaluateOnOffInterval(500, 500);
    if (areLEDsOn) {
        greenVal = 0;
        redVal = 1;
        blueVal = 0;
    } else {
        LEDsOff();
    }*/
}

void visionTargetFoundStatusMode() {
    if (isCycleUp) {
        greenVal += 0.1;
    } else {
        greenVal -= 0.1;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0.25) {
        isCycleUp = true;
    }
    /*evaluateOnOffInterval(300, 300);
    if (areLEDsOn) {
        greenVal = 1;
        redVal = 0;
        blueVal = 0;
    } else {
        LEDsOff();
    }*/
}

void autonomousIntakeOnStatusMode() {
    redVal = 1;
    greenVal = 0.2;
}

void autonomousDefaultStatusMode() 
{
  fireflyStatusMode();  
}

void autonomousFinishedStatusMode() {
    if (isCycleUp) {
        greenVal += 0.01;
    } else {
        greenVal -= 0.01;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }

    Serial.println(greenVal);

}

void endGameWarningStatusMode() {
    if (isCycleUp) {
        blueVal += 0.07;
    } else {
        blueVal -= 0.07;
    }
    greenVal = blueVal;
    redVal = blueVal;
    
    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
        cycleCount++;
    }
}

void climbingDefaultStatusMode() {
    if (isCycleUp) {
        blueVal += 0.025;
    } else {
        blueVal -= 0.025;
    }
    redVal = blueVal;
    greenVal = blueVal;

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }
}

void climberExtendingStatusMode() {
    if (isCycleUp) {
        blueVal += 0.05;
    } else {
        blueVal -= 0.05;
    }

    if (blueVal >= 1) {
        isCycleUp = false;
    } else if (blueVal <= 0.25) {
        isCycleUp = true;
    }
}

void climberRetractingStatusMode() {
    if (isCycleUp) {
        redVal += 0.05;
        blueVal = redVal * 0.05;
    } else {
        redVal -= 0.05;
        blueVal = redVal * 0.05;
    }

    if (redVal >= 1) {
        isCycleUp = false;
    } else if (redVal <= 0.25) {
        isCycleUp = true;
    }
}

void climberMaxExtensionStatusMode() {
    if (isCycleUp) {
        redVal += 0.07;
    } else {
        redVal -= 0.07;
    }

    if (redVal >= 1) {
        isCycleUp = false;
    } else if (redVal <= 0.25) {
        isCycleUp = true;
    }
}

void climberMinExtensionStatusMode() {
    if (isCycleUp) {
        redVal += 0.07;
    } else {
        redVal -= 0.07;
    }

    if (redVal >= 1) {
        isCycleUp = false;
    } else if (redVal <= 0.25) {
        isCycleUp = true;
    }
}

void climbingMidBarStatusMode() {
    greenVal = redVal = blueVal;
    if (isCycleUp) {
        blueVal += 0.08;
    } else {
        blueVal -= 0.08;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }
}

void climbingHighBarStatusMode() {
    greenVal = redVal = blueVal;
    if (isCycleUp) {
        blueVal += 0.11;
    } else {
        blueVal -= 0.11;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }
}

void climbingTraversalBarStatusMode() {
    greenVal = redVal = blueVal;
    if (isCycleUp) {
        blueVal += 0.15;
    } else {
        blueVal -= 0.15;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }
}

void climbingLockEngagedStatusMode() {
//    evaluateOnOffInterval(500, 500);
    // if (areLEDsOn) {
    //     redVal = 1;
    // } else {
    //     greenVal = 0;
    // }
  blinkColor(1, 0, 0, 1000);
}

void testStatusMode() {
//    evaluateOnOffInterval(1000, 1000);
    if (areLEDsOn) {
        greenVal = 1;
        redVal = 0.2;
    } else {
        LEDsOff();
    }
}

void climberArmsForwardStatusMode() {
  greenVal = 1; 
}

void climberArmsBackStatusMode() {
  redVal = 1; 
}

void shootingStatusMode() {
  if (isCycleUp) {
      blueVal += 0.1;
  } else {
      blueVal -= 0.1;
  }
  greenVal = blueVal;
  redVal = blueVal;

  if (blueVal >= 0.6) {
      isCycleUp = false;
  } else if (blueVal <= 0.1) {
      isCycleUp = true;
  }
}

void climberSwingingForwardStatusMode() {
  greenVal = 1;
}

void climberSwingingBackwardStatusMode() {
  redVal = 1;
}

void climbingTopOfSwingStatusMode() {
  LEDsOnWhite();
}

void teleopIntakeStatusMode() {
  if (isCycleUp) {
      redVal += 0.05;
  } else {
      redVal -= 0.05;
  }
  blueVal = redVal * 0.8;

  if (redVal >= 1) {
      isCycleUp = false;
  } else if (redVal <= 0.3) {
      isCycleUp = true;
  }
}

void hopperFullStatusMode() {
  if (isCycleUp) {
      redVal += 0.05;
  } else {
      redVal -= 0.05;
  }
  blueVal = redVal * 0.02;

  if (redVal >= 1) {
      isCycleUp = false;
  } else if (redVal <= 0.3) {
      isCycleUp = true;
  }
}

void doLoop()
{
  
  EVERY_N_MILLIS(20) //do not run any faster than every N milliseconds
  {
  
  //   DrawComet(NUM_LEDS);
  //   FastLED.show(g_Brightness);

    int code = 0;
    if (digitalRead(PIN_ONE))
    {
      code = code + 1;
    }
    if (digitalRead(PIN_TWO))
    {
      code = code + 2;
    }
    if (digitalRead(PIN_FOUR))
    {
      code = code + 4;
    }
    if (digitalRead(PIN_EIGHT))
    {
      code = code + 8;
    }
    if (digitalRead(PIN_SIXTEEN))
    {
      code = code + 16;
    }

//    Serial.println(code);

    if (code != lastCode)
    {
      redVal = 0;
      greenVal = 0; 
      blueVal = 0;
      cycleCount = 0;
      runStatusModeInitialActions(code);
      lastCode = code;
    }

    //the color code has a range of 0-31. Each number will represent a color preset with 0 = off
    switch (code) {
      case 1:
        rainbowStatusMode();
        break;
      case 2:
        blinkingRedStatusMode();
        break;
      case 3:
        LEDsOnWhite();
        break;
      case 4:
        fireflyStatusMode();
        break;
      case 5:
        autonomousIntakeOnStatusMode();
        break;
      case 6:
        autonomousDefaultStatusMode();
        break;
      case 7:
        autonomousFinishedStatusMode();
        break;
      case 8:
        fireflyStatusMode(); // teleopDefaultStatusMode();
        break;
      case 9:
        visionTargetingStatusMode();
        break;
      case 10:
        visionTargetFoundStatusMode();
        break;
      case 11:
        endGameWarningStatusMode();
        break;
      case 12:
        climbingDefaultStatusMode();
        break;
      case 13:
        climberExtendingStatusMode();
        break;
      case 14:
        climberRetractingStatusMode();
        break;
      case 15:
        climberMaxExtensionStatusMode();
        break;      
      case 16:
        climberMinExtensionStatusMode();
        break;   
      case 17:
        climbingMidBarStatusMode();
        break;   
      case 18:
        climbingHighBarStatusMode();
        break;   
      case 19:
        climbingTraversalBarStatusMode();
        break;   
      case 20:
        climbingLockEngagedStatusMode();
        break;   
      case 21:
        testStatusMode();
        break;   
      case 22:
        climberArmsBackStatusMode();
        break;
      case 23:
        climberArmsForwardStatusMode();
        break;
      case 24:
        shootingStatusMode();
        break;
      case 25:
        climberSwingingForwardStatusMode();
        break;
      case 26:
        climberSwingingBackwardStatusMode();
        break;
      case 27:
        climbingTopOfSwingStatusMode();
        break;
      case 28:
        teleopIntakeStatusMode();
        break;
      case 29:
        hopperFullStatusMode();
        break;
      case 30:
        rainbowStatusMode();
        break;
      default:
        LEDsOff();
        break;
  //    case 22:
  //      lightShowStatusMode();
  //      break;   
    }

    //make sure not less than zero due to rounding and multiplication
    redVal = (redVal < 0) ? 0 : redVal;
    greenVal = (greenVal < 0) ? 0 : greenVal;
    blueVal = (blueVal < 0) ? 0 : blueVal;

    //make sure not more than 1 due to rounding and multiplication
    redVal = (redVal > 1) ? 1 : redVal;
    greenVal = (greenVal > 1) ? 1 : greenVal;
    blueVal = (blueVal > 1) ? 1 : blueVal;

    setColor(redVal * 255, greenVal * 255, blueVal * 255);
  }
}

void loop()
{
  doLoop();
  //  setColor(255,0,0);  
}
