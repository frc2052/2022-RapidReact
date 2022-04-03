// Arduino analog out pins
int RED_PIN = 3;
int GREEN_PIN = 6;
int BLUE_PIN = 10;

//digital in pins
int PIN_ONE = 14;
int PIN_TWO = 15;
int PIN_FOUR = 16;
int PIN_EIGHT = 17;
int PIN_SIXTEEN = 18;

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
int cycleCount = 0;
int lastCode = 0;
int lastPulseCode = 0;
int brightnessModifier = 0;

void setup(){ 
  Serial.begin(9600);
  Serial.println("STARTUP");

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
} 

void loop(){

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

  if (code != lastCode)
  {
    redVal, greenVal, blueVal = 0;
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
//    case 22:
//      lightShowStatusMode();
//      break;   
  }

  //make sure not less than zero due to rounding and multiplication
  redVal = (redVal < 0) ? 0 : redVal;
  greenVal = (greenVal < 0) ? 0 : greenVal;
  blueVal = (blueVal < 0) ? 0 : blueVal;

  //make sure not more than 255 due to rounding and multiplication
  redVal = (redVal > 255) ? 255 : redVal;
  greenVal = (greenVal > 255) ? 255 : greenVal;
  blueVal = (blueVal > 255) ? 255 : blueVal;

  setColor(redVal, greenVal, blueVal);
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

void setColor(int r, int g, int b)
{
  //keep track of last color in global variable incase we want them next loop
  redVal = r;
  greenVal = g;
  blueVal = b;
  
  analogWrite(RED_PIN, redVal);
  analogWrite(GREEN_PIN, greenVal); 
  analogWrite(BLUE_PIN, blueVal);    
  Serial.println(String(redVal) + " RED " + String(greenVal) + " GREEN " + String(blueVal) + " BLUE");
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

void pulseColor(int colorCode, int highR, int highG, int highB, int lowR, int lowG, int lowB, int cycleSteps)
{
  //this method will fade in brightnets and return to the max color
  //this works best if you send in a color, then cut the color values, with the low values being half the high values, or low values being 0 if you want them to all fade out
  //cycle steps will change how quickly the pulse happens. If you send in red (255,0,0) and off (0,0,0) as colors a cycleSteps of 255 will go through every shade of red.
  //a step value of 10 will skip roughly every 25 shades of red, a value of 1000 will show every shade of red roughly 4 times before moving to the next one

  if (lastPulseCode != colorCode)
  {
    //the pulse is changing color - start over
    lastPulseCode = colorCode;
    isCycleUp = false;
    cycleCount = 0;
  }

  double doubleSteps = cycleSteps; //force to a double so we have factional steps

  double stepR = (highR - lowR) / doubleSteps; //how much to change the color each step
  double stepG = (highG - lowG) / doubleSteps;
  double stepB = (highB - lowB) / doubleSteps;

  int r = highR;
  int g = highG;
  int b = highB;
  
  if (cycleSteps == cycleCount) //we have reached the maximum cycles for this direction (fade/brighten)
  {
    cycleCount = 0; //reset cycleCount
    isCycleUp = !isCycleUp; //switch directions
  }

  if (isCycleUp) //gaining in brightness
  {
    r = lowR - (cycleCount * stepR);
    g = lowG - (cycleCount * stepG);
    b = lowB - (cycleCount * stepB);    
  }
  else  //fading in brightness
  {
    r = highR - (cycleCount * stepR);
    g = highG - (cycleCount * stepG);
    b = highB - (cycleCount * stepB);
  }

  //make sure not less than zero due to rounding and multiplication
  r = (r < 0) ? 0 : r;
  g = (g < 0) ? 0 : g;
  b = (b < 0) ? 0 : b;

  //make sure not more than 255 due to rounding and multiplication
  r = (r > 255) ? 255 : r;
  g = (g > 255) ? 255 : g;
  b = (b > 255) ? 255 : b;
  
  setColor(r,g,b);  
}

void LEDsOnWhite() 
{
  redVal = 255;
  greenVal = 255;
  blueVal = 255;
}

void LEDsOff() 
{
  redVal = 0;
  greenVal = 0;
  blueVal = 0;
}

void blinkingRedStatusMode()
{
  blinkColor(255, 0, 0, 1000);
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
        blueVal += 0.1;
    } else {
        blueVal -= 0.1;
    }

    if (blueVal >= 1) {
        isCycleUp = false;
    } else if (blueVal <= 0.5) {
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
        greenVal += 0.2;
    } else {
        greenVal -= 0.2;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0.5) {
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

void autonomousDefaultStatusMode() {}

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
}

void endGameWarningStatusMode() {
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
        cycleCount++;
    }
}

void climbingDefaultStatusMode() {
    greenVal = redVal = blueVal;
    if (isCycleUp) {
        blueVal += 0.01;
    } else {
        blueVal -= 0.01;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0) {
        isCycleUp = true;
    }
}

void climberExtendingStatusMode() {
    if (isCycleUp) {
        blueVal += 0.1;
    } else {
        blueVal -= 0.1;
    }

    if (blueVal >= 1) {
        isCycleUp = false;
    } else if (blueVal <= 0.5) {
        isCycleUp = true;
    }
}

void climberRetractingStatusMode() {
    if (isCycleUp) {
        redVal += 0.1;
        blueVal = redVal * 0.2;
    } else {
        blueVal -= 0.1;
        blueVal = redVal * 0.2;
    }

    if (blueVal >= 1) {
        isCycleUp = false;
    } else if (blueVal <= 0.5) {
        isCycleUp = true;
    }
}

void climberMaxExtensionStatusMode() {
    if (isCycleUp) {
        greenVal += 0.1;
    } else {
        greenVal -= 0.1;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0.5) {
        isCycleUp = true;
    }
}

void climberMinExtensionStatusMode() {
    if (isCycleUp) {
        redVal += 0.1;
    } else {
        redVal -= 0.1;
    }

    if (greenVal >= 1) {
        isCycleUp = false;
    } else if (greenVal <= 0.5) {
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
}

void climbingLockEngagedStatusMode() {
//    evaluateOnOffInterval(500, 500);
    if (areLEDsOn) {
        redVal = 1;
    } else {
        greenVal = 0;
    }
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
