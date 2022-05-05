//+--------------------------------------------------------------------------
//
// NightDriver - (c) 2020 Dave Plummer.  All Rights Reserved.
//
// File:                  
//
// Description:
//
//   
//
// History:     Sep-28-2020     davepl      Created
//
//---------------------------------------------------------------------------

#include <Arduino.h>
#define FASTLED_INTERNAL
#include <FastLED.h>

extern CRGB g_LEDs[];

void DrawComet(int numLeds)
{
    const byte fadeAmt = 128;
    const int cometSize = 3;
    const int deltaHue  = 4;

    static byte hue = HUE_RED;
    static int iDirection = 1;
    static int iPos = 0;

    hue += deltaHue;

    iPos += iDirection;
    if (iPos == (numLeds - cometSize) || iPos == 0)
        iDirection *= -1;
    
    for (int i = 0; i < cometSize; i++)
        g_LEDs[iPos + i].setHue(hue);
    
    // Randomly fade the LEDs
    for (int j = 0; j < numLeds; j++)
        if (random(10) > 5)
            g_LEDs[j] = g_LEDs[j].fadeToBlackBy(fadeAmt);  

    delay(25);
}