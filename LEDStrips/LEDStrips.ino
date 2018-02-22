//----------------------------------------------------------------------------
//
//  $Workfile: DisplayCabinet$
//
//  $Revision: X$
//
//  Project:    DisplayCabinet
//
//                            Copyright (c) 2017
//                              James A Wright
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "Common.h"

//----------------------------------------------------------------------------
//  Global Defines
//----------------------------------------------------------------------------
#define PIN1        5
#define PIN2       11

//----------------------------------------------------------------------------
//  Class attributes
//----------------------------------------------------------------------------
Adafruit_NeoPixel_ZeroDMA mStrip1(MAX_LEDS, PIN1, NEO_GRB);
Adafruit_NeoPixel_ZeroDMA mStrip2(MAX_LEDS, PIN2, NEO_GRB);        //  <----  Power 
int mLineCount = 0;                                                // O O O O O O O O    O O O O O O O O 
uint32 mShelfs[MAX_UNITS*MAX_SHELFS][MAX_LEDS];                    // 0 1 2 3 4 5 6 7    0 1 2 3 4 5 6 7
uint16 mShelfSettings[MAX_UNITS*MAX_SHELFS];
int mSensorPin = A1;

//----------------------------------------------------------------------------
//  Purpose:
//      Set up the Ardiuno
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);

  mStrip1.begin();
  mStrip1.setBrightness(200);
  mStrip1.show();
  mStrip2.begin();
  mStrip2.setBrightness(200);
  mStrip2.show();
  
  pinMode( 6, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);

  // Set up  the shelfs for testing
  mShelfSettings[ 0] = ANIM_LAVA;
  mShelfSettings[ 1] = ANIM_OCEAN;
  //mShelfSettings[ 0] = SOLID_WHITE;
  //mShelfSettings[ 1] = SOLID_WHITE;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Idle Loop
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void loop() 
{
  int newAmount = analogRead(mSensorPin);
  Serial.println(newAmount);

  if(newAmount<=700)
  {
    mShelfSettings[ 0] = (newAmount/50);
    mShelfSettings[ 1] = (newAmount/50);
  }

  mLineCount++;
  if(mLineCount>=MAX_LINES)
  {
    mLineCount = 0;
  }
  
  SetColors();
  for(int i=0; i<MAX_LEDS; i++) 
  {
    mStrip1.setPixelColor(i, mShelfs[0][i]);
    mStrip2.setPixelColor(i, mShelfs[1][i]);
  }
  
  // Set the pins after waiting due to the stripts are not done painting yet.
  delay(70);
  mStrip1.show();
  mStrip2.show();
}

//----------------------------------------------------------------------------
//  Purpose:
//      Set the array for displaying LEDs
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void SetColors()
{
  for(int i=0;i<MAX_SHELFS*MAX_UNITS;i++)
  {
    SetLEDs(i,mShelfSettings[i]);
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Set all of the LEDs in a row of the Shelf Array
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void SetLEDs(int index, int color)
{
  for(int i=0;i<MAX_LEDS;i++)
  {
    switch(color)
    {
      case(SOLID_BLUE):
        mShelfs[index][i] = COLOR_BLUE; 
        break;
      case(SOLID_GREEN):
        mShelfs[index][i] = COLOR_GREEN; 
        break;
      case(SOLID_ORANGE):
        mShelfs[index][i] = COLOR_ORANGE; 
        break;
      case(SOLID_PURPLE):
        mShelfs[index][i] = COLOR_PURPLE; 
        break;
      case(SOLID_YELLOW):
        mShelfs[index][i] = COLOR_YELLOW; 
        break;
      case(SOLID_RED):
        mShelfs[index][i] = COLOR_RED; 
        break;
      case(SOLID_WHITE):
        mShelfs[index][i] = COLOR_WHITE; 
        break;
      case(ANIM_FIRE):
        mShelfs[index][i] = gFireData[mLineCount][i]; 
        break;
      case(ANIM_LAVA):
        mShelfs[index][i] = gLavaData[mLineCount][i]; 
        break;
      case(ANIM_STARS):
        mShelfs[index][i] = gStarsData[mLineCount][i]; 
        break;
      case(ANIM_ALIEN):
        mShelfs[index][i] = gAlienData[mLineCount][i]; 
        break;
      case(ANIM_OCEAN):
        mShelfs[index][i] = gOceanData[mLineCount][i]; 
        break;
      case(ANIM_SWAMP):
        mShelfs[index][i] = gSwampData[mLineCount][i]; 
        break;
      default:
        mShelfs[index][i] = COLOR_BLACK; 
        break;
    }
  }
}

