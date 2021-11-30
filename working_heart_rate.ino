/*****************************************************************************
  HeartRate.ino

 Copyright (c) 2016 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <Wire.h>
#include <BH1790GLC.h>
//#include <FlexiTimer2.h>
//#include <SoftPWM.h>
//i could turn timer_flg into an int, count up, and do stuff with that

extern "C" {
#include <hr_bh1790.h>
}

#define SAMPLING_CNT_32HZ   (32)

BH1790GLC bh1790glc;
volatile bool timer_flg;
void timer_isr(void);

static uint8_t    s_cnt_freq = 0;

void setup() {

  uint16_t ret16 = ERROR_NONE;

  timer_flg = false;
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000L);

  s_cnt_freq = 0;

  ret16 = hr_bh1790_Init();
  if (ret16 == ERROR_NONE) {
    Serial.println(F("BPM, wearing"));
    //FlexiTimer2::stop();
    //FlexiTimer2::set(250, 1.0/8000, timer_isr);  // 32Hz timer
    //FlexiTimer2::start();

    ret16 = hr_bh1790_StartMeasure();
    if (ret16 != ERROR_NONE) {
      Serial.println(F("Error: hr_bh1790_StartMeasure function"));
      Serial.print(F("ret16 = "));
      Serial.println(ret16, HEX);      
    }
  } else {
    Serial.println(F("Error: hr_bh1790_Init function"));
    Serial.print(F("ret16 = "));
    Serial.println(ret16, HEX);
  }
}

void loop() {

  uint8_t  bpm     = 0U;
  uint8_t  wearing = 0U;
  uint16_t ret16   = ERROR_NONE;

  //if (timer_flg) {
    ret16 = hr_bh1790_Calc(s_cnt_freq);
    if (ret16 == ERROR_NONE) {
      s_cnt_freq++;
      if (s_cnt_freq >= SAMPLING_CNT_32HZ) {
        s_cnt_freq = 0;
        hr_bh1790_GetData(&bpm, &wearing);
        Serial.print(bpm, DEC);
        Serial.print(F(","));
        Serial.println(wearing, DEC);
      }
    } else {
      Serial.println(F("Error: hr_bh1790_Calc function"));
      Serial.print(F("ret16 = "));
      Serial.println(ret16, HEX);
    }
    timer_flg = false;
  //}
  delay(31.25);
}

void timer_isr(void) {
  timer_flg = true;
}

int8_t bh1790_Write(uint8_t adr, uint8_t *data, uint8_t size)
{
  byte   rc  = 0;
  int8_t ret = 0;
  
  rc = bh1790glc.write(adr, data, size);
  if (rc == 0) {
    ret = BH1790_RC_OK;
  } else {
    ret = BH1790_RC_I2C_ERR;
  }

  return (ret);
}

int8_t bh1790_Read(uint8_t adr, uint8_t *data, uint8_t size)
{
  byte   rc  = 0;
  int8_t ret = 0;

  rc = bh1790glc.read(adr, data, size);
  if (rc == 0) {
    ret = BH1790_RC_OK;
  } else {
    ret = BH1790_RC_I2C_ERR;
  }
  
  return (ret);
}
