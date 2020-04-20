#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
const int fwd1 = 33;//14;
const int rvs1 = 15;
const int fwd2 = 14;//32;
const int rvs2 = 32;//33;
const int pwmFreq = 25000;
const int encoder1 = 13;
const int encoder2 = 27;
const int batOut = 21;
const int pushbutton = 26;
const int topSwitch = 4;
int ctr = 0;
//int encoderValue;
//const int pwmResolution  = 8;
int fader1 = A2;
int fader2 = A3;
int fader1Val = 0;
int fader2Val = 0;
int fader1Old = 0;
int fader2Old = 0;
int batVoltage = 35;
long debounceMillis = 0;
int encoderCount  = 0;
int encoderCountOld = 0;
int buttonpress = 0;
int oldPress;
int tsPress = 0;
int oldTsPress;
const int numReadings = 15;

int stValCnt = 0;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;
int readings2[numReadings];      // the readings from the analog input
int readIndex2 = 0;              // the index of the current reading
int total2 = 0;                  // the running total
int average2 = 0;
int bat;

int stvCnt;
// New Variables:

// Variable Definitions
int itr = 0;
bool stat = false;
int tStat;
byte message_i2c;
byte sent_data;
//int forward = 9;
//int reverse = 10;
int setIndexString = 5;
int indexPin = A1;
int indexRead;
bool isMaster;
int i2cAddress;
int addresses[6];
int readVal[6];
byte address;
byte error;
int nDevices;
int ctr2 = 1;// was = 0
int x;
String message;
const int nbSlider = 6;
int val[7] = {500, 500, 500, 500, 500, 500, 500};
int requestedPos[nbSlider];
int desiredPos[nbSlider];
int digit1;
int digit2;
int digit3;
int digit4;
int digit5, digit6, digit7, digit8, digit9, digit10, digit11, digit12, digit13, digit14;
int id;
bool moveSlider[nbSlider] = {false, false, false, false, false, false};
int old_id = 7;
int mode = 0;
int yAxis[2] = { 0, 1 }, xAxis[2] = { 3, 2 }, zAxis[2] = { 5, 4 };
int disc_val;
int encoderValue = 0;
int timeNow;
int hapticsVal1, hapticsVal2;

const float minForce = 256;
const float maxForce = 100;
const int nbData = maxForce;
float data[nbData];
bool mov = true;
bool isFollowing;
const int nbAxe = 3;
int nbStepAxe[nbAxe];
int masterEncoderVals[10];
int MasterSwitches[10];
int switch_stat;
int followDist = 500;

bool serial_stat;
int interrupt_cnt = 0;
int times[2];

// Data sets for use in mapping haptics
//float data0[nbData] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10 , 10 , 10 , 10, 10, 10 , 10, 10, 0, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0}; //{0, 1, 2, 3, 4, 4, 4, 0};
//float data1[nbData] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10 , 10 , 10 , 10, 10, 10 , 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0}; // 10 replaced 4
//float data2[nbData] = {0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 100, 100, 100 , 100 , 100 , 100, 100, 100, 100 , 100, 100, 100, 100, 100, 100, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 200, 200, 200, 200, 200, 200, 200 , 200, 200, 200, 200, 200, 200, 200, 200, 200, 200 , 200, 200, 200, 200, 200, 200, 200, 200, 200 , 200, 200, 200, 200, 200, 200, 200, 200, 0, 0, 0, 0, 400, 400, 400, 400, 400, 400, 400, 400, 400}; //{0, 1, 2, 3, 4, 4, 4, 2, 1, 0};
bool corr_loc[3][2] = {{false, false},
  {false, false},
  {false, false}
};

int mssgRcvCount = 0;
int myAddress;
int val1;
int val2;
int hapticsStat;
int xStep;
int yStep;
int zStep;

int pos1Oldf = 0;
int pos2Oldf = 0;

int val1Old = 0, val2Old = 0;
/*
  ledcWrite(0, 255);
  ledcWrite(4, 255);
  delay (500);
  ledcWrite(0, 0);
  ledcWrite(1, 255);
  delay(100);
  ledcWrite(1, 0);

  ledcAttachPin(fwd1, 0);
  ledcAttachPin(rvs1, 1);
  ledcAttachPin(fwd2, 2);
  ledcAttachPin(rvs2, 3);
*/
void tap(int slider)
{
  if (slider == 1)
  {
    ledcWrite(1, 0);
    ledcWrite(0, 255);
    delay(1);
    ledcWrite(1, 255);
    ledcWrite(0, 0);
    delay(1);
  }
  else if (slider == 2)
  {
    ledcWrite(2, 0);
    ledcWrite(3, 255);
    delay(1);
    ledcWrite(2, 255);
    ledcWrite(3, 0);
    delay(1);
  }
}

// Not neccesarily needed any longer (might just adopt an array-based approach to addressing of axes
void setAddress(String address) {
  myAddress = address.toInt();
}

// Slider to val function- runs every iteration to set the position of the slider to the value provided by Unity
void sliderToVal(int val1, int val2) {

  // Analog read to find current positions of sliders
  int pos1 = analogRead(fader1);
  int pos2 = analogRead(fader2);
  //Serial.println("pos1");
  //Serial.println(pos1);
  //Serial.println("pos2");
  // Serial.println(pos2);
  //int value;

  int mapVal1 = map(val1, 0, 127, 0, 511);///
  int mapVal2 = map(val2, 0, 127, 0, 511);///

  // Adjust PWM of motors to increase smoothness
  int value1 = abs(pos1 - val1) * 0.15 + 240; // was 450
  if (value1 > 255) value1 = 255;
  int value2 = abs(pos2 - val2) * 0.15 + 240;
  if (value2 > 255) value2 = 255;
  // Serial.println("Val 2");
  // Serial.println(value2);
  //  Serial.println("Val 1");
  // Serial.println(value1);

  // Adjust accuracy of the sliders (4025 possible positions- is accurate to 80/4024)
  if (abs(pos1 - val1) > 40) // was 2
  {
    if (pos1 > val1)
    {
      ledcWrite(0, 0);
      ledcWrite(1, value1);
      //digitalWrite(33, LOW);
      ///analogWrite(10, value);//
      //digitalWrite(32, HIGH);
    }
    else
    {
      ledcWrite(1, 0);
      ledcWrite(0, value1);
      //digitalWrite(32, LOW);
      //analogWrite(33, value);///
      //digitalWrite(33, HIGH);
    }
  }
  else
  {
    ledcWrite(1, 0);
    ledcWrite(0, 0);
    //digitalWrite(32, LOW);
    //digitalWrite(33, LOW);
  }

  if (abs(pos2 - val2) > 40) // was 2
  {
    if (pos2 > val2)
    {
      ledcWrite(2, 0);
      ledcWrite(3, value2);
      //digitalWrite(14, LOW);
      ///analogWrite(10, value);//
      //digitalWrite(15, HIGH);
    }
    else
    {
      ledcWrite(3, 0);
      ledcWrite(2, value2);
      //digitalWrite(15, LOW);
      //analogWrite(33, value);///
      //digitalWrite(14, HIGH);
    }
  }
  else
  {
    ledcWrite(3, 0);
    ledcWrite(2, 0);
    //digitalWrite(14, LOW);
    //digitalWrite(15, LOW);
  }
}

int reqPosf1 = 0, reqPosf2 = 0;
int valsArray[2] = {500, 500};
int slider2Stat, slider1Stat, switch1 = 0, switch2 = 0;
int time1 = 0, time2 = 0, pos1, pos2;

// Function allow one slider to follow the other
void followMode(int dist)
{
  // Analog read to find current positions of sliders
  pos1 = analogRead(fader1);
  pos2 = analogRead(fader2);
  //int valsArray[2];
  int flag = 0;
  /*
    if (true)
    {
      dist = 400;
      if ((abs(pos1 - pos1Oldf) >= 85 && slider2Stat != 1) || slider1Stat == 1)
      {
        valsArray[0] = pos1;
        valsArray[1] = pos1 - dist;
        flag = 1;
        slider1Stat = 1;
        if (abs(pos2 - valsArray[1]) < 85) slider1Stat = 0;
        if (ctr2 % 5000 == 0)Serial.println("IN LOOP 1");
      }
      ///abs(pos1-valsArray[0]) < 35 && abs(pos2-valsArray[1]) < 35

      if ((abs(pos2 - pos2Oldf) >= 85 && slider1Stat != 1) || slider2Stat == 1)
      {
        valsArray[0] = pos2 + dist;
        valsArray[1] = pos2;
        flag = 1;
        slider2Stat = 1;
        if (abs(pos1 - valsArray[0]) < 85) slider2Stat = 0;
        if (ctr2 % 5000 == 0)Serial.println("IN LOOP 2");
      }
    }

    if ((abs(pos1 - pos1Oldf) >= 20) && slider2Stat == 0 && switch2 == 0)
    {
    valsArray[0] = pos1;
    valsArray[1] = pos1 - dist;
    flag = 1;
    slider1Stat = 1;
    }

    if ((abs(pos2 - pos2Oldf) >= 20) && slider1Stat == 0 && switch1 == 0)
    {
    valsArray[0] = pos2 + dist;
    valsArray[1] = pos2;
    flag = 1;
    slider2Stat = 1;
    }

    // Preclude the output of the other slider (the slider not manually adjusted) for a certain delay.

    if (slider2Stat = 1 && abs(pos1 - valsArray[0]) < 15)
    {
    slider2Stat = 0;
    if (time1 == 0)
    {
      time1 = millis();
      switch1 = 1;
    }
    else
    {
      if (millis() - time1 >= 5)
      {
        switch1 = 0;
        time1 = 0;
      }
    }
    }
    if (slider1Stat = 1 && abs(pos2 - valsArray[1]) < 15)
    {
    slider1Stat = 0;
    if (time2 == 0)
    {
      time2 = millis();
      switch2 = 1;
    }
    else
    {
      if (millis() - time2 >= 0)// set to 60 if both sliders should set the range -change to 0 for max repsonsivity on 1
      {
        switch2 = 0;
        time2 = 0;
      }
    }
    }
  */
  valsArray[0] = pos1;
  if ((abs(pos1 - pos1Oldf) >= 30) && ctr2 % 50 == 0)
  {
    valsArray[0] = pos1;
    valsArray[1] = pos1 - dist;

  }
  //  else if ((abs(pos2 - pos2Oldf) >= 20) || false)
  //  {
  //    valsArray[0] = pos2 + dist;
  //    valsArray[1] = pos2;
  //    flag = 1;
  //    slider2Stat = 1;
  //  }
  if (ctr2 % 5000 == 0 && false)
  {
    Serial.print("valsArray[0] ");
    Serial.println(valsArray[0]);
    Serial.print("valsArray[1] ");
    Serial.println(valsArray[1]);
    Serial.print("pos1 ");
    Serial.println(pos1);
    Serial.print("pos2 ");
    Serial.println(pos2);
  }
  /*
    if (flag == 0 && false)
    {
    valsArray[0] = pos1;
    valsArray[1] = pos2;
    }
  */

  sliderToVal( valsArray[0],  valsArray[1]);
  if (ctr2 % 400 == 0)
  {
    pos1Oldf = pos1;
    pos2Oldf = pos2;
  }
}

// Sends data over bluetooth back to Unity. Note that it also doesnt send every single time the values change
// Alos includes the joystick functionality
void SerialWriteIfChange()
{
  if (fader2Val % 50 == 0 && fader2Val > 100 && fader2Val < 900 && (fader2Val > 520 || fader2Val < 480) && hapticsStat == 6) {
    tap(2);
  }
  if (fader1Val % 50 == 0 && fader1Val > 100 && fader1Val < 900 && (fader1Val > 520 || fader1Val < 480) && hapticsStat == 6) {
    tap(1);
  }
  if ((ctr2 % 150 == 0))
  {
    int fader1Stat, fader2Stat;
    if ((abs(fader1Val - fader1Old) >= 20 || abs(fader2Val - fader2Old) >= 20 || encoderValue != encoderCountOld || buttonpress != oldPress || tsPress != oldTsPress))
    {
      //int fader1Stat, fader2Stat;
      if (abs(fader1Val - fader1Old) >= 20)
      {
        //if (fader1Val % 20 == 0) {tap(1);tap(1);}
        if (fader1Val < fader1Old)
        {
          fader1Stat = 1;
        }
        else
        {
          fader1Stat = 2;
        }
      }
      else
      {
        fader1Stat = 0;
      }
      /// F2
      if (abs(fader2Val - fader2Old) >= 20)
      {
        // if (fader2Val % 20 == 0) {tap(2);tap(2);}
        if (fader2Val < fader2Old)
        {
          fader2Stat = 1;
        }
        else
        {
          fader2Stat = 2;
        }
      }
      else
      {
        fader2Stat = 0;
      }

      if (hapticsStat != 6)
      {
        //      SerialBT.print(myAddress);
        //      SerialBT.print(",");
        SerialBT.print(fader1Val);
        SerialBT.print(",");
        SerialBT.print(fader2Val);
        SerialBT.print(",");
        // SerialBT.print (encoderCount);
        //SerialBT.print (",");
        SerialBT.print(buttonpress);
        SerialBT.print(",");
        SerialBT.print(encoderValue);
        SerialBT.print(",");
        SerialBT.print(tsPress);
        //SerialBT.print (",");
        //SerialBT.println (bat);
        SerialBT.println();
      }
      else
      {
        SerialBT.print(fader1Stat);
        SerialBT.print(",");
        SerialBT.print(fader2Stat);
        SerialBT.println();
      }
      fader1Old = fader1Val;
      fader2Old = fader2Val;

      encoderCountOld = encoderValue;
      oldPress = buttonpress;

      oldTsPress = tsPress;

      //    sliderToVal(desiredPos[0]);
    }

    if (hapticsStat == 6)
    {
      fader1Stat = 0;
      fader2Stat = 0;
      int sendStat = 0;
      if (fader1Val > 900)
      {
        fader1Stat = 2;
        if (ctr2 % 200 == 0) tap(1);
        sendStat = 1;
      }

      if (fader2Val > 900)
      {
        fader2Stat = 2;
        if (ctr2 % 200 == 0) tap(2);
        sendStat = 1;
      }

      if (fader2Val < 100)
      {
        fader2Stat = 1;
        if (ctr2 % 200 == 0) tap(2);
        sendStat = 1;
      }

      if (fader1Val < 100)
      {
        fader1Stat = 1;
        if (ctr2 % 200 == 0) tap(1);
        sendStat = 1;
      }
      if (sendStat == 1)
      {
        SerialBT.print(fader1Stat);
        SerialBT.print(",");
        SerialBT.print(fader2Stat);
        SerialBT.println();
      }
    }

    /*
        if (fader1Val > 900 && hapticsStat == 6)
        {
          int fader1Stat = 2;
          int fader2Stat;
          if (fader2Stat < 100)
          {
            fader2Stat = 2;
            if (ctr2 % 200 == 0) tap(1);
          }
          SerialBT.print(fader1Stat);
          SerialBT.print(",");
          SerialBT.print(fader2Stat);
          SerialBT.println();
          Serial.println("Above 2000");
          if (ctr2 % 200 == 0) tap(1);
        }
        else if (fader1Val < 100 && hapticsStat == 6)
        {
          int fader1Stat = 1;
          int fader2Stat;
          if (fader2Stat > 900)
          {
            fader2Stat = 1;
            if (ctr2 % 200 == 0) tap(1);
          }
          SerialBT.print(fader1Stat);
          SerialBT.print(",");
          SerialBT.print(fader2Stat);
          SerialBT.println();
          Serial.println("Less than 200");
          if (ctr2 % 200 == 0) tap(1);
        }
    */
  }

}

void ReadAndAverageInputs() {
  total = total - readings[readIndex];
  total2 = total2 - readings2[readIndex];
  readings[readIndex] = analogRead(fader1) / 4;
  readings2[readIndex] = analogRead(fader2) / 4;
  total = total + readings[readIndex];
  total2 = total2 + readings2[readIndex];
  readIndex ++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  fader1Val =  total / numReadings;
  fader2Val = total2 / numReadings;
}


void encoderRead()
{
  unsigned long currentMillis = millis();
  int encRead = digitalRead(encoder1);
  if (currentMillis  - debounceMillis > 20 ) // adds a 20 millisecond dbounce filter for the rotary encoder
  {
    debounceMillis = currentMillis;
    int encRead2 = digitalRead(encoder2);
    if (encRead && encRead2)
    {
      encoderCount --;
      if (encoderCount > 24) encoderCount = 0;
    }
    else if (!encRead && !encRead2)
    {
      encoderCount ++;
      if (encoderCount < 0) encoderCount = 24;
    }
  }
}

void newHapticFunc(int k, int j) // k is the data set the first slider is mapping, and j is the data set the second slider is mapping
{
  int currPosition1 = analogRead(fader1);
  int currPosition2 = analogRead(fader2);
  int intervals, intervalCount, Cnt;

  if (k != 0)
  {
    int motorValk = 250 - (k * 12);
    k = k * 9; // Scaling factor is 9- increase for the whole range of haptics sensations to decrease in magnitude
    //float data[nbData] = {0,5,10,100,100,10,5,0};

    // Define number of intervals needed
    intervals = 4025 / k;
    intervalCount = 0, Cnt = 0;

    // Iterate through past all the points already passed by the slider
    Cnt = currPosition1 / intervals;
    intervalCount = intervals * Cnt;


    // Snap to the nearest position-
    if (currPosition1 > (intervalCount + (intervals / 2) + 10) && !(currPosition1 > 4025 || currPosition1 < 1))
    {
      ledcWrite(0, 0);
      ledcWrite(1, motorValk);
    }
    else if (currPosition1 < (intervalCount + (intervals / 2) - 10) && !(currPosition1 > 4025 || currPosition1 < 1))
    {
      ledcWrite(0, motorValk);
      ledcWrite(1, 0);
    }
    else
    {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  }
  else {
    if (hapticsStat == 0)
    {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  }

  if (j != 0)
  {
    int motorValJ = 250 - (j * 12);
    j = j * 9;
    // Define number of intervals needed
    intervals = 4025 / j;
    intervalCount = 0, Cnt = 0;

    // Iterate through past all the points already passed by the slider
    Cnt = currPosition2 / intervals;
    intervalCount = intervals * Cnt;

    //intervalCount = 0

    // Snap to the nearest position-
    if (currPosition2 > (intervalCount + (intervals / 2) + 10) && !(currPosition2 > 4025 || currPosition2 < 1))
    {
      ledcWrite(2, 0);
      ledcWrite(3, motorValJ);
    }
    else if (currPosition2 < (intervalCount + (intervals / 2) - 10) && !(currPosition2 > 4025 || currPosition2 < 1))
    {
      ledcWrite(2, motorValJ);
      ledcWrite(3, 0);
    }
    else
    {
      ledcWrite(2, 0);
      ledcWrite(3, 0);
    }
  }
  else {
    if (hapticsStat == 0)
    {
      ledcWrite(2, 0);
      ledcWrite(3, 0);
    }
  }
}

void IRAM_ATTR updateEncoder() {
  //SerialBT.print ("Called");

  interrupt_cnt++;
  int timeDiff;
  //delay(3);
  int inputB = digitalRead(encoder2); // Don't need to test both encoder inputs because A is already high, this interrupt was called on the rising edge.
  //delay(3);
  int inputA = digitalRead(encoder1);
  //Serial.println(inputB);
  int state = 0, counter = 0;
  int no = 20;
  int stat[no + 1];
  times[interrupt_cnt % 2] = millis();
  if ((interrupt_cnt % 2) == 0)
  {
    timeDiff = millis() - times[1];
  }
  else
  {
    timeDiff = millis() - times[0];
  }

  if (timeDiff > 30) {
    state = 1;
  }
  else state = 0;

  //Serial.println(digitalRead(encoder1));
  //Serial.println(inputB);
  //Serial.println("-------------------");
  //state = 1;
  if (state == 1)
  {
    Serial.println(encoderValue);
    if ((inputB == LOW && inputA == HIGH) || (inputB == HIGH && inputA == LOW))
    {
      encoderValue++;  // A leads B - B is low
      if (encoderValue > 24)
      {
        encoderValue = 0;
      }
    }
    else
    {
      encoderValue--;          // B leads A - B was already high.
      if (encoderValue <= -1)
      {
        encoderValue = 24;
      }
    }
    state = 0;
  }

  //encoderValue = 70;
  //Serial.println(encoderValue); //  Right hand connector test ok.

}


int discreteAxes(int steps1, int steps2)
{
  //Serial.println("CALLED");
  int currPosition1 = analogRead(fader1);
  int currPosition2 = analogRead(fader2);
  int value;
  //Serial.println(intervalCount2);
  int Cnt2 = currPosition1 / (4025 / steps2);
  int intervalCount2 = (4025 / steps2) * Cnt2;
  int val2 = (intervalCount2 + ((4025 / steps2) / 2));

  if (currPosition1 > (intervalCount2 + ((4025 / steps2) / 2) - 200))
  {
    ledcWrite(0, 170);
    ledcWrite(1, 0);
  }
  else if (currPosition1 < (intervalCount2 + ((4025 / steps2) / 2) + 200))
  {
    ledcWrite(1, 170);
    ledcWrite(0, 0);
  }
  else
  {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }


  Cnt2 = currPosition2 / (4025 / steps2);
  intervalCount2 = (4025 / steps2) * Cnt2;
  val2 = (intervalCount2 + ((4025 / steps2) / 2));

  if (currPosition2 > (intervalCount2 + ((4025 / steps2) / 2) - 200))
  {
    ledcWrite(2, 170);
    ledcWrite(3, 0);
  }
  else if (currPosition2 < (intervalCount2 + ((4025 / steps2) / 2) + 200))
  {
    ledcWrite(3, 170);
    ledcWrite(2, 0);
  }
  else
  {
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  }
  return 1;
}


int asciiReader(char symbol)
{
  int number;
  int asciiNo = symbol;
  number = asciiNo - 48;
  return number;
}


void setup() {
  Serial.begin(2000000);
  SerialBT.begin("FaderAxis_NEW"); //Change this for each unit
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  pinMode(pushbutton, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  //pinMode(13, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1), updateEncoder, RISING);////////////////////////////////
  //attachInterrupt(digitalPinToInterrupt(encoder2), updateEncoder, FALLING);
  //analogReadResolution(4); // can set this all the way to 12bits of resolution
  ////pwmsetup

  for (int i = 0; i < 4; i++)
  {
    ledcSetup(i, pwmFreq, 8);
  }
  ledcSetup (4, pwmFreq, 8); //led outpin
  ledcAttachPin(fwd1, 0);
  ledcAttachPin(rvs1, 1);
  ledcAttachPin(fwd2, 2);
  ledcAttachPin(rvs2, 3);
  ledcAttachPin(batOut, 4);
  // Make the sldiers move up and down on boot up
  ledcWrite(0, 255);
  ledcWrite(4, 255);
  delay (500);
  ledcWrite(0, 0);
  ledcWrite(1, 255);
  delay(100);
  ledcWrite(1, 0);

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
    readings2[i] = 0;
  }
}

void loop() {
  //SerialBT.println("Third AXIS HERE");
  //sliderToVal(502, 10);
  //Serial.println(encoderValue);
  //Serial.println(analogRead(fader2));
  //Serial.println(analogRead(fader1));
  /*
    Serial.println(digitalRead(encoder1));
    Serial.println(digitalRead(encoder2));
    Serial.println("-------------------");
  */
  bat = analogRead(batVoltage);
  //buttonpress = digitalRead(pushbutton);

  // Rotary Encoder Switch
  if (digitalRead(pushbutton)) {
    buttonpress = 0;
  }
  else {
    buttonpress = 1;
  }

  if (digitalRead(topSwitch)) {
    tsPress = 0;
  }
  else {
    tsPress = 1;
  }

  ledcWrite(4, map(bat, 230, 273, 10, 255 ) );  // pwm output for battery level led
  ReadAndAverageInputs();

  SerialWriteIfChange();
  //ctr2++;

  //delay(3);
  //Serial.println("In UPDATE Loop");


  // If there is a serial bluetooth connection, read the message form Unity and act accordingly
  if (SerialBT.available() > 0) {
    //digitalWrite(13, HIGH);
    if (mssgRcvCount == 0)
    {
      message = Serial.readStringUntil('\n');
      myAddress = message[0] - '0';
      mssgRcvCount++;
    }
    else
    {
      //Serial.println("Serial Availible");
      //digitalWrite(13, HIGH);
      message = SerialBT.readStringUntil('\n');
      old_id = id;

      mode = 0;
      //      digit1 = message[0] - '0';
      //      digit2 = message[1] - '0';
      //      digit3 = message[2] - '0';
      //      digit4 = message[3] - '0';
      //      digit5 = message[4] - '0';
      //      digit6 = message[5] - '0';
      //      digit7 = message[6] - '0';
      //      digit8 = message[7] - '0';
      //      digit9 = message[8] - '0';
      //      digit10 = message[9] - '0';
      //      digit11 = message[10] - '0';
      //      digit12 = message[11] - '0';
      digit1 = asciiReader(message[0]);
      digit2 = asciiReader(message[1]);
      digit3 = asciiReader(message[2]);
      digit4 = asciiReader(message[3]);
      digit5 = asciiReader(message[4]);
      digit6 = asciiReader(message[5]);
      digit7 = asciiReader(message[6]);
      digit8 = asciiReader(message[7]);
      digit9 = asciiReader(message[8]);
      digit10 = asciiReader(message[9]);
      digit11 = asciiReader(message[10]);
      digit12 = asciiReader(message[11]);
      digit13 = asciiReader(message[12]); ///
      digit14 = asciiReader(message[13]);
      //digit12 = asciiReader(message[12]);
      // Serial.println(digit1);
      // Serial.println(digit2);
      // Serial.println(digit3);
      //  Serial.println(digit4);
      //Serial.println(digit5);
      // Serial.println(digit6);
      //      Serial.println(digit7);
      //      Serial.println(digit8);
      //      Serial.println(digit9);
      //      Serial.println(digit10);
      //      Serial.println(digit11);
      //      //Serial.println(digit12);
      //Serial.println("-----------");
      val1 = digit1 * 1000 + digit2 * 100 + digit3 * 10 + digit4;
      val2 = digit5 * 1000 + digit6 * 100 + digit7 * 10 + digit8;
      // Serial.println(val1);
      //  Serial.println(val2);
      // Serial.println(analogRead(fader1));
      //Serial.println(analogRead(fader2));
      //Serial.println(digit9);
      //Serial.println(digit2);
      //Serial.println(message[9].toInt());
      hapticsStat = digit9;

      //Serial.println("HAPTICS STAT");
      //  Serial.println(hapticsStat);
      xStep = digit10;
      yStep = digit11;
      zStep = digit12;

      hapticsVal1 = digit13;
      hapticsVal2 = digit14;
    }
  }
  if (hapticsStat != 6)
  {
    newHapticFunc(hapticsVal1, hapticsVal2);
  }
  //hapticsStat = 2;/////////////////////////////////////////////////////////////////////////////////////////////
  //xStep = 3;
  //yStep = 3;
  //val1 = 400;

  // This runs every loop, calls the relevant functions for the mode the slider has been set in
  if (hapticsStat == 0)
  {
    //val1 = 100; val2 = 100;
    if (val1 != val1Old || val2 != val2Old)
    {
      while (abs(analogRead(fader1) - val1) >= 40 || abs(analogRead(fader2) - val2) >= 40)
      {
        sliderToVal(val1, val2);
      }
      val1Old = val1;
      val2Old = val2;
      stValCnt = 0;
    }
  }
  else if (hapticsStat == 1)
  {
    newHapticFunc(hapticsVal1, hapticsVal2);
    if (val1 != val1Old || val2 != val2Old)
    {
      while (abs(analogRead(fader1) - val1) >= 40 || abs(analogRead(fader2) - val2) >= 40)
      {
        sliderToVal(val1, val2);
      }
      val1Old = val1;
      val2Old = val2;
      stValCnt = 0;
    }
  }
  else if (hapticsStat == 2)
  {
    int a = discreteAxes(xStep, yStep); // must change from unity side
    //Serial.println(a);
    if (val1 != val1Old || val2 != val2Old)
    {
      while (abs(analogRead(fader1) - val1) >= 30 || abs(analogRead(fader2) - val2) >= 40)
      {
        sliderToVal(val1, val2);
      }
      val1Old = val1;
      val2Old = val2;
      stValCnt = 0;
    }
  }
  else if (hapticsStat == 5)
  {
    //sliderToVal(1600, 1600);
    if (val2 != val2Old)
    {
      while (abs(analogRead(fader1) - val2) >= 40)
      {
        sliderToVal(val2, val2-val1);
      }
      val2Old = val2;
      stValCnt = 0;
    }

    followDist = val1;
    // new follow function
    followMode(followDist);
  }
  else if (hapticsStat == 6)
  {
    sliderToVal(1600, 1600); // In joystick mode, snap the sliders to the middle of the axes and move them from there
  }
  ctr2++;
}
