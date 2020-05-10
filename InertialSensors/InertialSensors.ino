#include "filter.h"
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonC buttonC;
Zumo32U4ButtonB buttonB;


ComplementaryFilter filter;
bool Ready;
float gyro;
float prediction;
//yaw = 0, pitch = 1, roll = 2
int axis = 0;
bool bFlag = false;

void setup() {
  filter.Init();
}

void loop() {


 Ready = filter.CalcAngle(gyro, prediction, axis);

  if (Ready){
    lcd.clear();
    if (bFlag){
      lcd.print(gyro);
    }
    else {
    lcd.print(filter.Average(prediction));
    }

    lcd.gotoXY(0, 1);
    if (axis == 0){
      lcd.print("Yaw");
    }
    if (axis == 1){
      lcd.print("Pitch");
    }
    if (axis == 2){
      lcd.print("Roll");
    }
  }

if (buttonC.getSingleDebouncedPress()){
  if (axis < 2){
    axis++;
  }
  else if (axis ==2){
    axis = 0;
  }
}

  if (buttonB.getSingleDebouncedPress()){
    bFlag = !bFlag;
  }


}
