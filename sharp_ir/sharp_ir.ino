  #include "SharpIR.h"
  
  int IRPin = A6;
  float distance = 0;
  SharpIR Sharp(IRPin); 
  
void setup() {
   
}



void loop() {

Sharp.Distance(distance);
  
  Serial.print("Distance :");
  Serial.print('\t');
  Serial.print(distance);
  Serial.println('\n');
}
