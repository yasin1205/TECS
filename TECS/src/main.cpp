#include <Arduino.h>
#include <TECS.h>

TECS tecs;

void setup() {

tecs.begin();// default değerlerle başlatıyorum

Serial.begin(9600);

}

void loop() {

//   void update(float h, float h_dot, float v, float v_dot, float h_sp, float h_dot_sp, float v_sp, float v_dot_sp, float dt);
  if(millis() < 10000){
  tecs.update(100, 0, 20, 0.5, 100, 0, 25, 2, 1);
}
  if(millis() > 10000){
  tecs.update(100, 0, 20, 0.5, 100, 5, 20, 2, 1);
}
  delay(100);
  Serial.print("millis:");
  Serial.print(millis());
  Serial.print(" pitch:");
  Serial.print(tecs.pitchOutput);
  Serial.print(" throttle:");  
  Serial.println(tecs.thrOutput);
}


