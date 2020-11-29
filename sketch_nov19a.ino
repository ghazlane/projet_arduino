/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett

 see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
 * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */
#include <ESP32Servo.h>
#include <stdio.h>
#include <stdlib.h>

#define SERVO_MOTEUR_PIN 4
#define OPEN 1 
#define CLOSE 2
#define ETAT_CONST 0

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int i=0;
int action_window;
int position_actuel;


void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // standard 50 hz servo
  myservo.setPeriodHertz(50); 
     
  // attaches the servo on pin 18 to the servo object
  myservo.attach(SERVO_MOTEUR_PIN, 500, 2400); 

  //action d'intitialisation
  action_window=CLOSE ;

  //serial Begin
  Serial.begin(115200);
}

void loop() {
  //initialisation action par des boutons ;;;
  
  

//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);    // tell servo to go to position in variable 'pos'
//    delay(15);             // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);    // tell servo to go to position in variable 'pos'
//    delay(15);             // waits 15ms for the servo to reach the position
//  }


  Serial.print("Bonjour mohamed !");
 // test = analogRead(10); 
   
  if(action_window == CLOSE){
    position_actuel = myservo.read();
    for (pos = 0; pos <= 90; pos++) {
      myservo.write(pos);
      delay(15); // Wait for 15 millisecond(s)
    }
    Serial.println("action == 2 ==> !");
    action_window = ETAT_CONST;
  } else if (action_window == OPEN){
    position_actuel = myservo.read();
    for (pos = 90; pos >= 0; pos--) {
      myservo.write(pos);
      delay(15); // Wait for 15 millisecond(s)
    }
    Serial.println("action == 1 ==>  !");
    action_window = ETAT_CONST;
  } else {
    Serial.println("action == 0 ==>  !");
  }
}
