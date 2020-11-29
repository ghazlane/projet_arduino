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
#define DEGREE_WINDOW_POSITION_CLOSE 0
#define DEGREE_WINDOW_POSITION_OPEN 90
#define DELAY_VALUE 15

Servo myservo;  // create servo object to control a servo
int positionner_servo;    // variable to store the servo position
int action_window;    // Action à appliquer sur la feneêtre
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
  action_window=OPEN ;

  //serial Begin
  Serial.begin(115200);
}

void loop() {
  //initialisation action par des boutons

  // initialistation action par blueetooth

  /******************* delete this block ************/
  delay(10000);
  Serial.println("retour position"); 
  myservo.write(25);
  delay(10000);
  /*************************************************/
  Serial.print("Commencer execution  de la bouce loop !");

  //récupérer la position acteulle du moteur
  position_actuel = myservo.read();

  // action fermeture fenêtre 
  if(action_window == CLOSE){
    for (positionner_servo = position_actuel ; positionner_servo >=DEGREE_WINDOW_POSITION_CLOSE ; positionner_servo--) {
      // déplacer le moteur vers la nouvelle position
      myservo.write(positionner_servo);
      // Wait for DELAY_VALUE millisecond(s)
      delay(DELAY_VALUE); 
    }
    action_window = ETAT_CONST;
  } 
  
  // action ouverture fenêtre
  else if (action_window == OPEN){
    for (positionner_servo = position_actuel; positionner_servo <= DEGREE_WINDOW_POSITION_OPEN ; positionner_servo++) {
      // déplacer le moteur vers la nouvelle position
      myservo.write(positionner_servo);
      // Wait for DELAY_VALUE millisecond(s)
      delay(DELAY_VALUE); 
    }
    action_window = ETAT_CONST;
  } 
  
  else {
    // Faire rien
  }
}
