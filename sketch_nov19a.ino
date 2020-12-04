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

// Bibliothéque et config bluethoot
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menu config`
#endif


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

//temperature 
float temperature; 
const int SENSOR_TEMPERATURE = A5; 

// Connexion Bluthoot
BluetoothSerial SerialBT; 
String inData; 
char receiveChar; 

// prototype des fonctions 
void ouvrir_fenetre(); 
void fermer_fenetre(); 
void traitement_ouverture_fermeture_fenetre();
void calculer_temerature(); 


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
  action_window=ETAT_CONST ;

  //serial Begin
  Serial.begin(115200);

  // Bluethoot message pour établir une connexion
  SerialBT.begin("Fenêtre de la salle Numéro : 1"); 
  Serial.println("the device started, now you can pair it with your phone"); 
}


void loop() {
  //initialisation action par des boutons

  // initialistation action par blueetooth
  if(Serial.available()){
      SerialBT.write(Serial.read());
  }

  while (SerialBT.available()){
    receiveChar = (char)SerialBT.read(); 
    inData += receiveChar;
      if(receiveChar == '\n'){
        SerialBT.print("Received message :"); 
        SerialBT.println(inData);
        action_window = inData.toInt();
        traitement_ouverture_fermeture_fenetre(); 
        inData = ""; 
      }
    delay(50); 
  }
calculer_temerature();  
}


void calculer_temerature(){
 temperature = analogRead(SENSOR_TEMPERATURE);
 temperature=(temperature*500)/1023;
 Serial.print("Temperature : "); 
 Serial.print(temperature); 
 Serial.println("°C"); 
 delay(1000); 
} 
 
void traitement_ouverture_fermeture_fenetre(){
    //récupérer la position acteulle du moteur
  position_actuel = myservo.read();
  Serial.println("=====> Action sur le moteur"); 

  // action fermeture fenêtre 
  if(action_window == CLOSE){
    fermer_fenetre(); 
  } 
  
  // action ouverture fenêtre
  else if (action_window == OPEN){
    ouvrir_fenetre();
  } 
  
  else {
    // Faire rien
  }  
}
void fermer_fenetre(){
  for (positionner_servo = position_actuel ; positionner_servo >=DEGREE_WINDOW_POSITION_CLOSE ; positionner_servo--) {
      // déplacer le moteur vers la nouvelle position
      myservo.write(positionner_servo);
      // Wait for DELAY_VALUE millisecond(s)
      delay(DELAY_VALUE); 
    }
    Serial.println("appel function fermer fenetre");
    action_window = ETAT_CONST;
}

void ouvrir_fenetre(){
  for (positionner_servo = position_actuel; positionner_servo <= DEGREE_WINDOW_POSITION_OPEN ; positionner_servo++) {
      // déplacer le moteur vers la nouvelle position
      myservo.write(positionner_servo);
      // Wait for DELAY_VALUE millisecond(s)
      delay(DELAY_VALUE); 
    }
     Serial.println("appel function ouvrir fenetre");
    action_window = ETAT_CONST;  
}
