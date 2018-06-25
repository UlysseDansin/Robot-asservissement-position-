/**********************************************************************************************/
/* Mesures des positions angulaires des roues d'un robot mobile à deux roues (type unicycle)  */
/* Conduite différnetielle - orientation du robot                                             */
/* Programme : Prog2_Mes_pos_robot                                                            */
/**********************************************************************************************/

// Déclarations entrée/sortie

#include <Servo.h>
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
Servo myservo;
#define codeurDroitInterruptionA 3  // interruption 3 = broche n°20 - Arduino MEGA2560
#define codeurDroitInterruptionB 2  // interruption 2 = broche n°21 - Arduino MEGA2560
#define codeurDroitPinA 20          // Signal A du codeur magnétique Moteur Droit
#define codeurDroitPinB 21          // Signal B du codeur magnétique Moteur Droit
volatile long ticksCodeurDroit = 0; //Compteur d'impulsions "ticks" codeur Moteur Droit
static int codeurDroitDeltaPos;

// Définitions et déclarations pour le codeur incrémental du moteur Gauche (MB)
#define codeurGaucheInterruptionA 5   // interruption 5 = broche n°18 - Arduino MEGA2560
#define codeurGaucheInterruptionB 4   // interruption 4 = broche n°19 - Arduino MEGA2560
#define codeurGauchePinA 18           // Signal A du codeur magnétique Moteur Gauche
#define codeurGauchePinB 19           // Signal A du codeur magnétique Moteur Gauche
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
volatile long ticksCodeurGauche = 0;  // Compteur d'impulsions "ticks" codeur Moteur Gauche
static int indiceTicksCodeurGauche = 0;
static int codeurGaucheDeltaPos;

const float rapport_reducteur = 43.7;  // Rapport de réduction du réducteur accouplé au moteur
const int res_codeur_voieA = 32;       // nombre d'impulsion sur la voie A pour 1 tour du codeur magnétique (16 pôles)
const int res_codeur_voieAB = 64;  
#define TSDATA 1000
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// On déclare ci-dessous les variables et paramètres nécessaires au traitement de l'information
static double R = 4.5; // Rayon d'une roue en cm
static double L = 22.3; // Largeur du robot en cm
static double pi = 3.14159 ; // valeur de pi

// Grandeur Mesurée
float AngleRoueDroite = 360 ;      // angle de rotation effectué par chaque roue Droite en °
float AngleRoueGauche = 360 ;      // angle de rotation effectué par chaque roue Gauche en °
float Dep_lineaire_Droite = 0.30 ;  // Le déplacement linéaire effectué par la rotation la roue Droite en cm.
float Dep_lineaire_Gauche = 0.30 ;  // Le déplacement linéaire effectué par la rotation la roue Gauche en cm.
float orientation_robot = 0.;
int potpin = 0;  // analog pin used to connect the potentiometer
int val;  
int nbtickspartour= 2797;

float distance1tour=28.27431;

float distance_1cm=nbtickspartour/distance1tour;

float distance_objectif ;

float nbtick1deg = nbtickspartour/360;

float angle_objectif;

int cercle= 6900; //2797*((pi*(L/2))/(pi*R));
Servo myservo1;  // create servo object to control a servo

int potpin1 = 0;  // analog pin used to connect the potentiometer
int val1;    // variable to read the value from the analog pin
 
int deg=cercle/360;
int moteurA=41;
int moteurA2=43;
int moteurB=35;
int moteurB2=37;

// Initialisations

void setup() {
  // Codeur incrémental moteur droit
  pinMode(codeurDroitPinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurDroitPinA, HIGH);  // activation de la résistance de pullup interne de l'arduino
  pinMode(codeurDroitPinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurDroitPinB, HIGH);  // activation de la résistance de pullup interne de l'arduino
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  attachInterrupt(codeurDroitInterruptionA, GestionInterruptionCodeurDroitPinA, CHANGE);
  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  attachInterrupt(codeurDroitInterruptionB, GestionInterruptionCodeurDroitPinB, CHANGE);

  // Codeur incrémental moteur gauche
  pinMode(codeurGauchePinA, INPUT);      // entrée digitale pin A codeur
  digitalWrite(codeurGauchePinA, HIGH);  // activation de la résistance de pullup
  pinMode(codeurGauchePinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurGauchePinB, HIGH);  // activation de la résistance de pullup
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  attachInterrupt(codeurGaucheInterruptionA, GestionInterruptionCodeurGauchePinA, CHANGE);

  // A chaque changement de niveau de tension sur le pin B du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinB (définie à la fin du programme)
  attachInterrupt(codeurGaucheInterruptionB, GestionInterruptionCodeurGauchePinB, CHANGE);
myservo.attach(40);
myservo1.attach(46);  
  pinMode(moteurA,OUTPUT);
  pinMode(moteurB,OUTPUT);
  pinMode(moteurA2,OUTPUT);
  pinMode(moteurB2,OUTPUT);
  Serial.begin(9600);

  // Compteur d'impulsions des encodeurs
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
  digitalWrite(33, LOW); 
  digitalWrite(31, LOW); 
  
}


// Boucle principale
void loop() { 

//monter_cannette();
//attraper_cannette_bas();
////deplacer_cannette(220);
//pousser_cannette(255);
//rentrer_canard(255);

      // Initialisation //
      
attraper_cannette_haut();
pas_monter_cannette();

      // Départ //
//      
delay(2000); 
attraper_cannette_haut();
pas_monter_cannette();
turn_droite_une_roue(13);
attraper_cannette_haut();
go(112);
delay(1000);
                              //// 1ère cannette ////
                      
attraper_cannette_bas();
pas_monter_cannette();
delay(2000);
attraper_cannette_haut();
monter_cannette();
delay(3800);
pas_monter_cannette();
deplacer_cannette(255);
turn_droite(44);
go(26);
delay(500);
                              //// 2ème Cannette////
attraper_cannette_bas();
pas_monter_cannette();
delay(2000);
attraper_cannette_haut();
monter_cannette();
delay(3800);
pas_monter_cannette();
deplacer_cannette(255);
delay(500);
turn_gauche_une_roue(38);
delay(1000);
reculer(47);
delay(500);
go(16);
delay(1000);
turn_droite(130);
delay(500);
go(40);
                              //// 3ème cannette /////
pas_monter_cannette();
attraper_cannette_bas();
delay(2000);
attraper_cannette_haut();
delay(500);
monter_cannette();
delay(3800);
pas_monter_cannette();
delay(1000);
turn_droite(79);
delay(500);
go(21);
delay(500);
                                  //// 4ème cannette ////
pas_monter_cannette();
attraper_cannette_bas();
delay(2000);
attraper_cannette_haut();
monter_moitier_cannette();
delay(800);
pas_monter_cannette();
delay(1000);
turn_gauche(155); 
delay(500);
reculer(20);
delay(500);
turn_droite(66);
delay(500);
go(36);
go(12);
delay(500);
pousser_canard(255);
delay(1000);
reculer(15);
turn_gauche_une_roue(26);
reculer(79);
delay(500);
pousser_cannette(255);
pousser_cannette(0);
delay(1000);
deplacer_cannette(255);
deplacer_cannette(255);
delay(1000);
monter_cannette();
delay(3000);
pas_monter_cannette();
delay(1000);
deplacer_cannette(255);
deplacer_cannette(255);
delay(1000);
pousser_cannette(255);
pousser_cannette(0);
attraper_cannette_haut();


delay(10000000);

}

void go (float distance_objectif){

  float nombre_de_tic_a_realiser = distance_1cm * distance_objectif;
  while (ticksCodeurDroit < 0.5*nombre_de_tic_a_realiser && ticksCodeurGauche < 0.5*nombre_de_tic_a_realiser) {
    
   Avancer(255);
   Serial.println(ticksCodeurDroit);
   
    while (ticksCodeurDroit + ticksCodeurGauche > 5) {
      motor(1, BRAKE, 0);
    }
    while (ticksCodeurGauche + ticksCodeurDroit < -5 ){
      motor(2, BRAKE, 0);
    }
  }
  motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);

  while (ticksCodeurDroit < nombre_de_tic_a_realiser && ticksCodeurGauche < nombre_de_tic_a_realiser) {
    
   Avancer(255);
   Serial.println(ticksCodeurDroit);
   
    while (ticksCodeurDroit + ticksCodeurGauche > 5) {
      motor(1, BRAKE, 0);
    }
    while (ticksCodeurGauche + ticksCodeurDroit < -5 ){
      motor(2, BRAKE, 0);
    }
  }
 motor(1, BRAKE, 0);
 motor(2, BRAKE, 0);
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
}

void reculer (float distance_objectif){

  float nombre_de_tic_a_realiser = distance_1cm * distance_objectif;
  while (-ticksCodeurDroit < 0.5*nombre_de_tic_a_realiser && -ticksCodeurGauche < 0.5*nombre_de_tic_a_realiser) {
    
   Reculer(255);
   Serial.println(ticksCodeurDroit);
   
    while (-ticksCodeurDroit - ticksCodeurGauche > 5) {
      motor(1, BRAKE, 0);
    }
    while (-ticksCodeurGauche - ticksCodeurDroit < -5 ){
      motor(2, BRAKE, 0);
    }
  }
  motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);

  while (-ticksCodeurDroit < nombre_de_tic_a_realiser && -ticksCodeurGauche < nombre_de_tic_a_realiser) {
    
   Reculer(255);
   Serial.println(ticksCodeurDroit);
   
    while (-ticksCodeurDroit - ticksCodeurGauche > 5) {
      motor(1, BRAKE, 0);
    }
    while (-ticksCodeurGauche - ticksCodeurDroit < -5 ){
      motor(2, BRAKE, 0);
    }
  }
 motor(1, BRAKE, 0);
 motor(2, BRAKE, 0);
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
}


void turn_droite(float angle_en_deg){

int angle_desire_en_tic= angle_en_deg*deg;

Serial.println(angle_desire_en_tic);
while (-ticksCodeurDroit < 0.6*angle_desire_en_tic  && ticksCodeurGauche < 0.6*angle_desire_en_tic){
    Serial.println(ticksCodeurDroit);
  Droite(255);
    while (20+ticksCodeurDroit <  ticksCodeurGauche) {
    motor(1, BRAKE, 0);
    }
    while (20+ ticksCodeurGauche < ticksCodeurDroit ) {
    motor(2, BRAKE, 0);
    }


  }
  while (-ticksCodeurDroit < angle_desire_en_tic  && ticksCodeurGauche < angle_desire_en_tic){
    Serial.println(ticksCodeurDroit);
  Droite(255);
    while (15+ticksCodeurDroit <  ticksCodeurGauche) {
    motor(1, BRAKE, 0);
    }
    while (15+ ticksCodeurGauche < ticksCodeurDroit ) {
    motor(2, BRAKE, 0);
    }


  }

  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
   motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);
  
}
void turn_gauche_une_roue(float angle_en_deg){

int angle_desire_en_tic= angle_en_deg*deg;


while (ticksCodeurGauche < 0.6*angle_desire_en_tic*2){
    Serial.println(ticksCodeurDroit);
  motor(2, BACKWARD, 255);
  motor(1,BRAKE,0);
  }
  while (ticksCodeurGauche < angle_desire_en_tic*2){
  motor(2, BACKWARD, 255);
  motor(1,BRAKE,0);
  }
// motor(1, BRAKE, 0);
// motor(2, BRAKE, 0);

//  ticksCodeurDroit = 0;
//  ticksCodeurGauche = 0;
 motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);
    ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
}
void turn_droite_une_roue(float angle_en_deg){

int angle_desire_en_tic= angle_en_deg*deg;


while (-ticksCodeurGauche < 0.6*angle_desire_en_tic*2){
    Serial.println(ticksCodeurGauche);
  motor(2, FORWARD, 255);
  motor(1,BRAKE,0);
  }
  while (-ticksCodeurGauche < angle_desire_en_tic*2){
  motor(2, FORWARD, 255);
  motor(1,BRAKE,0);
  }
// motor(1, BRAKE, 0);
// motor(2, BRAKE, 0);

//  ticksCodeurDroit = 0;
//  ticksCodeurGauche = 0;
 motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);
    ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
}

void turn_gauche(float angle_en_deg){

int angle_desire_en_tic= angle_en_deg*deg+1650;

Serial.println(angle_desire_en_tic);
while (ticksCodeurDroit < 0.6*angle_desire_en_tic  && -ticksCodeurGauche < 0.6*angle_desire_en_tic){
    Serial.println(ticksCodeurGauche);
  Gauche(255);
    while (15+ticksCodeurDroit <  ticksCodeurGauche) {
    motor(1, BRAKE, 0);
    }
    while (15+ ticksCodeurGauche > ticksCodeurDroit ) {
    motor(2, BRAKE, 0);
    }


  }
  motor(1, RELEASE, 0);
  motor(2, RELEASE, 0);
 


while (ticksCodeurDroit < angle_desire_en_tic  && -ticksCodeurGauche < angle_desire_en_tic){
    Serial.println(ticksCodeurGauche);
  Gauche(255);
    while (15+ticksCodeurDroit <  ticksCodeurGauche) {
    motor(1, BRAKE, 0);
    }
    while (15+ ticksCodeurGauche > ticksCodeurDroit ) {
    motor(2, BRAKE, 0);
    }


  }

   motor(1, RELEASE, 0);
 motor(2, RELEASE, 0);
  ticksCodeurDroit = 0;
  ticksCodeurGauche = 0;
}

void Avancer(int speed)
{
  motor(1, BACKWARD, speed);  // Moteur 1 Avance
  motor(2, FORWARD, speed);   // Moteur 2 Avance
}
void Reculer(int speed)
{
  motor(1, FORWARD, speed);   // Moteur 1 Recule
  motor(2, BACKWARD, speed);  // Moteur 2 Recule
}
void Droite(int speed)
{
  motor(1, FORWARD, speed);   // Moteur 1 Droite
  motor(2, FORWARD, speed);   // Moteur 2 Droite
}
void Gauche(int speed)
{
  motor(1, BACKWARD, speed);   // Moteur 1 Gauche
  motor(2, BACKWARD, speed);   // Moteur 2 Gauche
}

void pas_monter_cannette(){
  val1 = analogRead(potpin1);            // reads the value of the potentiometer (value between 0 and 1023)
  val1 = map(val1, 0, 1023, 0, 0);     // scale it to use it with the servo (value between 0 and 180)
  myservo1.write(val1);                  // sets the servo position according to the scaled value
  delay(15);                  // sets the servo position according to the scaled value
           
}
void monter_cannette(){
  val1 = analogRead(potpin1);            // reads the value of the potentiometer (value between 0 and 1023)
  val1 = map(val1, 0, 1023, 0, 150);     // scale it to use it with the servo (value between 0 and 180)
  myservo1.write(val1);                  // sets the servo position according to the scaled value
  delay(15);                  // sets the servo position according to the scaled value
           
}
void monter_moitier_cannette(){
  val1 = analogRead(potpin1);            // reads the value of the potentiometer (value between 0 and 1023)
  val1 = map(val1, 0, 1023, 0, 120);     // scale it to use it with the servo (value between 0 and 180)
  myservo1.write(val1);                  // sets the servo position according to the scaled value
  delay(15);                  // sets the servo position according to the scaled value
           
}

void deplacer_cannette(int speed){
analogWrite(moteurB,255);
analogWrite(moteurB2,0); // vitesse max = valeur 255 - vitesse 0 = valeur 0
delay(600);
analogWrite(moteurB,0);
analogWrite(moteurB2,0);
}
void attraper_cannette_haut(){
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1000, 0, 1000);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15); 
}
void attraper_cannette_bas(){
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 0);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15); 
}
void pousser_cannette(int speed){
  motor(3, FORWARD, speed);   // Moteur 2 Avance
  delay(900);
  motor(3, BACKWARD, speed);   // Moteur 2 Avance
  delay(900);
}
void pousser_canard(int speed){
  motor(4, BACKWARD, speed);   // Moteur 2 Avance
  delay(3000);
  motor(4, BRAKE, speed);
  
}
void rentrer_canard(int speed){
  motor(4, FORWARD, speed);
  delay(3000);
}
void shiftWrite(int output, int high_low){
  static int latch_copy;
  static int shift_register_initialized = false;
  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);
    // start with all outputs (of the shift register) low
    latch_copy = 0;
    shift_register_initialized = true;
  }
  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
} 

void motor(int nMotor, int command, int speed){
  
  int motorA, motorB;
  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
      motorA = MOTOR1_A;
      motorB = MOTOR1_B;
      break;
      case 2:
      motorA = MOTOR2_A;
      motorB = MOTOR2_B;
      break;
      case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
      case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
      default:
      break;
    }
    switch (command)
    {
      case FORWARD:
      motor_output (motorA, HIGH, speed); 
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1); // -1: no PWM set
      break;
      case BRAKE:
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      case RELEASE:
      motor_output (motorA, LOW, 0); // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      default:
      break;
    }
  }
}

void motor_output (int output, int high_low, int speed){
  int motorPWM;
  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
    case MOTOR2_A:
    case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
    default:
    speed = -3333;
    break;
    case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  } 
  if (speed != -3333)
  {
  shiftWrite(output, high_low);
  // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}

void GestionInterruptionCodeurDroitPinA() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  if (digitalRead(codeurDroitPinA) == digitalRead(codeurDroitPinB)) {
    ticksCodeurDroit++;
  }
  else {
    ticksCodeurDroit--;
  }
}

void GestionInterruptionCodeurDroitPinB() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  if (digitalRead(codeurDroitPinA) == digitalRead(codeurDroitPinB)) {
    ticksCodeurDroit--;
  }
  else {
    ticksCodeurDroit++;
  }
}

void GestionInterruptionCodeurGauchePinA() {
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  if (digitalRead(codeurGauchePinA) == digitalRead(codeurGauchePinB)) {
    ticksCodeurGauche++;
  }
  else {
    ticksCodeurGauche--;
  }
}

void GestionInterruptionCodeurGauchePinB() {
  // Routine de service d'interruption attachée à la voie B du codeur incrémental gauche
  if (digitalRead(codeurGauchePinA) == digitalRead(codeurGauchePinB)) {
    ticksCodeurGauche--;
  }
  else {
    ticksCodeurGauche++;
  }
}


