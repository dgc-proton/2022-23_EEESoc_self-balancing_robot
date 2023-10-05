// motor definitions
// left motor definitions
#define LMOTOR_PWM_PIN   3
#define LMOTOR_DIR_PIN_1 4 
#define LMOTOR_DIR_PIN_2 5
// right motor definitions
#define RMOTOR_DIR_PIN_1 6 
#define RMOTOR_DIR_PIN_2 7
#define RMOTOR_PWM_PIN   9

void setup() {
  // put your setup code here, to run once:
  pinMode(LMOTOR_DIR_PIN_1, OUTPUT);
  pinMode(LMOTOR_DIR_PIN_2, OUTPUT);
  pinMode(LMOTOR_PWM_PIN, OUTPUT); 
  pinMode(RMOTOR_DIR_PIN_1, OUTPUT);
  pinMode(RMOTOR_DIR_PIN_2, OUTPUT);
  pinMode(RMOTOR_PWM_PIN, OUTPUT); 
  Serial.begin(9600); // open the serial data port at 9600 bps

  digitalWrite(LMOTOR_DIR_PIN_1, LOW);
  digitalWrite(LMOTOR_DIR_PIN_2, HIGH);
  digitalWrite(RMOTOR_DIR_PIN_1, LOW);
  digitalWrite(RMOTOR_DIR_PIN_2, HIGH);

  analogWrite(RMOTOR_PWM_PIN, 80);
  analogWrite(LMOTOR_PWM_PIN, 80);
}

void loop() {
  // put your main code here, to run repeatedly:
  // set motor direction
   /* digitalWrite(LMOTOR_DIR_PIN_1, HIGH);
    digitalWrite(LMOTOR_DIR_PIN_2, LOW);
    digitalWrite(RMOTOR_DIR_PIN_1, HIGH);
    digitalWrite(RMOTOR_DIR_PIN_2, LOW); */

        // set motors in forward direction

/*
  for (int i = 40; i<110; i++) {
    Serial.println(i);
    analogWrite(LMOTOR_PWM_PIN, i);
    analogWrite(RMOTOR_PWM_PIN, i);
    delay(500);
  }*/
}
