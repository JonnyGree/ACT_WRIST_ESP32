#include "Parameters.h"

boolean DEBUG_ON = true;

actuator_current_state act_state = STATUS_OK;
actuator_current_mode act_mode = MODE_INIT;

motor_data motor;

void initpin(){
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
 
}
