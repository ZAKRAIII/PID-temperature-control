#include <MAX6675_Thermocouple.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SCK_PIN 5
#define CS_PIN 7
#define SO_PIN 6
#define READINGS_NUMBER 7
#define DELAY_TIME 20

MAX6675_Thermocouple* thermocouple = NULL;
LiquidCrystal_I2C lcd(0x27,20,4);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

//Pins
int PWM_pin = 3;

// Set Point
float set_temperature = 100;

//Variables
float temp = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;

//PID constants
int kp = 30, ki = 0, kd = 0;  //30  0 0
int PID_p = 0, PID_i = 0, PID_d = 0;
float last_kp = 0, last_ki = 0, last_kd = 0;
int PID_values_fixed =0;

// rotary encoder Variables
int clk_State;
int Last_State;
bool dt_State;
int clk = 8;      //Pin 1 from rotary encoder
int data = 9;     //Pin 2 from rotary encoder

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  thermocouple = new MAX6675_Thermocouple(
    SCK_PIN, CS_PIN, SO_PIN,
    READINGS_NUMBER, DELAY_TIME
  );
//  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);

  pinMode(PWM_pin,OUTPUT);

//  pin 3 and 11 PWM frequency of 928.5 Hz
  TCCR2B = TCCR2B & B11111000 | 0x03;

//  set encoder
  Last_State = (PINB & B00000001);      //Detect first state of the encoder
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);  //Set pin D11 trigger an interrupt on state change.
  pinMode(11,INPUT);
  pinMode(data,INPUT);
  pinMode(clk,INPUT);
  
  lcd.init();
  lcd.backlight();
  Time = millis();
}

void loop() {
/*==================================================================
                      READ REAL TEMPERATURE
===================================================================*/
  temp = thermocouple->readCelsius();
  Serial.println(temp);

/*==================================================================
                           PID VALUE CALC
  ===================================================================*/
  PID_error = set_temperature - temp;
  Serial.println(PID_error);

  //calculate Proportional value value
  PID_p = kp * PID_error;
  //Serial.println(PID_p);

  //calculate Ingtegral value in a range on +-3
  if(-3 < PID_error <3){
    PID_i = PID_i + (ki * PID_error);
  }
  //Serial.println(PID_i);

  //calculate Derivative value
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;

  PID_d = kd * ((PID_error - previous_error) / elapsedTime);
  //Serial.println(PID_d);

  //total PID value, sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;
    // PID_value = PID_p /*+ PID_i*/ + PID_d;
  //Serial.println(PID_value);

  //define PWM range between 0 and 255, for maximum power
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > 255){
    PID_value = 255;
  }
  // else if(PID_value > 255){
  //   PID_value = 255;
  // }

/*==================================================================
                          SIGNAL TO HEATER
  ===================================================================*/
  analogWrite(PWM_pin, 255-PID_value);

/*==================================================================
                       VALUE FOR NEXT PID CALC
  ===================================================================*/
  previous_error = PID_error;




/*==================================================================
                              TO LCD
  ===================================================================*/
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PID TEMP control");
  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.setCursor(2,1);
  lcd.print(set_temperature,1);
  lcd.setCursor(9,1);
  lcd.print("R:");
  lcd.setCursor(11,1);
  lcd.print(temp,1);

  // delay(300);
  delay(150);
}
