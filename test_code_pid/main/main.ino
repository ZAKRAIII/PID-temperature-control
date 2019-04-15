#include <JC_Button.h>
#include <MAX6675_Thermocouple.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define SCK_PIN         4
#define CS_PIN          5
#define SO_PIN          2
// #define READINGS_NUMBER 7
// #define DELAY_TIME      15

MAX6675_Thermocouple* thermocouple = NULL;
LiquidCrystal_I2C lcd(0x27,20,4);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

//Pins
int PWM_pin = 3;

// Set Point
float set_temperature = 175;

//Variables
float suhu = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0, signal_value = 0;
bool buttUpFlag = false, buttDownFlag = false,buttSelFlag = false;
int menu = 0;

//button
#define buttUp    A1    //PC1 PCINT9  PCIE1
#define buttDown  A2    //PC2 PCINT10 PCIE1
#define buttSel   A0    //PC0 PCINT8  PCIE1
#define buttRst   7

Button myBtn(buttRst);

//PID constants
//from zero           10  0.035  40
//from minimum value  q
float kp = 33, ki = 0.01, kd = 10;
float fix_kp = 0.0, fix_ki = 0.0, fix_kd = 0.0, fix_set_temperature= 0.0;
float PID_p = 0, PID_i = 0, PID_d = 0;

void setup() {
  Serial.begin(57600);
//  thermocouple = new MAX6675_Thermocouple(
//   SCK_PIN, CS_PIN, SO_PIN,
//   READINGS_NUMBER, DELAY_TIME
//  );
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);

  // register button
  PCICR   |= (1 << PCIE1) ;
  PCMSK1  |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);

  pinMode(buttUp, INPUT);
  pinMode(buttDown, INPUT);
  pinMode(buttSel, INPUT);
  pinMode(PWM_pin,OUTPUT);

  // pin 3 and 11 PWM frequency of 928.5 Hz
  TCCR2B = TCCR2B & 0b11111000 | 0x03;  //0x03;

  analogWrite(PWM_pin, 255);

  myBtn.begin(); 
  
  fix_kp = kp;
  fix_ki = ki;
  fix_kd = kd;
  fix_set_temperature = set_temperature;

  lcd.init();
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("PLEASE WAIT!");
  delay(700);
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("SYSTEM READY");
  delay(600);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("PID TEMP Control");
  lcd.setCursor(0,1);
  lcd.print(" KELOMPOK - 2 ");
  delay(600);

  Time = millis();
}

void loop() {
  myBtn.read();
  if(myBtn.wasPressed()){
//    Serial.println("setP = " + String(set_temperature) + " P = " + String(kp) + " I = " + String(ki) + " D = " + String(kd) + " " );
    lcd.clear();
    lcd.setCursor(5,0);
//    lcd.print("RESET");
    set_temperature = fix_set_temperature;
    kp = fix_kp;
    ki = fix_ki;
    kd = fix_kd;
    PID_p = 0;
    PID_i = 0;
    PID_d = 0;
    Time = millis();
    
    delay(300);
    
  }
  /*==================================================================
                        READ REAL TEMPERATURE
  ===================================================================*/
  suhu = thermocouple->readCelsius();

/*==================================================================
                           PID VALUE CALC
  ===================================================================*/
  PID_error = set_temperature - suhu;

  //calculate Proportional value value
  PID_p = kp * PID_error;

  //calculate Ingtegral value
  PID_i = PID_i + (ki * PID_error);
  

  //calculate Derivative value
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;

  PID_d = kd * ((PID_error - previous_error) / elapsedTime);

  //total PID value, sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //define PWM range between 0 and 255, for maximum power
  if(PID_value < 50){
    PID_value += 55;
    Serial.println("++");
  }
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > 255){
    PID_value = 255;
  }
  
  

/*==================================================================
                          SIGNAL TO HEATER
  ===================================================================*/
  signal_value = 255-PID_value;
  analogWrite(PWM_pin, signal_value);

/*==================================================================
                       VALUE FOR NEXT PID CALC
  ===================================================================*/
  previous_error = PID_error;

//  debug();
  ser_plot();

  if(buttSelFlag == true){
    menu++;
    buttSelFlag = false;
    delay(100);
  }

  if(menu > 3){
    menu = 0;
  }

  if (menu == 0){
  /*==================================================================
                                CHANGE VAL TEMP
    ===================================================================*/
    if(buttUpFlag == true){
      set_temperature++;
      buttUpFlag = false;
      delay(100);
    }
    if(buttDownFlag == true){
      set_temperature--;
      buttDownFlag = false;
      delay(100);
    }

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PID TEMP Control");
    lcd.setCursor(0,1);
    lcd.print("S:");
    lcd.setCursor(2,1);
    lcd.print(set_temperature,1);
    lcd.setCursor(9,1);
    lcd.print("R:");
    lcd.setCursor(11,1);
    lcd.print(suhu,1);
  }

  if (menu == 1){
    if(buttUpFlag == true){
      kp++;
      buttUpFlag = false;
      delay(100);
    }
    if(buttDownFlag == true){
      kp--;
      buttDownFlag = false;
      delay(100);
    }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Set   P  value  ");
      lcd.setCursor(5,1);
      lcd.print(kp);
  }

  if (menu == 2){
    if(buttUpFlag == true){
      ki += 0.1;
      buttUpFlag = false;
      delay(100);
    }
    if(buttDownFlag == true){
      ki -= 0.1;
      buttDownFlag = false;
      delay(100);
    }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Set   I  value  ");
      lcd.setCursor(5,1);
      lcd.print(ki);
  }

  if (menu == 3){
    if(buttUpFlag == true){
      kd++;
      buttUpFlag = false;
      delay(100);
    }
    if(buttDownFlag == true){
      kd--;
      buttDownFlag = false;
      delay(100);
    }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Set   D  value  ");
      lcd.setCursor(5,1);
      lcd.print(kd);
  }
  delay(300);
}
