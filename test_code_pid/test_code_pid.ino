#include <MAX6675_Thermocouple.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define SCK_PIN         6
#define CS_PIN          5
#define SO_PIN          4
#define READINGS_NUMBER 7
#define DELAY_TIME      20

MAX6675_Thermocouple* thermocouple = NULL;
LiquidCrystal_I2C lcd(0x27,20,4);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

//Pins
int PWM_pin = 3;
// Set Point
float set_temperature = 75;

//Variables
float suhu = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
bool buttUpFlag = false, buttDownFlag = false,buttSelFlag = false, eepromFlag = false;
int menu = 0;

//PID constants
float kp = 30.0, ki = 0.2, kd = 5;  //30  0 0
float PID_p = 0, PID_i = 0, PID_d = 0;
// int last_kp = 0;

//button
#define buttUp    A1    //PC1 PCINT9  PCIE1
#define buttDown  A2    //PC2 PCINT10 PCIE1
#define buttSel   A0    //PC0 PCINT8  PCIE1

void setup() {
  Serial.begin(9600);
  // thermocouple = new MAX6675_Thermocouple(
  //   SCK_PIN, CS_PIN, SO_PIN,
  //   READINGS_NUMBER, DELAY_TIME
  // );
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

  analogWrite(PWM_pin, 255-PID_value);

  lcd.init();
  lcd.backlight();

  // if(digitalRead(buttSel)==HIGH){
  //   while(1){
  //     lcd.print("reset data log");
  //     delay(700);
  //     EEPROM.write(1, set_temperature);
  //     EEPROM.write(3, kp);
  //     EEPROM.write(5, ki);
  //     EEPROM.write(7, kd);
  //     while(!false){
  //       lcd.setCursor(1,1);
  //       lcd.print("-PLEASE RESTART-");
  //     }
  //   }
  // }

  // set_temperature = EEPROM.read(1);
  // kp = EEPROM.read(3);
  // ki = EEPROM.read(5);
  // kd = EEPROM.read(7);

  lcd.setCursor(2,0);
  lcd.print("PLEASE WAIT!!");
  delay(1500);
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("SYSTEM READY");
  delay(1000);
  lcd.clear();

  Time = millis();
}

void loop() {
  /*==================================================================
                        READ REAL TEMPERATURE
  ===================================================================*/
  suhu = thermocouple->readCelsius();
//   Serial.println(suhu);

/*==================================================================
                           PID VALUE CALC
  ===================================================================*/
  PID_error = set_temperature - suhu;
//   Serial.println(PID_error);

  //calculate Proportional value value
  PID_p = kp * PID_error;
  //Serial.println(PID_p);

  //calculate Ingtegral value
  PID_i = PID_i + (ki * PID_error);
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
  Serial.println(PID_value);

/*==================================================================
                          SIGNAL TO HEATER
  ===================================================================*/
  analogWrite(PWM_pin, 255-PID_value);

/*==================================================================
                       VALUE FOR NEXT PID CALC
  ===================================================================*/
  previous_error = PID_error;

  if(buttSelFlag == true){
    buttSelFlag = false;
    menu++;
    delay(100);
  }

  if(menu > 3){
    menu = 0;
    // eepromFlag = true;
  }

  // if(eepromFlag == true){
  //   EEPROM.write(1, set_temperature);
  //   EEPROM.write(3, kp);
  //   EEPROM.write(5, ki);
  //   EEPROM.write(7, kd);
  //   eepromFlag = false;
  //   lcd.clear();
  //   lcd.setCursor(2,0);
  //   lcd.print("eeprom writed");
  //   delay(700);
  // }

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
    lcd.print("PID TEMP control");
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
      // last_kp = kp;
  }

  if (menu == 2){
    if(buttUpFlag == true){
      ki = ki + 0.2;
      buttUpFlag = false;
      delay(100);
    }
    if(buttDownFlag == true){
      ki = ki - 0.2;
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
