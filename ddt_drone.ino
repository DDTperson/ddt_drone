#include"Gyro.h"
#include"PID.h"
#include<Servo.h>

#define FLONT_LEFT_MOTER_PIN 3
#define FLONT_RIGHT_MOTER_PIN 10
#define BACK_LEFT_MOTER_PIN 11
#define BACK_RIGHT_MOTER_PIN 9

#define MIN_PULS 1000
#define MAX_PULS 2000

enum MoterPIN { FLONT_LEFT, FLONT_RIGHT, BACK_LEFT, BACK_RIGHT };

Servo MOTER[4];

Gyro gyro_data;
PID pid_X, pid_Y;

void setup() 
{
  Serial.begin(9600);
  gyro_data.initGyro();
  gyro_data.CalibrationGyro();
  Serial.println("GyroScope Setup Compelete...");

  MOTER[FLONT_LEFT].attach(FLONT_LEFT_MOTER_PIN, MIN_PULS, MAX_PULS);
  MOTER[BACK_RIGHT].attach(BACK_RIGHT_MOTER_PIN, MIN_PULS, MAX_PULS);
  MOTER[FLONT_RIGHT].attach(FLONT_RIGHT_MOTER_PIN, MIN_PULS, MAX_PULS);
  MOTER[BACK_LEFT].attach(BACK_LEFT_MOTER_PIN, MIN_PULS, MAX_PULS);
  
  for(int chenal = FLONT_LEFT; chenal <= BACK_RIGHT; chenal++)
    MOTER[chenal].writeMicroseconds(MAX_PULS);

  delay(9000);

  for(int chenal = FLONT_LEFT; chenal <= BACK_RIGHT; chenal++)
    MOTER[chenal].writeMicroseconds(MIN_PULS);
    
  pid_X.i_controll.start_dt =  millis();
  pid_X.d_controll.start_dt =  millis();
  pid_Y.i_controll.start_dt =  millis();
  pid_Y.d_controll.start_dt =  millis();
}

float pid_power[4] = { 0 };
int throthle = 0;
float pid_output_X = 0, pid_output_Y = 0;
bool calib = false;

void loop()
{
  gyro_data.GetDegree();
  pid_X.i_controll.dt =  (millis()-pid_X.i_controll.start_dt)/1000;
  pid_X.d_controll.dt =  (millis()-pid_X.d_controll.start_dt)/1000;
  pid_Y.i_controll.dt =  (millis()-pid_Y.i_controll.start_dt)/1000;
  pid_Y.d_controll.dt =  (millis()-pid_Y.d_controll.start_dt)/1000;
  
  if(Serial.available())
  {
    char key = Serial.read();

    if(key == 'c')
    {
      gyro_data.CalibrationGyro();
      pid_X.initPID(gyro_data.data.degree[X]);
      pid_Y.initPID(gyro_data.data.degree[Y]);
      calib = true;
    }
    else if(key == 'p')
      throthle++;
    else if(key == 'l')
      throthle--;
    else
      Serial.println(throthle);
  }

  if(calib)
  {
    pid_X.upDate(gyro_data.data.degree[X],gyro_data.data.value[GYRO][Y]);
    pid_Y.upDate(gyro_data.data.degree[Y],gyro_data.data.value[GYRO][X]);
    
    pid_output_X = pid_X.OutPut(aPI) + pid_X.OutPut(rPD);
    pid_output_Y = pid_Y.OutPut(aPI) + pid_Y.OutPut(rPD);
    
    pid_power[FLONT_LEFT] = throthle + pid_output_X - pid_output_Y;
    pid_power[FLONT_RIGHT] = throthle - pid_output_X - pid_output_Y;
    pid_power[BACK_LEFT] = throthle + pid_output_X + pid_output_Y;
    pid_power[BACK_RIGHT] = throthle - pid_output_X+ pid_output_Y;

    if(throthle <= 0)
      for(int chenal = FLONT_LEFT; chenal <= BACK_RIGHT; chenal++)
        MOTER[chenal].writeMicroseconds(MIN_PULS);
    else
      for(int chenal = FLONT_LEFT; chenal <= BACK_RIGHT; chenal++)
        MOTER[chenal].writeMicroseconds(MIN_PULS+pid_power[chenal]);
        
    Serial.println(pid_power[FLONT_LEFT]);
  }

  pid_X.i_controll.start_dt =  millis();
  pid_X.d_controll.start_dt =  millis();
  pid_Y.i_controll.start_dt =  millis();
  pid_Y.d_controll.start_dt =  millis();
}
