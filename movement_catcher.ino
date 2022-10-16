#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

//init  IMU variables and such 
MPU6050 IMU;
int16_t ax, ay, az, rx, ry, rz; //a_ = acceleration, m_ = rotation in direction

//button
int button1 = 1;
int state = 0;
int lastState;

//signal to be stored and read
int16_t dataSet[500][6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  while(!Serial); //wait until connection established with serial port
  pinMode(button1, INPUT_PULLUP);
  //turn on IMU
  IMU.initialize();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //printData();
  //delay(100);
  //plotGyro();
  //plotAccel();
  
  //take readings while pressed
  int count = 0;
  while (digitalRead(button1) == 0){
    IMU.getMotion6(&ax, &ay, &az, &rx, &ry, &rz);
    int16_t dataPoint[6] = {ax, ay, az, rx, ry, rz};
    for (int e = 0; e < 6; e++){
      dataSet[count][e] = dataPoint[e];
    }
    count++;
    lastState = 1;
    
    delay(50);
  } 
  
  if (lastState == 1){
    printData();
    lastState = 0;
  }
  
}

void printData() {
  // display tab-separated accel/gyro x/y/z values
  
  for (int e = 0; e < 500; e++){
    for (int a = 0; a < 5; a++){
      Serial.print(dataSet[e][a]);
      Serial.print(", ");
    }
    Serial.println(dataSet[e][5]);
  }
  Serial.println("End of signal");
}

void plotAccel() {
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.println(az);
  delay(50); // run at ~100 Hz
}

void plotGyro() {
  Serial.print(rx); Serial.print(",");
  Serial.print(ry); Serial.print(",");
  Serial.println(rz);
  delay(50); //
}
