#include <Adafruit_Sensor.h>
#include <Servo.h>
#include "PID_v1.h"
#include <LoRa.h>
#include <Wire.h>    //Include wire library 
//#include "MPU6050_light.h"  //Include library for MPU communication  //Library for LCD Display
#include <Adafruit_BMP280.h>

#include <mpu6050.h>

#define MPU_ADDRESS 0x68 //  mpu6050 address is 0x69 if AD0 pin is powered -  otherwise it's 0x68

float rawGX, rawGY, rawGZ; // initialise raw gyroscope variables
float dpsGX, dpsGY, dpsGZ; // initialise dps gyroscope variables
float gRoll, gPitch, gYaw; // initialise actual gyroscope variables
float rawAX, rawAY, rawAZ; // initialise raw accelerometer variables
float gForceAX, gForceAY, gForceAZ;  // initialise g force accelerometer variables
float aPitch, aRoll; // initialise actual accelerometer variables
double gyroOffsetX, gyroOffsetY, gyroOffsetZ;  // initialise gyroscope offset variables
double accelOffsetX, accelOffsetY, accelOffsetZ;  // initialise accelerometer offset variables


Servo ServoX1, ServoY1, parachuteServo;
//MPU6050 mpu(Wire);   //Create object mpu
    //Define LCD address and dimension
Adafruit_BMP280 bmp;

const int servoPin = 9;
const float seaLevelPressure = 1013.25;
float maxAltitude = 0;
bool apogeeReached = false;
int i;
float h;
double setpoint;
double ixangle = 0.0, oxangle = 0.0;
double iyangle = 0.0, oyangle = 0.0;
double kp = 2, ki = 1.15, kd = 3;
double altitude,verticalAcceleration;
#define PIN_INPUT 0
#define PIN_OUTPUT 3
PID myPIDX(&ixangle, &oxangle, &setpoint, kp, ki, kd, DIRECT);
PID myPIDY(&iyangle, &oyangle, &setpoint, kp, ki, kd, DIRECT);


unsigned long timer = 0;    

void setup() {
  ServoX1.attach(2);
  ServoY1.attach(4);
  Serial.begin(9600);    //Start serial communication
   
  setpoint = 0.00;
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);

  // mpu.begin();     
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(F("Calculating offsets, do not move MPU6050"));   
  // delay(1000);
  // mpu.calcGyroOffsets();     //Calibrate gyroscope
  // Serial.println("Done!\n");
  // bmp.begin(BMP280_ULTRALOWPOWER);

  wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode
  delay(1000);
  calculateGyroOffset(MPU_ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ); // provide MPU6050 address and gyroscope values are written to 3 provided variables
  calculateAccelOffset(MPU_ADDRESS, accelOffsetX, accelOffsetY);

  if (!bmp.begin(0x76)) 
  {
    //Serial.println("BMP280 initialization failed!");
    while (1);
  }
  
  //Serial.println("BMP280 Initialized");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                Adafruit_BMP280::SAMPLING_X2,
                Adafruit_BMP280::SAMPLING_X16,
                Adafruit_BMP280::FILTER_X16,
                Adafruit_BMP280::STANDBY_MS_1);
  
  
  
  parachuteServo.attach(servoPin);
  parachuteServo.write(0);

  //Serial.println("");
  delay(100);

}
void loop() {
  
  // dynamically find delta time (Δt/dt)
    static unsigned long lastTime = millis(); // only set once
    unsigned long currentTime = millis(); 
    float dt = (currentTime - lastTime) / 1000; // finds Δt and converts to seconds from milliseconds
    lastTime = currentTime; // last time updated for next loop's calculations

    readGyroData(MPU_ADDRESS, rawGX, rawGY, rawGZ); // pass MPU6050 address and gyroscope values are written to 3 provided variables
    rawGyroToDPS(rawGX, rawGY, rawGZ, dpsGX, dpsGY, dpsGZ); // provide the 3 raw gyroscope values and returns them in their dps (degrees per second) values
    readAccelData(MPU_ADDRESS, rawAX, rawAY, rawAZ); // pass MPU6050 address and accelerometer values are written to 3 provided variables
    rawAccelToGForce(rawAX, rawAY, rawAZ, gForceAX, gForceAY, gForceAZ); // provide the 3 raw accelerometer values and returns them in their g force values
    calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, aPitch, aRoll); // uses trigonometry to calculate angles with accelerometer values 

    dpsGX = dpsGX - gyroOffsetX; // adjust gyroscope values to compensate for offset values
    dpsGY = dpsGY - gyroOffsetY;
    dpsGZ = dpsGZ - gyroOffsetZ;

    aPitch = aPitch - accelOffsetX; // adjust accelerometer values to compensate for offset values
    aRoll = aRoll - accelOffsetY;

    // alpha is the weight for the gyroscope compared the accelerometer
    // helps tune the value although common values are 0.95 - 0.98
    // -- GUIDE TO WEIGHTING --
    // The higher the alpha the quicker the responsiveness and less noise 
    // but drift over time more and are more susceptible to gyro bias if offsets are offset
    float alpha = 0.95; 

    float filteredPitch = aPitch;
    float filteredRoll = aRoll;

    complementaryFilter(dpsGX, aPitch, alpha, dt, filteredPitch);
    complementaryFilter(dpsGY, aRoll, alpha, dt, filteredRoll);

    //prints mpu6050 values in the terminal
    // Serial.print("filtered pitch:");
    // Serial.print(filteredPitch);
    // Serial.print("/");
    // Serial.print("filtered Roll:");
    // Serial.println(filteredRoll);

  // --- Add this block to read and print BMP280 data ---
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  altitude = bmp.readAltitude(seaLevelPressure); // This assigns the reading to your global 'altitude' variable
 /*
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C\t"); 

  Serial.print("Pressure: ");
  Serial.print(pressure / 100.0F); 
  Serial.print(" hPa\t");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  // --- End of new block ---*/

  pinMode(LED_BUILTIN, OUTPUT);
  { // print data every 100ms
    ixangle = KALMAN(filteredPitch, 1);
    iyangle = KALMAN(filteredRoll, 2);

    myPIDX.Compute();
    myPIDY.Compute();

    ServoX1.write(oxangle);
    ServoY1.write(oyangle);
  }

  // This apogee logic can now work because 'altitude' has a value
  if (altitude > maxAltitude) {
    maxAltitude = altitude;
  } else if (verticalAcceleration < 1.1 && verticalAcceleration > 0.9 && !apogeeReached) {
    // Apogee detected
    apogeeReached = true;
    deployParachute();
  }
}
double KALMAN(double U, int axis)
{
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P1 = 0;
  static double U_hat1 = 0;
  static double K1 = 0.2;
  static double P2 = 0;
  static double U_hat2 = 0;
  static double K2 = 0.2;
  if (axis == 1)
  {
  K1 = P1*H/(H*P1*H+R);
  U_hat1 += K1*(U-H*U_hat1);
  P1 = (1-K1*H)*P1+Q;
  // Serial.print("kalmanx:");
  // Serial.print(U_hat1);
  // Serial.print(",");
  return U_hat1;
  }
  else if(axis==2)
  {
  K2 = P2*H/(H*P2*H+R);
  U_hat2 += K2*(U-H*U_hat2);
  P2 = (1-K2*H)*P2+Q;
  // Serial.print("kalmanZ:");
  // Serial.print(U_hat2);
  // Serial.print(",");
  return U_hat2;

  }
  return 0;
}


void deployParachute() {
  //Serial.println("Apogee reached! Deploying parachute...");
  parachuteServo.write(180);  // Rotate servo 180 degrees
  
  // Send apogee event via LoRa
  LoRa.beginPacket();
  LoRa.print("APOGEE");
  LoRa.endPacket();
}