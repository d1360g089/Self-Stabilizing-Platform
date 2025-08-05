
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <Servo.h>



#define BNO055_SAMPLERATE_DELAY_MS (15)

Adafruit_BNO055 myIMU = Adafruit_BNO055();


Servo pitchServo;
Servo rollServo;

int servoPin1 = 9;
int servoPin2 = 8;


float deltaPos = 1;



float thetaM; //angle(tilt) in the x-direction
float phiM; // angle (tilt) in the y direction


// Low Pass filter variables
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;


//Gyro variables
float gyroTheta = 0;
float gyroPhi = 0;


//Complimentary filter Variables -> Overall System
float theta = 0;
float phi = 0;


//Radian version
float thetaRad;
float phiRad;


//Magnetometer Values
float Xm;    // x-direction
float Ym;   //y-direction
float psi;   //z-direction



float dt;
unsigned long millisOld;





//Quaternions
float qW;
float qX;
float qY;
float qZ;




//Control system


//PID vars
float k_p = .75;
float k_d = .01; // derivate constant 
float k_i = 0; // integral constant


float rollTarget = 0;
float rollActual;
float rollError = 0;
float rollErrorOld;
float rollErrorChange;
float rollErrorSlope = 0;
float rollErrorArea = 0;
float rollServoPos = 90;



float pitchActual;
float pitchTarget = 0;
float pitchError = 0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope = 0;
float pitchErrorArea = 0;
float pitchServoPos = 90;








void setup() {
  Serial.begin(115200);
  myIMU.begin();
  millisOld = millis();
  pitchServo.attach(servoPin2);
  rollServo.attach(servoPin1);
  

}

void loop() {


  uint8_t system, gyr, accel, mg = 0;

  //Calibration will get us more accurate numbers
  myIMU.getCalibration(&system, &gyr, &accel, &mg); // 0-3   


  //Getting acceleration
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  //Getting gyro (angular velocity)
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  //getting magnetometer
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  


  // Getting the angle of tilt
  thetaM  = -atan2(acc.x()/9.8,acc.z()/9.8) / 2 / 3.141592654 *360;
  phiM  = -atan2(acc.y()/9.8, acc.z()/9.8) / 2 / 3.141592654 *360;


  //Getting the angular velocity
  
  dt = (millis() - millisOld)/1000.; //t1 - t0
  millisOld = millis();
  gyroTheta = gyroTheta + gyro.y()*dt;
  gyroPhi = gyroPhi - gyro.x()*dt;



  //we will apply a low pass filter to filter out high frequency data(rapid moving signals) and remove noise so that we the
  // axis tilts dont interfere with one another
  //Low pass filter
  phiFnew = .9*(phiFold) + .1*(phiM);
  thetaFnew = .9*(thetaFold) + .1*(thetaM);

  // We need to apply a complimentary filter to combine gyro and accelerometer data to
  // be able to get the best of both worlds
  // gyro - good short term - bad long term
  // accel - good long term - bad short term

  
  theta = .85*(theta + gyro.y()*dt) + .15*(thetaFnew);
  phi = .85*(phi + gyro.x()*dt) + .15*(phiM);

  phiRad = phi/360*(2*3.14);
  thetaRad = theta/360*(2*3.14);

  Xm = mag.x()*cos(thetaRad) - mag.y()*sin(phiRad) * sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
  Ym = mag.y() * cos(phiRad) + mag.z()*sin(phiRad);

  psi = atan2(Ym, Xm) / (2*3.14) * 360;

  
  
  imu::Quaternion quat = myIMU.getQuat();

  qW = quat.w();
  qX = quat.x();
  qY = quat.y();
  qZ = quat.z();

  rollActual = atan2(2*(qW*qX + qY*qZ), 1-2*(qX*qX + qY*qY));
  pitchActual = asin(2*(qW*qY - qX*qZ));
  //convert to degrees
  rollActual = rollActual/(2*3.14159265) * 360;
  pitchActual = pitchActual/(2*3.14159265) * 360;

  //roll parameters
  rollErrorOld = rollError; 
  rollError = rollTarget - rollActual;
  rollErrorChange = rollError - rollErrorOld;
  rollErrorSlope = rollErrorChange/dt; // de(t)/dt
  
  rollErrorArea = rollErrorArea + rollError * dt; // Integral of e(t) dt

  //pitch parameters
  pitchErrorOld = pitchError; 
  pitchError = pitchTarget - pitchActual;
  pitchErrorChange = pitchError - pitchErrorOld;
  pitchErrorSlope = pitchErrorChange/dt; // de(t)/dt
  
  pitchErrorArea = pitchErrorArea + pitchError * dt; // Integral of e(t) dt


  
  if (abs(rollError) > 2){
    rollServoPos = rollServoPos + rollError/6 + k_d*(rollErrorSlope);
    rollServo.write(rollServoPos);
    delay(15);
  }

  if(abs(pitchError)> 2){
    pitchServoPos = pitchServoPos + (pitchError)/6 + k_d*(pitchErrorSlope);
    pitchServo.write(pitchServoPos);
    delay(15);
  }


  
  
  
  
    
 
  
  //pitchServoPos = pitchServoPos + k_p*(pitchError) + k_d*(pitchErrorSlope) + k_i*(pitchErrorArea);
  //pitchServo.write(pitchServoPos);




  
  

  /*
  servoPos1 += deltaPos;
  myServo1.write(servoPos1);

  if (servoPos1 >=180){
    servoPos1 = 180;
    deltaPos *= -1;

  }

  if (servoPos1 <= 0){
    servoPos1 = 0;
    deltaPos *= -1;

  }
  delay(10);
  */
  


  Serial.print(rollTarget);
  Serial.print(",");
  Serial.print(rollActual);
  Serial.print(",");
  Serial.print(pitchTarget);
  Serial.print(",");
  Serial.print(pitchActual);
  Serial.print(",");
  Serial.println(rollServoPos);

 
  



}
