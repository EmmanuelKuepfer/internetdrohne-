///////////////////////////////
// sensor.cpp - Berechnung der Winkelwerte anhand der Sensorwerte
// Emmanuel Kuepfer
//
// Übernommen aus dem Beispiel der sparcfun Bibliothek
// https://github.com/sparkfun/MPU-9250_Breakout/blob/master/Libraries/Arduino/examples/MPU9250BasicAHRS/MPU9250BasicAHRS.ino
///////////////////////////////

// System Headerfiles
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "math.h"
#include "application.h"
#include "sensor.h"

// Deklination Magnetfeld (Wert muss nach geografischer Lage angepasst werden)
#define DECLINATION 2.12

// Definition der Klasse "MPU9250" mit der Variabel "sensor".
MPU9250 sensor;

// Intitialisierung Sensor MPU9250
void sensorInit()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(D6, INPUT);
  digitalWrite(D6, LOW);
  pinMode(D7, OUTPUT);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = sensor.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    sensor.MPU9250SelfTest(sensor.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(sensor.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(sensor.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(sensor.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(sensor.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(sensor.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(sensor.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    sensor.calibrateMPU9250(sensor.gyroBias, sensor.accelBias);

    sensor.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = sensor.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    sensor.initAK8963(sensor.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(sensor.magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(sensor.magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(sensor.magCalibration[2], 2);
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

// Inhalt aus Beispiel von SparkFun
void sensorUpdate()
{
  sensor.readAccelData(sensor.accelCount);  // Read the x/y/z adc values
  sensor.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  sensor.ax = (float)sensor.accelCount[0]*sensor.aRes; // - accelBias[0];
  sensor.ay = (float)sensor.accelCount[1]*sensor.aRes; // - accelBias[1];
  sensor.az = (float)sensor.accelCount[2]*sensor.aRes; // - accelBias[2];

  sensor.readGyroData(sensor.gyroCount);  // Read the x/y/z adc values
  sensor.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  sensor.gx = (float)sensor.gyroCount[0]*sensor.gRes;
  sensor.gy = (float)sensor.gyroCount[1]*sensor.gRes;
  sensor.gz = (float)sensor.gyroCount[2]*sensor.gRes;

  sensor.readMagData(sensor.magCount);  // Read the x/y/z adc values
  sensor.getMres();
  // User environmental x-axis correction in milliGauss, should be
  // automatically calculated
  sensor.magbias[0] = +470.;
  // User environmental x-axis correction in milliGauss TODO axis??
  sensor.magbias[1] = +120.;
  // User environmental x-axis correction in milliGauss
  sensor.magbias[2] = +125.;

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  sensor.mx = (float)sensor.magCount[0]*sensor.mRes*sensor.magCalibration[0] -
             sensor.magbias[0];
  sensor.my = (float)sensor.magCount[1]*sensor.mRes*sensor.magCalibration[1] -
             sensor.magbias[1];
  sensor.mz = (float)sensor.magCount[2]*sensor.mRes*sensor.magCalibration[2] -
             sensor.magbias[2];

  // Must be called before updating quaternions!
  sensor.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(sensor.ax, sensor.ay, sensor.az, sensor.gx*DEG_TO_RAD,
                         sensor.gy*DEG_TO_RAD, sensor.gz*DEG_TO_RAD, sensor.my,
                         sensor.mx, sensor.mz, sensor.deltat);

  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  sensor.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  sensor.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                *(getQ()+2)));
  sensor.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  sensor.pitch *= RAD_TO_DEG;
  sensor.yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  sensor.yaw   += DECLINATION;
  sensor.roll  *= RAD_TO_DEG;
}
