/* Example Arduino code for one UM7 sending IMU binary packets at 255 Hz over UART.
 * A revision of the library written by Mike Hoyer: https://github.com/mikehoyer/UM7-Arduino.git
 * Implemented all read/write functions, all configuration settings, and all possible 
 * datasets. See README for a complete list.
 * 
 * Table for binary packet rate over UART (1 start bit + 8 data bits + 1 stop bit)/Byte
 *  |    Baud    |    Rate    |    B/transfer   |   Registers/transfer
 *       9600          255            3.7                  0.9
 *      14400          255            5.6                  1.4
 *      19200          255            7.5                  1.9
 *      38400          255           15.1                  3.8              
 *      57600          255           22.6                  5.6
 *     115200          255           45.2                 11.3
 *     128000*         255           50.2                 12.6
 *     153600*         255           60.2                 15.1
 *     230400*         255           90.4                 22.6
 *     256000*         255          100.4                 25.1
 *     460800*         255          180.7                 45.2
 *     921600*         255          361.4                 90.4
 *  *Most MCUs won't support these higher baud rates, you require a high speed UART.
 * 
 */
#include <UM7.h>

// Setup one UM7 on Serial1 (TX1/RX1)
MYUM7 imu(Serial1);
long init;

void setup() {
  // Set baud rate between PC and MCU
  Serial.begin(115200);
  
  // Set baud rate on Serial1 to read at 115200 bps for the MCU
  Serial1.begin(115200);
  delay(100);

  // Set the UM7 baud rate to output at 115200 bps to match the MCU
  imu.set_sensor_baud_rate(115200);
  delay(100);
  
  // Set euler and all processed datasets to 255 Hz
  imu.set_euler_rate(255);
  delay(100);
  imu.set_all_processed_rate(255);
  delay(100);
  
  // Can also set processed rates individually, (accelerometer, gyroscope, magnetometer)
  // to accel @ 100 Hz, gyro @ 200 Hz, mag @ 255 Hz:
  // imu.set_processed_rate(100, 200, 255);

  // Zero Gyros and Calibrate Accelerometers
  // Note: Calibration sets the biasing for the XYZ accel vectors, it will rotate the coordinate system to center about [0,0,1](G).
//   imu.zero_gyros();
//   delay(100);
//   imu.calibrate_accelerometers();
//   delay(100);

  // If you'd like to extract the health packet to see any errors in the UM7,
//  if(Serial1.available())
//   if(imu.decode(Serial1.read())
//    Serial.println(imu.error);
  
  init = micros();
}

void loop() {
  return;

  // Read data only when it's available from Serial1 on the MCU
//   if (Serial1.available()) {  
    
    // Reads byte from buffer, valid packet returns true and executes if statement.
//     if (imu.decode(Serial1.read())) {
//       Serial.print(micros()-init); Serial.print(", ");
      
      // Print all raw datasets (exluding times)
//      Serial.print(imu.gyro_raw_x); Serial.print(", ");
//      Serial.print(imu.gyro_raw_y); Serial.print(", ");
//      Serial.print(imu.gyro_raw_z); Serial.print(", ");
//      
//      Serial.print(imu.accel_raw_x); Serial.print(", ");
//      Serial.print(imu.accel_raw_y); Serial.print(", ");
//      Serial.print(imu.accel_raw_z); Serial.print(", ");
//      
//      Serial.print(imu.mag_raw_x); Serial.print(", ");
//      Serial.print(imu.mag_raw_y); Serial.print(", ");
//      Serial.print(imu.mag_raw_z); Serial.print(", ");

//      Serial.print(imu.temp); Serial.print(", ");


      // Print all position and velocity datasets (excluding times)
//      Serial.print(imu.north_pos); Serial.print(", ");
//      Serial.print(imu.east_pos); Serial.print(", ");
//      Serial.print(imu.up_pos); Serial.print(", ");
//
//      Serial.print(imu.north_vel); Serial.print(", ");
//      Serial.print(imu.east_vel); Serial.print(", ");
//      Serial.print(imu.up_vel); Serial.print(", ");


      // Print all processed datasets (excluding times)
//       Serial.print(imu.gyro_x); Serial.print(", ");
//       Serial.print(imu.gyro_y); Serial.print(", ");
//       Serial.print(imu.gyro_z); Serial.print(", ");
      
//       Serial.print(imu.accel_x); Serial.print(", ");
//       Serial.print(imu.accel_y); Serial.print(", ");
//       Serial.print(imu.accel_z); Serial.print(", ");
      
//       Serial.print(imu.mag_x); Serial.print(", ");
//       Serial.print(imu.mag_y); Serial.print(", ");
//       Serial.print(imu.mag_z); Serial.print(", ");


      // Print all quaternion datasets (excluding times)
//      Serial.print(imu.quat_a); Serial.print(", ");
//      Serial.print(imu.quat_b); Serial.print(", ");
//      Serial.print(imu.quat_c); Serial.print(", ");
//      Serial.print(imu.quat_d); Serial.print(", ");


      // Print all euler datasets (excluding times)
//       Serial.print(imu.roll); Serial.print(", ");
//       Serial.print(imu.pitch); Serial.print(", ");
//       Serial.print(imu.yaw); Serial.print(", ");
//       Serial.print(imu.roll_rate); Serial.print(", ");
//       Serial.print(imu.pitch_rate); Serial.print(", ");
//       Serial.print(imu.yaw_rate); 
      
//       Serial.print("\n"); // Must have \n on the last dataset for Arduino IDE graph
//      }
//   }
}

void serialEvent1() {
  //  Reads byte from buffer, valid packet returns true and executes if statement.
  if (imu.decode(Serial1.read())) {
    Serial.print(micros()-init); Serial.print(", ");
    Serial.print(imu.roll); Serial.print(", ");
    Serial.print(imu.pitch); Serial.print(", ");

    // Correct negative readings
    if (imu.yaw < 0) {
      Serial.print(360 + imu.yaw); Serial.print(", ");
    } else {
      Serial.print(imu.yaw); Serial.print(", ");
    }

    Serial.print("\n"); // Must have \n on the last dataset for Arduino IDE graph
  }
}
