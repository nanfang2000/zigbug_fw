/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

/* [] END OF FILE */
#include "inv_mpu_cal.h"
#include <stm32f4xx.h>
#include <stdlib.h>
#include "fitting.h"
#include "inv_mpu.h"

#define i2c_write   I2CMaster_write_bus0
#define i2c_read    I2CMaster_read_bus0
#define delay_ms    CyDelay
#define writeRegister(regaddr, byte) {unsigned char tmp = byte; i2c_write(MPU9250_DEFAULT_ADDRESS, regaddr, 1, &tmp);}

#define MAG_CALIBRATE_CNT   (n)
#define MAG_CALIBRATE_THR   (5.f)

#define CALIBRATION_DATA_COUNT  (200)

#if 0
void mpu_calibrate_gyr_acc(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeRegister(MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay_ms(1);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeRegister(MPU9250_PWR_MGMT_1, 0x01);
  writeRegister(MPU9250_PWR_MGMT_2, 0x00);
  delay_ms(1);

// Configure device for bias calculation
  writeRegister(MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
  writeRegister(MPU9250_FIFO_EN, 0x00);      // Disable FIFO
  writeRegister(MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeRegister(MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeRegister(MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeRegister(MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay_ms(1);

// Configure MPU9250 gyro and accelerometer for bias calculation
  writeRegister(MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeRegister(MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeRegister(MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeRegister(MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeRegister(MPU9250_USER_CTRL, 0x40);   // Enable FIFO
  writeRegister(MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  delay_ms(1); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeRegister(MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  i2c_read(MPU9250_DEFAULT_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    i2c_read(MPU9250_DEFAULT_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  i2c_read(MPU9250_DEFAULT_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  i2c_read(MPU9250_DEFAULT_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  i2c_read(MPU9250_DEFAULT_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
#else
void mpu_calibrate_gyr_acc(float * gypo_bias, float * accel_bias)
{
    short accel[3], gypo[3];
    s32 accel_acc[3] = {0, 0, 0}, gypo_acc[3] = {0, 0 ,0};
    u16 gyr_fsr;
    u8 acc_fsr;
    u16 mag_fsr;
    u32 timestamp0, timestamp1;
    s32 i;
    
    mpu_get_gyro_fsr(&gyr_fsr);
    mpu_get_accel_fsr(&acc_fsr);
    mpu_get_compass_fsr(&mag_fsr);
    
    for(i = 0 ; i < CALIBRATION_DATA_COUNT; i++)
    {
        mpu_get_accel_reg(accel, &timestamp0);
        mpu_get_gyro_reg(gypo, &timestamp1);
        accel_acc[0] += accel[0];
        accel_acc[1] += accel[1];
        accel_acc[2] += accel[2];
        gypo_acc[0] += gypo[0];
        gypo_acc[1] += gypo[1];
        gypo_acc[2] += gypo[2];
    }
    
    gypo_bias[0] = (int)gypo_acc[0]*(int)gyr_fsr/(float)CALIBRATION_DATA_COUNT/32768.f;
    gypo_bias[1] = (int)gypo_acc[1]*(int)gyr_fsr/(float)CALIBRATION_DATA_COUNT/32768.f;
    gypo_bias[2] = (int)gypo_acc[2]*(int)gyr_fsr/(float)CALIBRATION_DATA_COUNT/32768.f;
    
    accel_bias[0] = (int)accel_acc[0]*(int)acc_fsr/(float)CALIBRATION_DATA_COUNT/32768.f;
    accel_bias[1] = (int)accel_acc[1]*(int)acc_fsr/(float)CALIBRATION_DATA_COUNT/32768.f;
    accel_bias[2] = (int)accel_acc[2]*(int)acc_fsr/(float)CALIBRATION_DATA_COUNT/32768.f - 1.f;
}
#endif

void mpu_calibrate_mag(float *magOffset)
{
    float magData[MAG_CALIBRATE_CNT][3];
    int32_t validCounter = 0;

    while(validCounter < MAG_CALIBRATE_CNT)
    {
        short magn[3];
        unsigned long timestamp;
        
        mpu_get_compass_reg(magn, &timestamp);
        if(validCounter == 0)
        {
            magData[validCounter][0] = magn[0];
            magData[validCounter][1] = magn[1];
            magData[validCounter][2] = magn[2];
            validCounter++;
        }
        else
        {
            uint8_t found = 0;
            int32_t i;
            for(i = 0; i < validCounter; i++)
            {
                if(abs(magn[0] - magData[i][0]) < MAG_CALIBRATE_THR
                        && abs(magn[1] - magData[i][1]) < MAG_CALIBRATE_THR
                        && abs(magn[2] - magData[i][2]) < MAG_CALIBRATE_THR)
                {
                    found = 1;
                    break;
                }
            }
            if(!found)
            {
                magData[validCounter][0] = magn[0];
                magData[validCounter][1] = magn[1];
                magData[validCounter][2] = magn[2];
                validCounter++;
            }
        }
    }

    SphereFit((float**)magData, magOffset);
}