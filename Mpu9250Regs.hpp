/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of MPU9250-SPI-driver.
 * MPU9250-SPI-driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MPU9250-SPI-driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MPU9250-SPI-driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MPU9250_REGS_H
#define MPU9250_REGS_H

#include <stdint.h>

// Resets values
#define MPU9250_DEFAULT_RESET_VALUE     0x00
#define MPU9250_PWR_MGMT_1_RESET_VALUE  0x01
#define MPU9250_WHO_AM_I_RESET_VALUE    0x71
#define MPU9250_SPI_READ_MASK           0x80   // use with |


// I2C address
#define MPU9250_I2C_ADDR                0x68    // Base + pin SDO input (0 or 1)
#define MPU9250_I2C_MAG_ADDRESS         0x0C

/*********************
 *                   *
 * REGISTERS ADDRESS *
 *                   *
 *********************/

/*
 * Register with _H contains High byte of the value
 * Register with _L contains Low byte of the value
 */

// Gyroscope Self-Test Registers
#define MPU9250_REG_SELF_TEST_X_GYRO    0x00    // RW
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01    // RW
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02    // RW

// Accelerometer Self-Test Registers
#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D    // RW
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E    // RW
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F    // RW

// Gyro Offset Registers
#define MPU9250_REG_XG_OFFSET_H         0x13    // RW
#define MPU9250_REG_XG_OFFSET_L         0x14    // RW
#define MPU9250_REG_YG_OFFSET_H         0x15    // RW
#define MPU9250_REG_YG_OFFSET_L         0x16    // RW
#define MPU9250_REG_ZG_OFFSET_H         0x17    // RW
#define MPU9250_REG_ZG_OFFSET_L         0x18    // RW

// Sample Rate Divider
#define MPU9250_REG_SMPLRT_DIV          0x19    // RW

// Configuration
#define MPU9250_REG_CONFIG              0x1A    // RW

// Gyroscope configuration
#define MPU9250_REG_GYRO_CONFIG         0x1B    // RW

// Accelerometer Configuration
#define MPU9250_REG_ACCEL_CONFIG        0x1C    // RW
#define MPU9250_REG_ACCEL_CONFIG2       0x1D    // RW

// Low Power Accelerometer ODR Control
#define MPU9250_REG_LP_ACCEL_ODR        0x1E    // RW

// Wake-on Motion Threshold
#define MPU9250_REG_WOM_THR             0x1F    // RW

// FIFO Enable
#define MPU9250_REG_FIFO_EN             0x23    // RW

// I2C Master Control
#define MPU9250_REG_I2C_MST_CTRL        0x24    // RW

// I2C Slave 0 Control
#define MPU9250_REG_I2C_SLV0_ADDR       0x25    // RW
#define MPU9250_REG_I2C_SLV0_REG        0x26    // RW
#define MPU9250_REG_I2C_SLV0_CTRL       0x27    // RW

// I2C Slave 1 Control
#define MPU9250_REG_I2C_SLV1_ADDR       0x28    // RW
#define MPU9250_REG_I2C_SLV1_REG        0x29    // RW
#define MPU9250_REG_I2C_SLV1_CTRL       0x2A    // RW

// I2C Slave 2 Control
#define MPU9250_REG_I2C_SLV2_ADDR       0x2B    // RW
#define MPU9250_REG_I2C_SLV2_REG        0x2C    // RW
#define MPU9250_REG_I2C_SLV2_CTRL       0x2D    // RW

// I2C Slave 3 Control
#define MPU9250_REG_I2C_SLV3_ADDR       0x2E    // RW
#define MPU9250_REG_I2C_SLV3_REG        0x2F    // RW
#define MPU9250_REG_I2C_SLV3_CTRL       0x30    // RW

// I2C Slave 4 Control
#define MPU9250_REG_I2C_SLV4_ADDR       0x31    // RW
#define MPU9250_REG_I2C_SLV4_REG        0x32    // RW
#define MPU9250_REG_I2C_SLV4_DO         0x33    // RW  data to be written
#define MPU9250_REG_I2C_SLV4_CTRL       0x34    // RW
#define MPU9250_REG_I2C_SLV4_DI         0x35    // R   data to read

// I2C Master Status
#define MPU9250_REG_I2C_MST_STATUS      0x36    // R

// INT Pin / Bypass Enable Configuration
#define MPU9250_REG_INT_PIN_CFG         0x37    // RW

// Interrupt Enable
#define MPU9250_REG_INT_ENABLE          0x38    // RW

// Interrupt Status
#define MPU9250_REG_INT_STATUS          0x3A    // R

// Accelerometer Measurements
#define MPU9250_REG_ACCEL_XOUT_H        0x3B    // R
#define MPU9250_REG_ACCEL_XOUT_L        0x3C    // R
#define MPU9250_REG_ACCEL_YOUT_H        0x3D    // R
#define MPU9250_REG_ACCEL_YOUT_L        0x3E    // R
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F    // R
#define MPU9250_REG_ACCEL_ZOUT_L        0x40    // R

// Temperature Measurement
#define MPU9250_REG_TEMP_OUT_H          0x41    // R
#define MPU9250_REG_TEMP_OUT_L          0x42    // R

// Gyroscope Measurements
#define MPU9250_REG_GYRO_XOUT_H         0x43    // R
#define MPU9250_REG_GYRO_XOUT_L         0x44    // R
#define MPU9250_REG_GYRO_YOUT_H         0x45    // R
#define MPU9250_REG_GYRO_YOUT_L         0x46    // R
#define MPU9250_REG_GYRO_ZOUT_H         0x47    // R
#define MPU9250_REG_GYRO_ZOUT_L         0x48    // R

// External Sensor Data
#define MPU9250_REG_EXT_SENS_DATA_00    0x49    // R
#define MPU9250_REG_EXT_SENS_DATA_01    0x4A    // R
#define MPU9250_REG_EXT_SENS_DATA_02    0x4B    // R
#define MPU9250_REG_EXT_SENS_DATA_03    0x4C    // R
#define MPU9250_REG_EXT_SENS_DATA_04    0x4D    // R
#define MPU9250_REG_EXT_SENS_DATA_05    0x4E    // R
#define MPU9250_REG_EXT_SENS_DATA_06    0x4F    // R
#define MPU9250_REG_EXT_SENS_DATA_07    0x50    // R
#define MPU9250_REG_EXT_SENS_DATA_08    0x51    // R
#define MPU9250_REG_EXT_SENS_DATA_09    0x52    // R
#define MPU9250_REG_EXT_SENS_DATA_10    0x53    // R
#define MPU9250_REG_EXT_SENS_DATA_11    0x54    // R
#define MPU9250_REG_EXT_SENS_DATA_12    0x55    // R
#define MPU9250_REG_EXT_SENS_DATA_13    0x56    // R
#define MPU9250_REG_EXT_SENS_DATA_14    0x57    // R
#define MPU9250_REG_EXT_SENS_DATA_15    0x58    // R
#define MPU9250_REG_EXT_SENS_DATA_16    0x59    // R
#define MPU9250_REG_EXT_SENS_DATA_17    0x5A    // R
#define MPU9250_REG_EXT_SENS_DATA_18    0x5B    // R
#define MPU9250_REG_EXT_SENS_DATA_19    0x5C    // R
#define MPU9250_REG_EXT_SENS_DATA_20    0x5D    // R
#define MPU9250_REG_EXT_SENS_DATA_21    0x5E    // R
#define MPU9250_REG_EXT_SENS_DATA_22    0x5F    // R
#define MPU9250_REG_EXT_SENS_DATA_23    0x60    // R

// I2C Slave 0 Data Out
#define MPU9250_REG_I2C_SLV0_DO         0x63    // RW

// I2C Slave 1 Data Out
#define MPU9250_REG_I2C_SLV1_DO         0x64    // RW

// I2C Slave 2 Data Out
#define MPU9250_REG_I2C_SLV2_DO         0x65    // RW

// I2C Slave 3 Data Out
#define MPU9250_REG_I2C_SLV3_DO         0x66    // RW

// I2C Master Delay Control
#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67    // RW

// Signal Path Reset
#define MPU9250_REG_SIGNAL_PATH_RESET   0x68    // RW

// Accelerometer Interrupt Control
#define MPU9250_REG_MOT_DETECT_CTRL     0x69    // RW

// User Control
#define MPU9250_REG_USER_CTRL           0x6A    // RW

// Power Management
#define MPU9250_REG_PWR_MGMT_1          0x6B    // RW
#define MPU9250_REG_PWR_MGMT_2          0x6C    // RW

// FIFO Count Registers
#define MPU9250_REG_FIFO_COUNTH         0x72    // RW
#define MPU9250_REG_FIFO_COUNTL         0x73    // RW

// FIFO Read Write
#define MPU9250_REG_FIFO_R_W            0x74    // RW

// Who Am I
#define MPU9250_REG_WHO_AM_I            0x75    // R

// Accelerometer Offset Registers
#define MPU9250_REG_XA_OFFSET_H         0x77    // RW
#define MPU9250_REG_XA_OFFSET_L         0x78    // RW
#define MPU9250_REG_YA_OFFSET_H         0x7A    // RW
#define MPU9250_REG_YA_OFFSET_L         0x7B    // RW
#define MPU9250_REG_ZA_OFFSET_H         0x7D    // RW
#define MPU9250_REG_ZA_OFFSET_L         0x7E    // RW


/*****************************
 *                           *
 * Magnetometer register map *
 *                           *
 * ***************************/

// Device ID (Who Am I)
#define MPU9250_MAG_REG_WIA             0x00    // R
// Information for AKM
#define MPU9250_MAG_REG_INFO            0x01    // R
// Status 1
#define MPU9250_MAG_REG_ST1             0x02    // R
// Measurement Data
#define MPU9250_MAG_REG_HXL             0x03    // R
#define MPU9250_MAG_REG_HXH             0x04    // R
#define MPU9250_MAG_REG_HYL             0x05    // R
#define MPU9250_MAG_REG_HYH             0x06    // R
#define MPU9250_MAG_REG_HZL             0x07    // R
#define MPU9250_MAG_REG_HZH             0x08    // R
// Status 2
#define MPU9250_MAG_REG_ST2             0x09    // R
// Control 1
#define MPU9250_MAG_REG_CNTL            0x0A    // RW
// Self test control
#define MPU9250_MAG_REG_ASTC            0x0C    // RW
// Disable I2C
#define MPU9250_MAG_REG_I2CDIS          0x0F    // RW
// Sensitivity adjustment values
// Hadj = H * ((((ASA - 128)*0.5)/128) + 1)
#define MPU9250_MAG_REG_ASAX            0x10    // R
#define MPU9250_MAG_REG_ASAY            0x11    // R
#define MPU9250_MAG_REG_ASAZ            0x12    // R

#define MPU9250_MAG_WIA_VALUE           0x48    // Use for connection test
// Value to set at MPU9250_MAG_REG_I2CDIS to disable I2C
#define MPU9250_MAG_I2C_DISABLE_VALUE   0x1B

/** \brief  name space for common tools to work with MPU9250
 */
namespace mpu9250 {
    /*
     * Struct for conveniance
     */

    /** \brief Conversion between two uint8_t and one uint16_t
    */
    union TwoByte{
        struct {
            uint8_t lsb;    /**< Least significant Byte */
            uint8_t msb;    /**< Most significant Byte */
        };
        uint16_t value;     /**< uint16_t value */
    };

/*****************************
 *                           *
 *    MPU9250 enumerations   *
 *                           *
 * ***************************/

    /*
     * Enumerations of possible values
     */

    /** 
     * \brief Accelerometer low power mode
     * 
     * select the frequency of waking up the chip to take a sample of accel data
     */
    enum AccelLowPowerOutFreq {
        ACCEL_LOW_POWER_OUT_FREQ_0_24HZ     = 0x0,  /**< 0,24Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_0_49HZ     = 0x1,  /**< 0,49Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_0_98HZ     = 0x2,  /**< 0,98Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_1_95HZ     = 0x3,  /**< 1,95Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_3_91HZ     = 0x4,  /**< 3,91Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_7_81HZ     = 0x5,  /**< 7,81Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_15_63HZ    = 0x6,  /**< 15,63Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_31_25HZ    = 0x7,  /**< 31,25Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_62_5HZ     = 0x8,  /**< 62,5Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_125HZ      = 0x9,  /**< 125Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_250HZ      = 0xA,  /**< 250Hz wake up frequency */
        ACCEL_LOW_POWER_OUT_FREQ_500HZ      = 0xB   /**< 500Hz wake up frequency */
    };


    /** 
     * \brief Options for gyroscope
     * \see enum Gyro_Fchoice_b
     */
    enum DLPFGyro {
        DLPF_GYRO_250HZ     = 0x0,  /**< Bandwith : 250Hz; Delay : 0.97ms; Sampling rate : 8kHz */
        DLPF_GYRO_184HZ     = 0x1,  /**< Bandwith : 184Hz; Delay : 2.9ms; Sampling rate : 1kHz */
        DLPF_GYRO_92HZ      = 0x2,  /**< Bandwith : 92Hz; Delay : 3.9ms; Sampling rate : 1kHz */
        DLPF_GYRO_41HZ      = 0x3,  /**< Bandwith : 41Hz; Delay : 5.9ms; Sampling rate : 1kHz */
        DLPF_GYRO_20HZ      = 0x4,  /**< Bandwith : 20Hz; Delay : 9.9ms; Sampling rate : 1kHz */
        DLPF_GYRO_10HZ      = 0x5,  /**< Bandwith : 10Hz; Delay : 17.85ms; Sampling rate : 1kHz */
        DLPF_GYRO_05HZ      = 0x6,  /**< Bandwith : 5Hz; Delay : 33.48ms; Sampling rate : 1kHz */
        DLPF_GYRO_3600HZ    = 0x7   /**< Bandwith : 3600Hz; Delay : 0.17ms; Sampling rate : 8kHz */
    };

    /**
     * \brief Digital Low Pass Filter Options for accelerometer
     */
    enum DLPFAccel {
        DLPF_ACCEL_460HZ    = 0x0,  /**< Bandwith : 460Hz; Delay : 1.94ms; Sampling rate : 1kHz */
        DLPF_ACCEL_184HZ    = 0x1,  /**< Bandwith : 184Hz; Delay : 5.80ms; Sampling rate : 1kHz */
        DLPF_ACCEL_92HZ     = 0x2,  /**< Bandwith : 92Hz; Delay : 7.80ms; Sampling rate : 1kHz */
        DLPF_ACCEL_41HZ     = 0x3,  /**< Bandwith : 41Hz; Delay : 11.80ms; Sampling rate : 1kHz */
        DLPF_ACCEL_20HZ     = 0x4,  /**< Bandwith : 20Hz; Delay : 19.80ms; Sampling rate : 1kHz */
        DLPF_ACCEL_10HZ     = 0x5,  /**< Bandwith : 10Hz; Delay : 35.70ms; Sampling rate : 1kHz */
        DLPF_ACCEL_05HZ     = 0x6   /**< Bandwith : 5Hz; Delay : 66.96ms; Sampling rate : 1kHz */
    };


    /** 
     * \brief Gyroscope Digital Low Pass Filter activation
     * \see enum DLPFGyro
     */
    enum Gyro_Fchoice_b {
        FCHOICE_DLPF_ENABLE         = 0x0,  /**< Enable DLPF */
        FCHOICE_DLPF_DISABLE_SLOW   = 0x2,  /**< Disable DLPF; Bandwith : 3600Hz; Delay : 0.11ms; Sampling rate : 32kHz */
        FCHOICE_DLPF_DISABLE_FAST   = 0x3   /**< Disable DLPF; Bandwith : 8800Hz; Delay : 0.064ms; Sampling rate : 32kHz */
    };

    /** 
     *  \brief Fsync will be latched to capture short strobes.
     */
    enum Fsync{
        FSYNC_DISABLED      = 0x0,
        FSYNC_TEMP_OUT_L    = 0x1,
        FSYNC_GYRO_XOUT_L   = 0x2,
        FSYNC_GYRO_YOUT_L   = 0x3,
        FSYNC_GYRO_ZOUT_L   = 0x4,
        FSYNC_ACCEL_XOUT_L  = 0x5,
        FSYNC_ACCEL_YOUT_L  = 0x6,
        FSYNC_ACCEL_ZOUT_L  = 0x7
    };


    /** 
     * \brief Gyroscope full scale values
     */
    enum GyroFullScale {
        GYRO_FULL_SCALE_250DPS   = 0x0, /**< +/- 250 °/s */
        GYRO_FULL_SCALE_500DPS   = 0x1, /**< +/- 500 °/s */
        GYRO_FULL_SCALE_1000DPS  = 0x2, /**< +/- 1000 °/s */
        GYRO_FULL_SCALE_2000DPS  = 0x3  /**< +/- 2000 °/s */
    };

    /** 
     * \brief Accelerometer full scale values
     */
    enum AccelFullScale {
        ACCEL_FULL_SCALE_2G      = 0x0, /**< +/- 2G */
        ACCEL_FULL_SCALE_4G      = 0x1, /**< +/- 4G */
        ACCEL_FULL_SCALE_8G      = 0x2, /**< +/- 8G */
        ACCEL_FULL_SCALE_16G     = 0x3  /**< +/- 16G */
    };

    /** 
     * \brief I2C master clock values
     */
    enum I2cMstClk {
        I2C_MST_CLK_348KHZ     = 0x0,   /**< I2C clock : 348kHz */
        I2C_MST_CLK_333KHZ     = 0x1,   /**< I2C clock : 333kHz */
        I2C_MST_CLK_320KHZ     = 0x2,   /**< I2C clock : 320kHz */
        I2C_MST_CLK_308KHZ     = 0x3,   /**< I2C clock : 308kHz */
        I2C_MST_CLK_296KHZ     = 0x4,   /**< I2C clock : 296kHz */
        I2C_MST_CLK_286KHZ     = 0x5,   /**< I2C clock : 286kHz */
        I2C_MST_CLK_276KHZ     = 0x6,   /**< I2C clock : 276kHz */
        I2C_MST_CLK_267KHZ     = 0x7,   /**< I2C clock : 267kHz */
        I2C_MST_CLK_258KHZ     = 0x8,   /**< I2C clock : 258kHz */
        I2C_MST_CLK_500KHZ     = 0x9,   /**< I2C clock : 500kHz */
        I2C_MST_CLK_471KHZ     = 0xA,   /**< I2C clock : 471kHz */
        I2C_MST_CLK_444KHZ     = 0xB,   /**< I2C clock : 444kHz */
        I2C_MST_CLK_421KHZ     = 0xC,   /**< I2C clock : 421kHz */
        I2C_MST_CLK_400KHZ     = 0xD,   /**< I2C clock : 400kHz */
        I2C_MST_CLK_381KHZ     = 0xE,   /**< I2C clock : 381kHz */
        I2C_MST_CLK_364KHZ     = 0xF    /**< I2C clock : 364kHz */
    };

    /** 
     * Clock source for the chip values
     */
    enum ClockSource {
        CLOCK_SOURCE_INTERNAL   = 0x0,  /**< Internal 20MHz oscillator */
        CLOCK_SOURCE_AUTO       = 0x1,  /**< Auto selects the best available clock source – PLL if ready, else use the Internal oscillator*/
        CLOCK_SOURCE_STOP       = 0x7   /**< Stops the clock and keeps timing generator in reset */
    };


/*****************************
 *                           *
 * MPU9250 registers struct  *
 *                           *
 * ***************************/

    /*
     * General Configuration
     */

    /** 
     * \brief Configuration register
     */
    union Config_reg {
        /** 
         * \brief Configuration register
         */
        struct {
            enum DLPFGyro dlpf_cfg : 3;    /**< Configure DLPF if Gyro_config_reg.fchoice_b = FCHOICE_DLPF_ENABLE; See enum DLPFGyro */
            enum Fsync ext_sync_set : 3;   /**< Flatch sync. See enum Fsync */
            bool fifo_mod : 1;             /**< When set to ‘1’, when the fifo is full, additional writes will not be written to fifo */
            bool : 1;
        } __attribute__((packed));
        uint8_t value;
    };


    /*
     * Sensors configuration
     */

    /** \brief Gyroscope configuration register
    */
    union Gyro_config_reg {
        struct {
            enum Gyro_Fchoice_b fchoice_b : 2;     /**< Use to bypass DLPF. See enum Fchoice_b */
            bool : 1;
            enum GyroFullScale gyro_fs_sel : 2;    /**< Set the full scale of the gyroscope. See enum GyroFullScale */
            bool zgyro_cten : 1;                   /**< Z gyro self test */
            bool ygyro_cten : 1;                   /**< Y gyro self test */
            bool xgyro_cten : 1;                   /**< Z gyro self test */
        } __attribute__((packed));
        uint8_t value;
    };

    /** \brief Accelerometer configuration register
    */
    union Accel_config_reg {
        struct {
            unsigned char : 3;
            enum AccelFullScale accel_fs_sel : 2;  /**< Set full scale of the accelerometer. See enum AccelFullScale */
            bool az_st_en : 1;                     /**< Z accel self-test */
            bool ay_st_en : 1;                     /**< Y accel self-test */
            bool ax_st_en : 1;                     /**< X accel self-test */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Accelerometer configuration 2 register
    */
    union Accel_config_reg2 {
        struct {
            enum DLPFAccel dlpf : 3;           /**< Accelerometer low pass filter setting. See enum DLPFAccel */
            bool disable_dlpf : 1;             /**<  true -> disable DLPF and have a bandwith of 1.13KHz */
            unsigned char : 4;
        }__attribute__((packed));
        uint8_t value;
    };


    /*
     * I2C configuration / Status
     */

    /** \brief I2C Master control register
    */
    union I2c_master_control_reg {
        struct {
            enum I2cMstClk i2c_mst_clk : 4;    /**< Configure the divider on the MPU9250 internal 8MHz clock. See enum I2cMstClk */
            bool i2c_mst_p_nsr : 1;            /**< This bit controls the I2C Master’s transition from one slave read to the next slave read. If 0, there is a restart between reads. If 1, there is a stop between reads. */
            bool slv_3_fifo_en : 1;            /**< true : write EXT_SENS_DATA registers associated to SLV_3 to the FIFO at the sample rate; */
            bool wait_for_es : 1;              /**< Delays the data ready interrupt until external sensor data is loaded. If I2C_MST_IF is disabled, the interrupt will still occur */
            bool mult_mst_en : 1;              /**< Enables multi-master capability. When disabled, clocking to the I2C_MST_IF can be disabled when not in use and the logic to detect lost arbitration is disabled. */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief I2C Master Delay Control register
    */
    union I2c_master_delay_control_reg {
        struct {
            /*
             * When enabled, slave [0-4] will only be accessed (1+I2C_MST_DLY) samples
             * as determined by SMPLRT_DIV and DLPF_CFG
             */
            bool i2c_slv0_dly_en : 1;  /**<  When enabled, slave 0 will only be accessed (1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG */
            bool i2c_slv1_dly_en : 1;  /**<  When enabled, slave 1 will only be accessed (1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG */
            bool i2c_slv2_dly_en : 1;  /**<  When enabled, slave 2 will only be accessed (1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG */
            bool i2c_slv3_dly_en : 1;  /**<  When enabled, slave 3 will only be accessed (1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG */
            bool i2c_slv4_dly_en : 1;  /**<  When enabled, slave 4 will only be accessed (1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG */
            bool :2;
            bool delay_es_shadow : 1;  /**< Delays shadowing of external sensor data until all data is received */
        }__attribute__((packed));
        uint8_t value;
    };


    /** \brief I2C Slave 0 to 4 address registers
    */
    union I2c_slv0_4_addr {
        struct {
            unsigned char i2c_id : 7;  /**< Physical address of I2C slave */
            bool i2c_slave_read : 1;   /**< true : transfert is read; false : transfert is write */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief I2C Slave 0 to 3 Control registers
    */
    union I2c_slv0_3_ctrl_reg {
        struct {
            unsigned char i2c_slv_leng : 4;    /**< Number of bytes to be read from I2C slave */
            bool i2c_slv_grp : 1;              /** External sensor data typically comes in as groups of two bytes. This bit is used to determine if the groups are from the slave’s register address 0 and 1, 2 and 3, etc.., or if the groups are address 1 and 2, 3 and 4. 0 indicates slave register addresses 0 and 1 are grouped together (odd numbered register ends the group). 1 indicates slave register addresses 1 and 2 are grouped together (even numbered register ends the group). This allows byte swapping of registers that are grouped starting at any address. */
            bool i2c_slv_reg_dis : 1;          /**< When set, the transaction does not write a register value, it will only read data, or write data */
            bool i2c_slv_byte_sw : 1;          /**< Swap bytes when reading both the low and high byte of a word.*/
            bool i2c_slv_en : 1;               /**< Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA*/
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief I2C Slave 4 Control register
    */
    union I2c_slv4_ctrl_reg {
        struct {
            unsigned char i2c_mst_dly : 5;     /**< When enabled via the I2C_MST_DELAY_CTRL, those slaves will only be enabled every (1+I2C_MST_DLY) samples */
            bool i2c_slv4_reg_dis : 1;         /**< When set, the transaction does not write a register value, it will only read data, or write data */
            bool slv4_done_int_en : 1;         /**< Enables the completion of the I2C slave 4 data transfer to cause an interrupt */
            bool i2c_slv4_en : 1;              /**< Enable data transfer with this slave at the sample rate. */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief I2C Master Status
    *
    * A read of this register clears all status bits in this register.
    * Bit asserted will cause an interrupt if bit I2C_MST_INT_EN in the INT_ENABLE register is asserted.
    */
    union I2c_master_status_reg {
        struct {
            bool i2c_slv0_nack : 1;    /**< Asserted when slave 0 receives a nack */
            bool i2c_slv1_nack : 1;    /**< Asserted when slave 1 receives a nack */
            bool i2c_slv2_nack : 1;    /**< Asserted when slave 2 receives a nack */
            bool i2c_slv3_nack : 1;    /**< Asserted when slave 3 receives a nack */
            bool i2c_slv4_nack : 1;    /**< Asserted when slave 4 receives a nack; need also SLV4_DONE_INT_EN bit is asserted in the I2C_SLV4_CTRL register to cause an interrupt. */
            bool i2c_lost_arb : 1;     /**< Asserted when I2C slave looses arbitration of the I2C bus */
            bool i2c_slv4_done : 1;    /**< Asserted when I2C slave 4’s transfer is complete */
            bool pass_through : 1;     /**< Status of FSYNC interrupt – used as a way to pass an external interrupt through this chip to the host. */
        }__attribute__((packed));
        uint8_t value;
    };


    /*
     * Interrupt configuration / Status
     */

    /** \brief Interrupt enable register
    */
    union Interrupt_enable_reg {
        struct {
            bool raw_rdy_en : 1;       /**< 1 – Enable interrupt for wake on motion to propagate to interrupt pin. */
            bool : 2;
            bool fsync_int_en : 1;     /**< 1 – Enable interrupt for fifo overflow to propagate to interrupt pin. */
            bool fifo_overflow_en : 1; /**< 1 – Enable Fsync interrupt to propagate to interrupt pin. */
            bool : 1;
            bool wom_en : 1;           /**< 1 – Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES. */
            bool :1;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Accelerometer Interrupt Control register
    */
    union Accel_interrupt_control_reg {
        struct {
            bool : 6;
            bool accel_intel_mode : 1;   /**< This bit defines : 1 = Compare the current sample with the previous sample. 0 = Not used. */
            bool accel_intel_en : 1;     /**< This bit enables the Wake-on-Motion detection logic. */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief INT Pin / Bypass Enable Configuration register
    */
    union Int_pin_bypass_enable_conf_reg {
        struct {
            bool : 1;
            bool bypass_en : 1;        /**< When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c master interface is disabled. The pins will float high due to the internal pull-up if not enabled and the i2c master interface is disabled. */
            bool fsync_int_mode_en : 1; /**< 1 – This enables the FSYNC pin to be used as an interrupt. A transition to the active level described by the ACTL_FSYNC bit will cause an interrupt. The status of the interrupt is read in the I2C Master Status register PASS_THROUGH bit. */
            bool actl_fsync : 1;       /**< 1 – The logic level for the FSYNC pin as an interrupt is active low. 0 – The logic level for the FSYNC pin as an interrupt is active high. */
            bool int_anyrd_2clear : 1; /**< 1 – Interrupt status is cleared if any read operation is performed. 0 – Interrupt status is cleared only by reading INT_STATUS register */
            bool latch_int_en : 1;     /**< 1 – INT pin level held until interrupt status is cleared. 0 – INT pin indicates interrupt pulse’s is width 50us. */
            bool open : 1;             /**< 1 – INT pin is configured as open drain. 0 – INT pin is configured as push-pull. */
            bool actl : 1;             /**< 1 – The logic level for INT pin is active low. 0 – The logic level for INT pin is active high. */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Interrupt Status register
    */
    union Interrupt_status_reg {
        struct {
            bool raw_data_rdy_int : 1; /**< 1 – Sensor Register Raw Data sensors are updated and Ready to be read. The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES. */
            bool : 2;
            bool fsync_int : 1;        /**< 1 – Fsync interrupt occurred. */
            bool fifo_overflow_int : 1;/**< 1 – Fifo Overflow interrupt occurred. Note that the oldest data is has been dropped from the fifo */
            bool : 1;
            bool wom_int : 1;          /**< 1 – Wake on motion interrupt occurred. */
            bool : 1;
        }__attribute__((packed));
        uint8_t value;
    };


    /*
     * Reset & FIFO configuration
     */

    /** \brief Signal Path Reset register
    */
    union Signal_path_reset_reg {
        struct {
            bool temp_rst : 1;      /**< Reset temp digital signal path */
            bool accel_rst : 1;     /**< Reset accel digital signal path */
            bool gyro_rst : 1;      /**< Reset gyro digital signal path */
            bool : 5;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief User Control register
    */
    union User_control_reg {
        struct {
            bool sig_cond_rst : 1;  /**< 1 – Reset all gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide. */
            bool i2c_mst_rst : 1;   /**< 1 – Reset I2C Master module. Reset is asynchronous. This bit auto clears after one clock cycle. */
            bool fifo_rst : 1;      /**< 1 – Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle. */
            bool : 1;
            bool i2c_if_dis : 1;    /**< 1 – Reset I2C Slave module and put the serial interface in SPI mode only. This bit auto clears after one clock cycle. */
            bool i2c_mst_en : 1;    /**< 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK. 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically driven by pins SDA/SDI and SCL/ SCLK */
            bool fifo_en : 1;       /**< 1 – Enable FIFO operation mode. */
            bool : 1;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief FIFO Enable register
    */
    union Fifo_enable_reg {
        struct {
            bool slv_0 : 1;         /**< write EXT_SENS_DATA registers associated to SLV_0 to the FIFO at the sample rate */
            bool slv_1 : 1;         /**< write EXT_SENS_DATA registers associated to SLV_1 to the FIFO at the sample rate */
            bool slv_2 : 1;         /**< write EXT_SENS_DATA registers associated to SLV_2 to the FIFO at the sample rate */
            bool accel : 1;         /**< write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at the sample rate; */
            bool gyro_zout : 1;     /**< Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby. */
            bool gyro_yout : 1;     /**< Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby. */
            bool gyro_xout : 1;     /**< Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby. */
            bool temp_out : 1;      /**< Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby. */
        }__attribute__((packed));
        uint8_t value;
    };



    /*
     * Power Management
     */

    /** \brief Low Power Accelerometer ODR Control register
    */
    union Accel_low_power_odr_control_reg {
        struct {
            enum AccelLowPowerOutFreq lposc_clksel : 4;    /**< Set the frequency of waking up the chip to take a sample of data. */
            unsigned char :4;
        } __attribute((packed));
        uint8_t value;
    };

    /** \brief Power Management 1 register
    */
    union Power_management1_reg {
        struct {
            enum ClockSource clksel : 3;   /**< Clock source selection; see enum ClockSource */
            bool pd_ptat : 1;              /**< Power down internal PTAT voltage generator and PTAT ADC */
            bool gyro_standby : 1;         /**< When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros. */
            bool cycle : 1;                /**< When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register */
            bool sleep : 1;                /**< When set, the chip is set to sleep mode */
            bool h_reset : 1;              /**< 1 – Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear. */
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Power Management 2 register
    */
    union Power_management2_reg {
        struct {
            bool disable_zg : 1;   /**< 1 – Z gyro is disabled */
            bool disable_yg : 1;   /**< 1 – Y gyro is disabled */
            bool disable_xg : 1;   /**< 1 – X gyro is disabled */
            bool disable_za : 1;   /**< 1 – Z accelerometer is disabled */
            bool disable_ya : 1;   /**< 1 – Y accelerometer is disabled */
            bool disable_xa : 1;   /**< 1 – X accelerometer is disabled */
            bool : 2;
        }__attribute__((packed));
        uint8_t value;
    };

    /*
     * The MPU-9250 can be put into Accelerometer Only Low Power Mode using the following steps:
     * (i)   Set CYCLE bit to 1
     * (ii)  Set SLEEP bit to 0
     * (iii) Set TEMP_DIS bit to 1
     * (iv)  Set DIS_XG, DIS_YG, DIS_ZG bits to 1
     * The bits mentioned in the steps (i) to (iii) can be found in Power Management 1 register (Register
     * 107).
     * In this mode, the device will power off all devices except for the primary I2C interface, waking only
     * the accelerometer at fixed intervals to take a single measurement.
     */


    //
    /** \brief FIFO Count Register MSB register
    */
    union Fifo_countH_reg {
        struct {
            unsigned char fifo_cnt : 5;     /**< High Bits, count indicates the number of written bytes in the FIFO. Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL. */
            unsigned char : 3;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief X/Y/Z accelerometer offset LSB register
    */
    union XYZa_offsL_reg {
        struct {
            unsigned char : 1;
            unsigned char xyza_offs : 7;    /**< Lower bits of the XYZ accelerometer offset cancellation. +/- 16g Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps */
        }__attribute__((packed));
        uint8_t value;
    };


/*****************************
 *                           *
 * Magnetometer enumerations *
 *                           *
 * ***************************/

    /** \brief Operation mode of the magnetometer
    */
    enum MagOperationMode {
        MAG_POWER_DOWN_MODE                     = 0x0,
        MAG_SINGLE_MEASUREMENT_MODE             = 0x1,
        MAG_CONTINUOUS_MEASUREMENT_8Hz          = 0x2,
        MAG_CONTINUOUS_MEASUREMENT_100Hz        = 0x6,
        MAG_EXTERNEL_TRIGGER_MEASUREMENT_MODE   = 0x4,
        MAG_SELF_TEST_MODE                      = 0x8,
        MAG_FUSE_ROM_ACCESS_MODE                = 0xF
    };

/*********************************
 *                               *
 * Magnetometer registers struct *
 *                               *
 * *******************************/

    /** \brief Status 1 register
    */
    union Mag_Status1_reg {
        struct {
            bool drdy : 1;      /**< data ready */
            bool dor : 1;       /**< data overrun */
            bool : 6;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Status 2 register
    */
    union Mag_status2_reg {
        struct {
            bool : 3;
            bool hofl : 1;      /**< Magnetic sensor overflow */
            bool bitm : 1;      /**< mirror of cntl1 bit */
            bool : 3;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief Control 1 register
    */
    union Mag_cntl1_reg {
        struct {
            enum MagOperationMode mode : 4; /**< Operation mode of the magnetometer */
            bool bitm : 1;                  /**< Output bit setting : 0 = 14 bit output; 1 = 16 bit output */
            bool : 3;
        }__attribute__((packed));
        uint8_t value;
    };

    /** \brief self test control register
    */
    union Mag_astc_reg {
        struct {
            bool : 6;
            bool self : 1;      /**< self test control; 1 : generate magnetic field for self test */
            bool : 1;
        }__attribute__((packed));
        uint8_t value;
    };



} //namespace mpu9250

#endif //MPU9250_REGS_H
