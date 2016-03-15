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

#ifndef MPU9250_H
#define MPU9250_H

#include "Mpu9250Regs.hpp"
#include <math.h>

namespace mpu9250 {

    /** \brief Slave selector
     */
    enum Slave {
        SLAVE_0,
        SLAVE_1,
        SLAVE_2,
        SLAVE_3,
        SLAVE_4
    };


    /** \brief 3D vector
     */
    struct Int16Vect3D {
        int16_t x;  /**< x axis of the vector */
        int16_t y;  /**< y axis of the vector */
        int16_t z;  /**< z axis of the vector */
    };


    /** \brief FIFO Count Registers
    */
    union Fifo_count {
        struct {
            uint16_t fifo_cnt : 13; /**< 13 bit unsigned int count of FIFO element. Maximale value is 512 */
            uint16_t : 3;
        }__attribute__((packed));
        uint16_t value;             /**< registers as uint16_t value */
    };

    /** \brief Accelerometer offset registers
    * This can be use for X or Y or Z axis
    */
    union XYZa_offs {
        struct {
            int16_t : 1;
            int16_t xyza_offs : 15; /**< 15 bits signed integer offset value.
                                         Steps of 0.98-mg */
        }__attribute__((packed));
        uint16_t value;      /**< registers as uint16_t value */
    };

    class Mpu9250 {
        public :

            //Sensors

            void setAccelDLPF(bool enable = true, enum DLPFAccel dlpfAccel = DLPF_ACCEL_41HZ);
            void setAccelFullScale(enum AccelFullScale fscale = ACCEL_FULL_SCALE_2G);
            void setAccelInterrupt(bool enableWakeOnMotion = false, bool compareSampleWithPrevious = false);
            void setAccelOnlyLowPower(bool enable = false, enum AccelLowPowerOutFreq wakeFreq = ACCEL_LOW_POWER_OUT_FREQ_0_24HZ);
            void setGyroDLPF(enum Gyro_Fchoice_b state = FCHOICE_DLPF_ENABLE, enum DLPFGyro dlpfGyro = DLPF_GYRO_41HZ);
            void setGyroFullScale(enum GyroFullScale fscale = GYRO_FULL_SCALE_250DPS);
            void setEnabledSensor(bool gyroX = true, bool gyroY = true, bool gyroZ = true, bool accelX = true, bool accelY = true, bool accelZ = true, bool temp = true);
            void setMagnetometer(enum MagOperationMode mode = MAG_CONTINUOUS_MEASUREMENT_100Hz, bool output16bit = true);
            
            //General

            void setClockSource(enum ClockSource clksel = CLOCK_SOURCE_AUTO);
            void setFifo(bool enable = false, bool overwrite = false, bool accel = false, bool gyroX = false, bool gyroY = false, bool gyroZ = false, bool temp = false, bool mag = false, bool slv1 = false, bool slv2 = false, bool slv3 = false);
            void setFsync(enum Fsync fsync = FSYNC_DISABLED);
            void setSampleRateDivider(uint8_t srd = 0x00);
            void setSleep(bool sleep = false);
            void setWakeOnMotionThreshold(uint8_t threshold);

            // Interrupt
            void setInterrupt(bool fsyncAsInterrupt = false, bool fsyncIntLow = false, bool clearOnAnyRead = false, bool holdInt = true, bool openDrain = false, bool logicLevelLow = false);
            void setInterruptPin(bool wakeOnMotion = false, bool fsync = false, bool fifoOverflow = false, bool rawDataAvailable = false);
            void setSlave4Interrupt(bool enableIntOnComplete);

            // I2C

            bool setSlaveConf(enum Slave num, unsigned char addr, bool enable, bool delayEs = false);
            bool setSlaveTransfer(enum Slave num, unsigned char regAddr, uint8_t dataOut, bool isRead, unsigned char nByte, bool dontWriteRegAddr, bool swapByte, bool groupOddEnd);
            void setI2cMaster(enum I2cMstClk clock = I2C_MST_CLK_400KHZ, bool stopBtwRead = false, bool delayDataRdyInt = false, bool multMasterCapa = false, bool delayESShadow = false);
            void setSlave4MstDelay(unsigned char delay);

            //Status

            bool isDataReady();
            bool isFifoOverflow();
            bool isFsyncInt();
            bool isI2cLostArbitration();
            bool isI2cSlave4TransfertComplete();
            bool isSlaveNack(unsigned short slave);
            bool isPassThroughFsyncInt();
            bool isWakeOnMotionInt();
            // Magnetometer Status
            bool isMagDataReady();
            bool isMagDataOverrun();
            bool isMagOverflow();
            bool isMag16BitOutput();

            // Getters
            float getAccelRes();
            float getGyroRes();




        protected :
            /** I2C address of the MPU9250 */
            enum {
                ADDR_SDO_LOW = MPU9250_I2C_ADDR + 0x0,
                ADDR_SDO_HIGH = MPU9250_I2C_ADDR + 0x1
            } mpuI2cAddr;

            /** \brief Configuration of the MPU9250
             *
             * This structure is mapped in the register order
             * It lets burst read and write
             */
            struct {
                int16_t gyroXOffset;        /**< Gyroscope X axis offset */
                int16_t gyroYOffset;        /**< Gyroscope Y axis offset */
                int16_t gyroZOffset;        /**< Gyroscope Z axis offset */
                uint8_t sampleRateDivider;  /**< SAMPLE_RATE= Internal_Sample_Rate / (1 + sampleRateDivider)*/
                union Config_reg config;
                union Gyro_config_reg gyroConfig;
                union Accel_config_reg accelConfig;
                union Accel_config_reg2 accelConfig2;
                union Accel_low_power_odr_control_reg accelLowPower;
                uint8_t wakeOnMotionThreshold;
                /* * Registry break * */
                union Fifo_enable_reg fifoEnabled;
                union I2c_master_control_reg i2cMasterCtrl;
                union I2c_slv0_4_addr i2cSlave0Addr;
                uint8_t i2cSlave0Reg;
                union I2c_slv0_3_ctrl_reg i2cSlave0Ctrl;
                union I2c_slv0_4_addr i2cSlave1Addr;
                uint8_t i2cSlave1Reg;
                union I2c_slv0_3_ctrl_reg i2cSlave1Ctrl;
                union I2c_slv0_4_addr i2cSlave2Addr;
                uint8_t i2cSlave2Reg;
                union I2c_slv0_3_ctrl_reg i2cSlave2Ctrl;
                union I2c_slv0_4_addr i2cSlave3Addr;
                uint8_t i2cSlave3Reg;
                union I2c_slv0_3_ctrl_reg i2cSlave3Ctrl;
                union I2c_slv0_4_addr i2cSlave4Addr;
                uint8_t i2cSlave4Reg;
                uint8_t i2cSlave4Do;
                union I2c_slv4_ctrl_reg i2cSlave4Ctrl;
                /* * Registry break * */
                union Int_pin_bypass_enable_conf_reg intBypassEnable;
                union Interrupt_enable_reg interruptEnable;
                /* * Registry break * */
                uint8_t i2cSlave0Do;
                uint8_t i2cSlave1Do;
                uint8_t i2cSlave2Do;
                uint8_t i2cSlave3Do;
                /* * Registry break * */
                union I2c_master_delay_control_reg i2cMasterDelayCtrl;
                /* * Registry break * */
                union Accel_interrupt_control_reg accelInterruptCtrl;
                union User_control_reg userCtrl;
                union Power_management1_reg powerMngt1;
                union Power_management2_reg powerMngt2;
                /* * Specific registry access * */
                union Mag_cntl1_reg magCtrl1;
            } _config;

            /** \brief Status of the MPU9250
             *
             * This structure is mapped in the register order
             */
            struct {
                union I2c_master_status_reg i2cMasterStatus;
                /* * Registry break * */
                union Interrupt_status_reg interruptStatus;
                /* * Specific registry access * */
                union Mag_Status1_reg magStatus1;
                /* * Registry break * */
                union Mag_status2_reg magStatus2;
            } _status;

            /**< I2C address of the first 4 slaves */
            uint8_t _slaveDataAddr[4];
            
            
            /** \brief Enable or disable accelerometer self test mode
             *
             * \param x enable for x axis
             * \param y enable for y axis
             * \param z enable for z axis
             *
             */
            void setAccelSelfTest(bool x = false, bool y = false, bool z = false);
            
            /** \brief Enable or disable gyroscope self test mode
             *
             * \param x enable for x axis
             * \param y enable for y axis
             * \param z enable for z axis
             *
             */
            void setGyroSelfTest(bool x = false, bool y = false, bool z = false);

            void updateSlavesDataAddr();

    };



} // namespace mpu9250
#endif // MPU9250_H
