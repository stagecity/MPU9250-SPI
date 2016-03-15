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

#include "Mpu9250Spi.hpp"

namespace mpu9250 {
        
    /* 
     * PUBLIC
     */
     
     /*
      * General
      */
         
    // Based on : https://github.com/kriswiner/MPU-9250
    void Mpu9250Spi::init() {
        pinMode(_slaveSelectPin, OUTPUT);
        pinMode(_interruptPin, INPUT);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.begin();

        while(!testConnection()); //Don't continue if connection failed

        calibrate();

        // Clear sleep mode bit (6), enable all sensors
        _config.powerMngt1.value = 0;
        writeRegister(MPU9250_REG_PWR_MGMT_1, _config.powerMngt1.value);
        delay(100); // Wait for all registers to reset

        // get stable time source
        _config.powerMngt1.clksel = CLOCK_SOURCE_AUTO; // Auto select clock source to be PLL gyroscope reference if ready else
        writeRegister(MPU9250_REG_PWR_MGMT_1, _config.powerMngt1.value);
        delay(200);


        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        _config.config.value = 0;
        _config.config.dlpf_cfg = DLPF_GYRO_41HZ;
        writeRegister(MPU9250_REG_CONFIG, _config.config.value);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        _config.sampleRateDivider = 0x04;       // Use a 200 Hz rate; a rate consistent with the filter update rate
        writeRegister(MPU9250_REG_SMPLRT_DIV, _config.sampleRateDivider);


        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        readRegister(MPU9250_REG_GYRO_CONFIG, &_config.gyroConfig.value);
        _config.gyroConfig.fchoice_b = FCHOICE_DLPF_ENABLE;
        _config.gyroConfig.gyro_fs_sel = GYRO_FULL_SCALE_250DPS;
        writeRegister(MPU9250_REG_GYRO_CONFIG, _config.gyroConfig.value);


        // Set accelerometer full-scale range configuration
        readRegister(MPU9250_REG_ACCEL_CONFIG, &_config.accelConfig.value);
        _config.accelConfig.accel_fs_sel = ACCEL_FULL_SCALE_2G;
        _config.accelConfig.az_st_en = false;
        _config.accelConfig.ay_st_en = false;
        _config.accelConfig.ax_st_en = false;
        writeRegister(MPU9250_REG_ACCEL_CONFIG, _config.accelConfig.value);

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        readRegister(MPU9250_REG_ACCEL_CONFIG2, &_config.accelConfig2.value);
        _config.accelConfig2.dlpf = DLPF_ACCEL_41HZ; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        _config.accelConfig2.disable_dlpf = false;
        writeRegister(MPU9250_REG_ACCEL_CONFIG2, _config.accelConfig2.value);

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        _config.intBypassEnable.bypass_en = false;
        _config.intBypassEnable.fsync_int_mode_en = false;
        _config.intBypassEnable.actl_fsync = false;
        _config.intBypassEnable.int_anyrd_2clear = false;
        _config.intBypassEnable.latch_int_en = true;
        _config.intBypassEnable.open = false;
        _config.intBypassEnable.actl = false;

        writeRegister(MPU9250_REG_INT_PIN_CFG, _config.intBypassEnable.value);


        // Enable data ready (bit 0) interrupt
        _config.interruptEnable.raw_rdy_en = true;
        _config.interruptEnable.fsync_int_en = false;
        _config.interruptEnable.fifo_overflow_en = false;
        _config.interruptEnable.wom_en = false;
        writeRegister(MPU9250_REG_INT_ENABLE,_config.interruptEnable.value);


        delay(100);
        
        // Enable I2C master capabilities
        _config.userCtrl.value = MPU9250_DEFAULT_RESET_VALUE;
        _config.userCtrl.i2c_mst_en = true;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
        
        
        // Set I2C master configuration
        _config.i2cMasterCtrl.i2c_mst_clk = I2C_MST_CLK_400KHZ; // Clock at 400kHz
        _config.i2cMasterCtrl.i2c_mst_p_nsr = true;
        _config.i2cMasterCtrl.slv_3_fifo_en = false;
        _config.i2cMasterCtrl.wait_for_es = true;
        _config.i2cMasterCtrl.mult_mst_en = false;
        writeRegister(MPU9250_REG_I2C_MST_CTRL, _config.i2cMasterCtrl.value);
    
        // Set default i2c delay
        _config.i2cMasterDelayCtrl.value = MPU9250_DEFAULT_RESET_VALUE;
        writeRegister(MPU9250_REG_I2C_MST_DELAY_CTRL, _config.i2cMasterDelayCtrl.value);
    
        // Set magnetometer as Slave0
        setSlaveConf(SLAVE_0, MPU9250_I2C_MAG_ADDRESS, true, false);
        // And try to read Who I Am register to test connection
        setSlaveTransfer(SLAVE_0, MPU9250_MAG_REG_WIA, 0x00, true, 1, false, false, false);
        uploadConfig();
        delay(100);     // wait to be sure read is done
        
        uint8_t tmp;
        readSlaveData(SLAVE_0, &tmp);       
        if(tmp != MPU9250_MAG_WIA_VALUE)    // ensure connection is good!
            while(1);
        
        // Configure magnetometer to continuously mesure, with 16bit precision
        _config.magCtrl1.mode = MAG_CONTINUOUS_MEASUREMENT_100Hz;
        _config.magCtrl1.bitm = true;
        setSlaveTransfer(SLAVE_0, MPU9250_MAG_REG_CNTL, _config.magCtrl1.value, false, 1, false, false, false);
        uploadConfig();
        delay(100);
        
        setSlaveTransfer(SLAVE_0, MPU9250_MAG_REG_ST1, 0x00, true, 8, false, true, true);

        uploadConfig();
        delay(100);
    }

    //TODO magnetometer
    void Mpu9250Spi::downloadConfig() {
        // Read gyro offset
        readMultipleRegister16(MPU9250_REG_XG_OFFSET_H, (uint16_t *) &_config, 3);
        // Read from sampleRateDivider to wakeOnMotionThreshold
        readMultipleRegister(MPU9250_REG_SMPLRT_DIV, &(_config.sampleRateDivider), 7);
        // Read from fifoEnabled to i2cSlave4Reg
        readMultipleRegister(MPU9250_REG_FIFO_EN, &(_config.fifoEnabled.value), 18);
        // Read from intBypassEnable to interruptEnable
        readMultipleRegister(MPU9250_REG_INT_PIN_CFG, &(_config.intBypassEnable.value), 2);
        // read from i2cSlave0Do to i2cSlave3Do
        readMultipleRegister(MPU9250_REG_I2C_SLV0_DO, &(_config.i2cSlave0Do), 4);
        // Read i2cMasterDelayCtrl
        readRegister(MPU9250_REG_I2C_MST_DELAY_CTRL, &(_config.i2cMasterDelayCtrl.value));
        // Read from accelInterruptCtrl to powerMngt2
        readMultipleRegister(MPU9250_REG_MOT_DETECT_CTRL, &(_config.accelInterruptCtrl.value), 4);
    }
    
    //TODO magnetometer
    void Mpu9250Spi::downloadStatus() {
        readRegister(MPU9250_REG_I2C_MST_STATUS, &(_status.i2cMasterStatus.value));
        readRegister(MPU9250_REG_INT_STATUS, &(_status.interruptStatus.value));
        readRegister(MPU9250_REG_EXT_SENS_DATA_00, &(_status.magStatus1.value));
        readRegister(MPU9250_REG_EXT_SENS_DATA_07, &(_status.magStatus2.value));
    }

    //TODO magnetometer
    void Mpu9250Spi::uploadConfig() {
        // Read gyro offset
        writeMultipleRegister16(MPU9250_REG_XG_OFFSET_H, (uint16_t *) &_config, 3);
        // write from sampleRateDivider to wakeOnMotionThreshold
        writeMultipleRegister(MPU9250_REG_SMPLRT_DIV, (uint8_t *) &(_config.sampleRateDivider), 7);
        // write from fifoEnabled to i2cSlave4Reg
        writeMultipleRegister(MPU9250_REG_FIFO_EN, (uint8_t *) &(_config.fifoEnabled), 18);
        // write from intBypassEnable to interruptEnable
        writeMultipleRegister(MPU9250_REG_INT_PIN_CFG, (uint8_t *) &(_config.intBypassEnable), 2);
        // write from i2cSlave0Do to i2cSlave3Do
        writeMultipleRegister(MPU9250_REG_I2C_SLV0_DO, (uint8_t *) &(_config.i2cSlave0Do), 4);
        // write i2cMasterDelayCtrl
        writeRegister(MPU9250_REG_I2C_MST_DELAY_CTRL, _config.i2cMasterDelayCtrl.value);
        // write from accelInterruptCtrl to powerMngt2
        writeMultipleRegister(MPU9250_REG_MOT_DETECT_CTRL, (uint8_t *) &(_config.accelInterruptCtrl), 4);
        
        
    }

    void Mpu9250Spi::resetAccelSignalPath() {
        union Signal_path_reset_reg spr;
        spr.value = 0;
        spr.accel_rst = true;
        writeRegister(MPU9250_REG_SIGNAL_PATH_RESET, spr.value);
        delay(100);
    }

    void Mpu9250Spi::resetFifo() {
        _config.userCtrl.fifo_rst = true;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
        delay(100);
        _config.userCtrl.fifo_rst = false;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
    }
    
    void Mpu9250Spi::resetGyroSignalPath() {
        union Signal_path_reset_reg spr;
        spr.value = 0;
        spr.gyro_rst = true;
        writeRegister(MPU9250_REG_SIGNAL_PATH_RESET, spr.value);
        delay(100);
    }
    
    void Mpu9250Spi::resetI2cMaster() {
        _config.userCtrl.i2c_mst_rst = true;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
        delay(100);
        _config.userCtrl.i2c_mst_rst = false;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
    }
    
    void Mpu9250Spi::resetI2cSlave() {
        _config.userCtrl.i2c_if_dis = true;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
        delay(100);
        _config.userCtrl.i2c_if_dis = false;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
    }
    
    void Mpu9250Spi::resetSignalPathAndRegister() {
        _config.userCtrl.sig_cond_rst = true;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
        delay(100);
        _config.userCtrl.sig_cond_rst = false;
        writeRegister(MPU9250_REG_USER_CTRL, _config.userCtrl.value);
    }
    
    void Mpu9250Spi::resetTempSignalPath() {
        union Signal_path_reset_reg spr;
        spr.value = 0;
        spr.temp_rst = true;
        writeRegister(MPU9250_REG_SIGNAL_PATH_RESET, spr.value);
        delay(100);
    }

    bool Mpu9250Spi::testConnection() {
        uint8_t val;
        readRegister(MPU9250_REG_WHO_AM_I, &val);
        return (val == MPU9250_WHO_AM_I_RESET_VALUE);
    }

    
    /*
     * Sensors getters
     */
    
    // Accelerometer
    void Mpu9250Spi::getAccelX(int16_t * x) {
        readRegister16(MPU9250_REG_ACCEL_XOUT_H, (uint16_t *) x);
    }

    void Mpu9250Spi::getAccelY(int16_t * y) {
        readRegister16(MPU9250_REG_ACCEL_YOUT_H, (uint16_t *) y);
    }

    void Mpu9250Spi::getAccelZ(int16_t * z) {
        readRegister16(MPU9250_REG_ACCEL_ZOUT_H, (uint16_t *) z);
    }

    void Mpu9250Spi::getAccel(int16_t * x, int16_t * y, int16_t * z) {
        struct Int16Vect3D values;
        getAccel(&values);
        *x = values.x;
        *y = values.y;
        *z = values.z;
    }

    void Mpu9250Spi::getAccel(struct Int16Vect3D * values) {
        readMultipleRegister16(MPU9250_REG_ACCEL_XOUT_H, (uint16_t *) values, 3);
    }
    
    // Gyroscope
    void Mpu9250Spi::getGyroX(int16_t * x) {
        readRegister16(MPU9250_REG_GYRO_XOUT_H, (uint16_t *) x);
    }

    void Mpu9250Spi::getGyroY(int16_t * y) {
        readRegister16(MPU9250_REG_GYRO_YOUT_H, (uint16_t *) y);
    }

    void Mpu9250Spi::getGyroZ(int16_t * z) {
        readRegister16(MPU9250_REG_GYRO_ZOUT_H, (uint16_t *) z);
    }

    void Mpu9250Spi::getGyro(int16_t * x, int16_t * y, int16_t * z) {
        struct Int16Vect3D values;
        getGyro(&values);
        *x = values.x;
        *y = values.y;
        *z = values.z;
    }
    void Mpu9250Spi::getGyro(struct Int16Vect3D * values) {
        readMultipleRegister16(MPU9250_REG_GYRO_XOUT_H, (uint16_t *) values, 3);
    }
    
    // Magnetometer
    void Mpu9250Spi::getMagX(int16_t * x) {
        readRegister16(MPU9250_REG_EXT_SENS_DATA_01, (uint16_t *) x);
    }
    
    void Mpu9250Spi::getMagY(int16_t * y) {
        readRegister16(MPU9250_REG_EXT_SENS_DATA_03, (uint16_t *) y);
    }
    
    void Mpu9250Spi::getMagZ(int16_t * z){
        readRegister16(MPU9250_REG_EXT_SENS_DATA_05, (uint16_t *) z);
    }
    
    // Prefer void getMag(struct Int16Vect3D * values)
    void Mpu9250Spi::getMag(int16_t * x, int16_t * y, int16_t * z) {
        struct Int16Vect3D values;
        getMag(&values);
        *x = values.x;
        *y = values.y;
        *z = values.z;
    }
    
    void Mpu9250Spi::getMag(struct Int16Vect3D * values) {
        readMultipleRegister16(MPU9250_REG_EXT_SENS_DATA_01, (uint16_t *) values, 3);
    }
    
    // Temperature
    void Mpu9250Spi::getTemp(int16_t * temp) {
        readRegister16(MPU9250_REG_TEMP_OUT_H, (uint16_t *) temp);
    }
    
    // Get all sensors data
    void Mpu9250Spi::getSensors(struct Int16Vect3D * accel, struct Int16Vect3D * gyro, struct Int16Vect3D * mag, int16_t * temp) {
        uint16_t buf[7];
        readMultipleRegister16(MPU9250_REG_ACCEL_XOUT_H, buf, 7);
        *accel = *((struct Int16Vect3D *) buf);
        *temp = *((int16_t *) (buf + 3));
        *gyro = *((struct Int16Vect3D *) (buf + 4));
        readMultipleRegister16(MPU9250_REG_EXT_SENS_DATA_01, buf, 3);
        *mag = *((struct Int16Vect3D *) buf);
    }

    void Mpu9250Spi::getAccelOffset(struct Int16Vect3D * offset) {
        readMultipleRegister16(MPU9250_REG_XA_OFFSET_H, (uint16_t*) offset, 3);
    }

    uint16_t Mpu9250Spi::getFifoCount() {
        union Fifo_count cnt;
        readRegister16(MPU9250_REG_FIFO_COUNTH, &cnt.value);
        return cnt.fifo_cnt;
    }

    bool Mpu9250Spi::readFifo(uint8_t * data, uint16_t size) {
        if(size > 512)
            return false;
        readMultipleRegister(MPU9250_REG_FIFO_R_W, data, size);
        return true;
    }

    bool Mpu9250Spi::readFifo16(uint16_t * data, uint16_t size) {
        if(size > 512)
            return false;
        readMultipleRegister16(MPU9250_REG_FIFO_R_W, data, size);
        return true;
    }
    
    bool Mpu9250Spi::readSlaveData(enum Slave num, uint8_t * data) {
        switch(num) {
        case SLAVE_0:
            if(_config.i2cSlave0Ctrl.i2c_slv_en)
                readMultipleRegister(_slaveDataAddr[0], data, _config.i2cSlave0Ctrl.i2c_slv_leng);
            break;
        case SLAVE_1:
            if(_config.i2cSlave1Ctrl.i2c_slv_en)
                readMultipleRegister(_slaveDataAddr[1], data, _config.i2cSlave1Ctrl.i2c_slv_leng);

            break;
        case SLAVE_2:
            if(_config.i2cSlave2Ctrl.i2c_slv_en)
                readMultipleRegister(_slaveDataAddr[2], data, _config.i2cSlave2Ctrl.i2c_slv_leng);
            break;
        case SLAVE_3:
            if(_config.i2cSlave3Ctrl.i2c_slv_en)
                readMultipleRegister(_slaveDataAddr[3], data, _config.i2cSlave3Ctrl.i2c_slv_leng);
            break;
        case SLAVE_4:
            if(_config.i2cSlave4Ctrl.i2c_slv4_en)
                readRegister(MPU9250_REG_I2C_SLV4_DI, data);

            break;
        default:
            return false;
        }
        return true;
    }

    bool Mpu9250Spi::readSlaveData16(enum Slave num, uint16_t * data) {
        switch(num) {
        case SLAVE_0:
            if(_config.i2cSlave0Ctrl.i2c_slv_en)
                readMultipleRegister16(_slaveDataAddr[0], data, _config.i2cSlave0Ctrl.i2c_slv_leng);
            break;
        case SLAVE_1:
            if(_config.i2cSlave1Ctrl.i2c_slv_en)
                readMultipleRegister16(_slaveDataAddr[1], data, _config.i2cSlave1Ctrl.i2c_slv_leng);

            break;
        case SLAVE_2:
            if(_config.i2cSlave2Ctrl.i2c_slv_en)
                readMultipleRegister16(_slaveDataAddr[2], data, _config.i2cSlave2Ctrl.i2c_slv_leng);
            break;
        case SLAVE_3:
            if(_config.i2cSlave3Ctrl.i2c_slv_en)
                readMultipleRegister16(_slaveDataAddr[3], data, _config.i2cSlave3Ctrl.i2c_slv_leng);
            break;
            default:
                return false;
        }
        return true;
    }


    /*
     * PRIVATE
     */

    // Based on : https://github.com/kriswiner/MPU-9250
    void Mpu9250Spi::calibrate() {
        uint16_t data[3]; // data array to hold accelerometer and gyro x, y, z, data
        uint16_t ii, packet_count, fifo_count;
        int32_t gyro_bias[3]  = {0, 0, 0};

        // reset device
        hardwareReset();
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        writeRegister(MPU9250_REG_PWR_MGMT_1, 0x01);
        writeRegister(MPU9250_REG_PWR_MGMT_1, 0x00);
        delay(200);

        // Configure device for bias calculation
        writeRegister(MPU9250_REG_INT_ENABLE, 0x00);    // Disable all interrupts
        writeRegister(MPU9250_REG_FIFO_EN, 0x00);       // Disable FIFO
        writeRegister(MPU9250_REG_PWR_MGMT_1, 0x00);    // Turn on internal clock source
        writeRegister(MPU9250_REG_I2C_MST_CTRL, 0x00);  // Disable I2C master
        writeRegister(MPU9250_REG_USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
        writeRegister(MPU9250_REG_USER_CTRL, 0x0C);     // Reset FIFO and DMP
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        writeRegister(MPU9250_REG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
        writeRegister(MPU9250_REG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeRegister(MPU9250_REG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        delay(40);

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeRegister(MPU9250_REG_USER_CTRL, 0x40);   // Enable FIFO
        writeRegister(MPU9250_REG_FIFO_EN, 0x70);     // Enable gyro sensor for FIFO
        delay(80); // accumulate 80 samples in 80 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeRegister(MPU9250_REG_FIFO_EN, 0x00);        // Disable gyro sensor for FIFO
        readRegister16(MPU9250_REG_FIFO_COUNTH, &fifo_count);

        packet_count = fifo_count/6;// How many sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++) {
            readMultipleRegister16(MPU9250_REG_FIFO_R_W, &data[0], 3);  // read data for averaging

            gyro_bias[0]  += (int16_t) data[0];
            gyro_bias[1]  += (int16_t) data[1];
            gyro_bias[2]  += (int16_t) data[2];
        }

        gyro_bias[0]  /= packet_count;
        gyro_bias[1]  /= packet_count;
        gyro_bias[2]  /= packet_count;

        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        // Biases are additive, so change sign on calculated average gyro biases
        _config.gyroXOffset = (int16_t) (-gyro_bias[0]/4);
        _config.gyroYOffset = (int16_t) (-gyro_bias[1]/4);
        _config.gyroZOffset = (int16_t) (-gyro_bias[2]/4);
        // Push gyro biases to hardware registers
        writeMultipleRegister16(MPU9250_REG_XG_OFFSET_H, (uint16_t *) &_config.gyroXOffset, 3);

        delay(100);
    }

    void Mpu9250Spi::hardwareReset() {
        union Power_management1_reg pm;
        pm.value = 0;
        pm.h_reset = true;
        writeRegister(MPU9250_REG_PWR_MGMT_1, pm.value);
    }

    // Based on : https://github.com/kriswiner/MPU-9250
    //result must be a 3 cells array
    void Mpu9250Spi::selfTest(float * result) {
        int16_t rawData[3] = {0, 0, 0};
        uint8_t selfTest[6];
        int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
        float factoryTrim[6];
        uint8_t FS = 0;
        union Config_reg config;
        union Gyro_config_reg gyroConfig;
        union Accel_config_reg accelConfig;
        union Accel_config_reg2 accelConfig2;
        config.value = 0;
        gyroConfig.value = 0;
        accelConfig.value = 0;
        accelConfig2.value = 0;

        config.dlpf_cfg = DLPF_GYRO_92HZ;
        gyroConfig.gyro_fs_sel = GYRO_FULL_SCALE_250DPS;
        accelConfig2.dlpf = DLPF_ACCEL_92HZ;
        accelConfig.accel_fs_sel = ACCEL_FULL_SCALE_2G;

        writeRegister(MPU9250_REG_SMPLRT_DIV, 0x00);                    // Set gyro sample rate to 1 kHz
        writeRegister(MPU9250_REG_CONFIG, config.value);                // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        writeRegister(MPU9250_REG_GYRO_CONFIG, gyroConfig.value);       // Set full scale range for the gyro to 250 dps
        writeRegister(MPU9250_REG_ACCEL_CONFIG2, accelConfig2.value);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        writeRegister(MPU9250_REG_ACCEL_CONFIG, accelConfig.value);     // Set full scale range for the accelerometer to 2 g



        for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

            readMultipleRegister16(MPU9250_REG_ACCEL_XOUT_H, (uint16_t *) &rawData[0], 3);
            aAvg[0] += rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += rawData[1] ;
            aAvg[2] += rawData[2] ;

            readMultipleRegister16(MPU9250_REG_GYRO_XOUT_H, (uint16_t *) &rawData[0], 3);
            gAvg[0] += rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += rawData[1] ;
            gAvg[2] += rawData[2] ;
        }

        for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }


        gyroConfig.zgyro_cten = true;
        gyroConfig.ygyro_cten = true;
        gyroConfig.xgyro_cten = true;

        accelConfig.az_st_en = true;
        accelConfig.az_st_en = true;
        accelConfig.az_st_en = true;

        writeRegister(MPU9250_REG_ACCEL_CONFIG, accelConfig.value); // Enable self test on all three axes and set accelerometer range to +/- 2 g
        writeRegister(MPU9250_REG_GYRO_CONFIG, gyroConfig.value);   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

        delay(25);  // Delay a while to let the device stabilize

        for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
            readMultipleRegister16(MPU9250_REG_ACCEL_XOUT_H, (uint16_t *) &rawData[0], 3);
            aSTAvg[0] += rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += rawData[1] ;
            aSTAvg[2] += rawData[2] ;

            readMultipleRegister16(MPU9250_REG_GYRO_XOUT_H, (uint16_t *) &rawData[0], 3);
            gSTAvg[0] += rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += rawData[1] ;
            gSTAvg[2] += rawData[2] ;
        }

        for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        gyroConfig.value = 0;
        accelConfig.value = 0;

        // Configure the gyro and accelerometer for normal operation
        writeRegister(MPU9250_REG_ACCEL_CONFIG, accelConfig.value);
        writeRegister(MPU9250_REG_GYRO_CONFIG, gyroConfig.value);

        delay(25);  // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        readMultipleRegister(MPU9250_REG_SELF_TEST_X_ACCEL, &selfTest[0], 3);
        readMultipleRegister(MPU9250_REG_SELF_TEST_X_GYRO, &selfTest[3], 3);


        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++) {
            result[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
            result[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
        }

    }


    /*
     * Read functions
     */

    void Mpu9250Spi::readRegister(const uint8_t addr, uint8_t * val) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr | MPU9250_SPI_READ_MASK);
        *val =  SPI.transfer(0xFF);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::readRegister16(const uint8_t addr, uint16_t * val) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr | MPU9250_SPI_READ_MASK);
        *val =  SPI.transfer16(0xFFFF);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::readMultipleRegister(const uint8_t addr, uint8_t * buf, const size_t count) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr | MPU9250_SPI_READ_MASK);
        SPI.transfer((void*) buf, count);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::readMultipleRegister16(const uint8_t addr, uint16_t * buf, const size_t count) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr | MPU9250_SPI_READ_MASK);

        for(size_t i = 0; i < count; i++)
            buf[i] =  SPI.transfer16(0xFFFF);

        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    /*
     * Write functions
     */

    void Mpu9250Spi::writeRegister(const uint8_t addr, const uint8_t val) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr);
        SPI.transfer(val);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::writeRegister16(const uint8_t addr, const uint16_t val) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr);
        SPI.transfer16(val);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::writeMultipleRegister(const uint8_t addr, const uint8_t * val, const size_t count) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr);
        for(size_t i = 0; i < count; i++)
            SPI.transfer(val[i]);
        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

    void Mpu9250Spi::writeMultipleRegister16(const uint8_t addr, const uint16_t * val, const size_t count) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_slaveSelectPin, LOW);
        SPI.transfer(addr);

        for(size_t i = 0; i < count; i++)
            SPI.transfer16(val[i]);

        digitalWrite(_slaveSelectPin, HIGH);
        SPI.endTransaction();
    }

} // namespace mpu9250
