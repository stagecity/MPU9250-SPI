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

#include "Mpu9250.hpp"
#include <string.h>

namespace mpu9250 {

    /*
     * Sensors
     */

    void Mpu9250::setAccelDLPF(bool enable, enum DLPFAccel dlpfAccel) {
        _config.accelConfig2.disable_dlpf = !enable;
        _config.accelConfig2.dlpf = dlpfAccel;
    }

    void Mpu9250::setAccelFullScale(enum AccelFullScale fscale) {
        _config.accelConfig.accel_fs_sel = fscale;
    }

    void Mpu9250::setAccelInterrupt(bool enableWakeOnMotion, bool compareSampleWithPrevious) {
        _config.accelInterruptCtrl.accel_intel_mode = compareSampleWithPrevious;
        _config.accelInterruptCtrl.accel_intel_en = enableWakeOnMotion;
    }

    void Mpu9250::setAccelOnlyLowPower(bool enable, enum AccelLowPowerOutFreq wakeFreq) {
        _config.powerMngt1.cycle = enable;
        _config.powerMngt1.sleep = false;
        _config.powerMngt1.gyro_standby = false;
        setEnabledSensor(!enable, !enable, !enable, true, true, true, !enable);
        _config.accelLowPower.lposc_clksel = wakeFreq;
    }

    void Mpu9250::setGyroDLPF(enum Gyro_Fchoice_b state, enum DLPFGyro dlpfGyro) {
        _config.config.dlpf_cfg = dlpfGyro;
        _config.gyroConfig.fchoice_b = state;
    }

    void Mpu9250::setGyroFullScale(enum GyroFullScale fscale) {
        _config.gyroConfig.gyro_fs_sel = fscale;
    }

    void Mpu9250::setEnabledSensor(bool gyroX, bool gyroY, bool gyroZ, bool accelX, bool accelY, bool accelZ, bool temp) {
        _config.powerMngt2.disable_xa = !accelX;
        _config.powerMngt2.disable_xg = !gyroX;
        _config.powerMngt2.disable_ya = !accelY;
        _config.powerMngt2.disable_yg = !gyroY;
        _config.powerMngt2.disable_za = !accelZ;
        _config.powerMngt2.disable_zg = !gyroZ;
        _config.powerMngt1.pd_ptat = !temp;
    }

    void Mpu9250::setMagnetometer(enum MagOperationMode mode, bool output16bit) {
        _config.magCtrl1.mode = mode;
        _config.magCtrl1.bitm = output16bit;
    }
    

    /*
     * General
     */

    void Mpu9250::setClockSource(enum ClockSource clksel) {
        _config.powerMngt1.clksel = clksel;
    }

    void Mpu9250::setFifo(bool enable, bool overwrite, bool accel, bool gyroX, bool gyroY, bool gyroZ, bool temp, bool mag, bool slv1, bool slv2, bool slv3) {
        _config.config.fifo_mod = !overwrite;
        _config.fifoEnabled.accel = accel;
        _config.fifoEnabled.gyro_xout = gyroX;
        _config.fifoEnabled.gyro_yout = gyroY;
        _config.fifoEnabled.gyro_zout = gyroZ;
        _config.fifoEnabled.temp_out = temp;
        _config.fifoEnabled.slv_1 = slv1;
        _config.fifoEnabled.slv_2 = slv2;
        _config.i2cMasterCtrl.slv_3_fifo_en = slv3;
        _config.fifoEnabled.slv_0 = mag;
        _config.userCtrl.fifo_en = enable;
    }

    void Mpu9250::setFsync(enum Fsync fsync) {
        _config.config.ext_sync_set = fsync;
    }

    void Mpu9250::setSampleRateDivider(uint8_t srd) {
        _config.sampleRateDivider = srd;
    }

    void Mpu9250::setSleep(bool sleep) {
        _config.powerMngt1.sleep = sleep;
    }

    void Mpu9250::setWakeOnMotionThreshold(uint8_t threshold) {
        _config.wakeOnMotionThreshold = threshold;
    }


    /*
     * Interrupt
     */

    void Mpu9250::setInterrupt(bool fsyncAsInterrupt, bool fsyncIntLow, bool clearOnAnyRead, bool holdInt, bool openDrain, bool logicLevelLow) {
        _config.intBypassEnable.fsync_int_mode_en = fsyncAsInterrupt;
        _config.intBypassEnable.actl_fsync = fsyncIntLow;
        _config.intBypassEnable.int_anyrd_2clear = clearOnAnyRead;
        _config.intBypassEnable.latch_int_en = holdInt;
        _config.intBypassEnable.open = openDrain;
        _config.intBypassEnable.actl = logicLevelLow;
    }

    void Mpu9250::setInterruptPin(bool wakeOnMotion, bool fsync, bool fifoOverflow, bool rawDataAvailable) {
        _config.interruptEnable.wom_en = wakeOnMotion;
        _config.interruptEnable.fsync_int_en = fsync;
        _config.interruptEnable.raw_rdy_en = rawDataAvailable;
        _config.interruptEnable.fifo_overflow_en = fifoOverflow;
    }

    void Mpu9250::setSlave4Interrupt(bool enableIntOnComplete) {
        _config.i2cSlave4Ctrl.slv4_done_int_en = enableIntOnComplete;
    }


    /*
     * I2C
     */

    bool Mpu9250::setSlaveConf(enum Slave num, unsigned char addr, bool enable, bool delayEs) {
        switch(num) {
        case SLAVE_0:
            _config.i2cSlave0Addr.i2c_id = addr;
            _config.i2cSlave0Ctrl.i2c_slv_en = enable;
            _config.i2cMasterDelayCtrl.i2c_slv0_dly_en = delayEs;
            break;
        case SLAVE_1:
            _config.i2cSlave1Addr.i2c_id = addr;
            _config.i2cSlave1Ctrl.i2c_slv_en = enable;
            _config.i2cMasterDelayCtrl.i2c_slv1_dly_en = delayEs;
            break;
        case SLAVE_2:
            _config.i2cSlave2Addr.i2c_id = addr;
            _config.i2cSlave2Ctrl.i2c_slv_en = enable;
            _config.i2cMasterDelayCtrl.i2c_slv2_dly_en = delayEs;
            break;
        case SLAVE_3:
            _config.i2cSlave3Addr.i2c_id = addr;
            _config.i2cSlave3Ctrl.i2c_slv_en = enable;
            _config.i2cMasterDelayCtrl.i2c_slv3_dly_en = delayEs;
            break;
        case SLAVE_4:
            _config.i2cSlave4Addr.i2c_id = addr;
            _config.i2cSlave4Ctrl.i2c_slv4_en = enable;
            _config.i2cMasterDelayCtrl.i2c_slv4_dly_en = delayEs;
            break;
        default:
            return false;
        }
        updateSlavesDataAddr();
        return true;
    }

    bool Mpu9250::setSlaveTransfer(enum Slave num, unsigned char regAddr, uint8_t dataOut, bool isRead, unsigned char nByte, bool dontWriteRegAddr, bool swapByte, bool groupOddEnd) {
        if(nByte > 15)
            return false;
        switch(num) {
        case SLAVE_0:
            _config.i2cSlave0Reg = regAddr;
            _config.i2cSlave0Addr.i2c_slave_read = isRead;
            _config.i2cSlave0Ctrl.i2c_slv_leng = nByte;
            _config.i2cSlave0Ctrl.i2c_slv_reg_dis = dontWriteRegAddr;
            _config.i2cSlave0Ctrl.i2c_slv_byte_sw = swapByte;
            _config.i2cSlave0Ctrl.i2c_slv_grp = groupOddEnd;
            _config.i2cSlave0Do = dataOut;
            break;
        case SLAVE_1:
            _config.i2cSlave1Reg = regAddr;
            _config.i2cSlave1Addr.i2c_slave_read = isRead;
            _config.i2cSlave1Ctrl.i2c_slv_leng = nByte;
            _config.i2cSlave1Ctrl.i2c_slv_reg_dis = dontWriteRegAddr;
            _config.i2cSlave1Ctrl.i2c_slv_byte_sw = swapByte;
            _config.i2cSlave1Ctrl.i2c_slv_grp = groupOddEnd;
            _config.i2cSlave1Do = dataOut;
            break;
        case SLAVE_2:
            _config.i2cSlave2Reg = regAddr;
            _config.i2cSlave2Addr.i2c_slave_read = isRead;
            _config.i2cSlave2Ctrl.i2c_slv_leng = nByte;
            _config.i2cSlave2Ctrl.i2c_slv_reg_dis = dontWriteRegAddr;
            _config.i2cSlave2Ctrl.i2c_slv_byte_sw = swapByte;
            _config.i2cSlave2Ctrl.i2c_slv_grp = groupOddEnd;
            _config.i2cSlave2Do = dataOut;
            break;
        case SLAVE_3:
            _config.i2cSlave3Reg = regAddr;
            _config.i2cSlave3Addr.i2c_slave_read = isRead;
            _config.i2cSlave3Ctrl.i2c_slv_leng = nByte;
            _config.i2cSlave3Ctrl.i2c_slv_reg_dis = dontWriteRegAddr;
            _config.i2cSlave3Ctrl.i2c_slv_byte_sw = swapByte;
            _config.i2cSlave3Ctrl.i2c_slv_grp = groupOddEnd;
            _config.i2cSlave3Do = dataOut;
            break;
        case SLAVE_4:
            _config.i2cSlave4Reg = regAddr;
            _config.i2cSlave4Addr.i2c_slave_read = isRead;
            _config.i2cSlave4Ctrl.i2c_slv4_reg_dis = dontWriteRegAddr;
            _config.i2cSlave4Do = dataOut;
            break;
        default:
            return false;
        }
        updateSlavesDataAddr();
        return true;
    }

    void Mpu9250::setI2cMaster(enum I2cMstClk clock, bool stopBtwRead, bool delayDataRdyInt, bool multMasterCapa, bool delayESShadow) {
        _config.i2cMasterCtrl.i2c_mst_clk = clock;
        _config.i2cMasterCtrl.i2c_mst_p_nsr = stopBtwRead;
        _config.i2cMasterCtrl.wait_for_es = delayDataRdyInt;
        _config.i2cMasterCtrl.mult_mst_en = multMasterCapa;
        _config.i2cMasterDelayCtrl.delay_es_shadow = delayESShadow;
    }

    void Mpu9250::setSlave4MstDelay(unsigned char delay) {
        if(delay < 32)
            _config.i2cSlave4Ctrl.i2c_mst_dly = delay;
    }


    /*
     * Status
     */

    bool Mpu9250::isDataReady() {
        return _status.interruptStatus.raw_data_rdy_int;
    }

    bool Mpu9250::isFifoOverflow() {
        return _status.interruptStatus.fifo_overflow_int;
    }

    bool Mpu9250::isFsyncInt() {
        return _status.interruptStatus.fsync_int;
    }

    bool Mpu9250::isI2cLostArbitration() {
        return _status.i2cMasterStatus.i2c_lost_arb;
    }

    bool Mpu9250::isI2cSlave4TransfertComplete() {
        return _status.i2cMasterStatus.i2c_slv4_done;
    }

    bool Mpu9250::isSlaveNack(unsigned short slave) {
        switch (slave) {
        case 0 :
            return _status.i2cMasterStatus.i2c_slv0_nack;
            break;
        case 1 :
            return _status.i2cMasterStatus.i2c_slv1_nack;
            break;
        case 2:
            return _status.i2cMasterStatus.i2c_slv2_nack;
            break;
        case 3:
            return _status.i2cMasterStatus.i2c_slv3_nack;
            break;
        case 4:
            return _status.i2cMasterStatus.i2c_slv4_nack;
            break;
        }

        return false;
    }

    bool Mpu9250::isPassThroughFsyncInt() {
        return _status.i2cMasterStatus.pass_through;
    }

    bool Mpu9250::isWakeOnMotionInt() {
        return _status.interruptStatus.wom_int;
    }

	// Magnetometer Status
	bool Mpu9250::isMagDataReady() {
		return _status.magStatus1.drdy;
	}
	
	bool Mpu9250::isMagDataOverrun() {
		return _status.magStatus1.dor;
	}
	
	bool Mpu9250::isMagOverflow() {
		return _status.magStatus2.hofl;
	}
	
	bool Mpu9250::isMag16BitOutput() {
		return _status.magStatus2.bitm;
	}
            
    /*
     * Getters
     */

     // Based on : https://github.com/kriswiner/MPU-9250
    float Mpu9250::getAccelRes() {
        switch(_config.accelConfig.accel_fs_sel) {
            case ACCEL_FULL_SCALE_2G:
                return 4.0/65535.0;
            break;
            case ACCEL_FULL_SCALE_4G:
                return 8.0/65535.0;
            break;
            case ACCEL_FULL_SCALE_8G:
                return 16.0/65535.0;
            break;
            case ACCEL_FULL_SCALE_16G:
                return 32.0/65535.0;
            break;
        }
        return NAN;
    }

    // Based on : https://github.com/kriswiner/MPU-9250
    float Mpu9250::getGyroRes() {
        switch(_config.gyroConfig.gyro_fs_sel) {
            case GYRO_FULL_SCALE_250DPS:
                return (500.0/65535.0);
            break;
            case GYRO_FULL_SCALE_500DPS:
                return (1000.0/65535.0);
            break;
            case GYRO_FULL_SCALE_1000DPS:
                return (2000.0/65535.0);
            break;
            case GYRO_FULL_SCALE_2000DPS:
                return (4000.0/65535.0);
            break;
        }
        return NAN;
    }



    /*
     * Protected Functions
     */

    void Mpu9250::setAccelSelfTest(bool x, bool y, bool z) {
        _config.accelConfig.ax_st_en = x;
        _config.accelConfig.ay_st_en = y;
        _config.accelConfig.az_st_en = z;
    }

    void Mpu9250::setGyroSelfTest(bool x, bool y, bool z) {
        _config.gyroConfig.xgyro_cten = x;
        _config.gyroConfig.ygyro_cten = y;
        _config.gyroConfig.zgyro_cten = z;
    }

    void Mpu9250::updateSlavesDataAddr() {
        if(_config.i2cSlave0Ctrl.i2c_slv_en) {
            _slaveDataAddr[0] = MPU9250_REG_EXT_SENS_DATA_00;
        } else {
            _slaveDataAddr[0] = MPU9250_REG_EXT_SENS_DATA_00 - _config.i2cSlave0Ctrl.i2c_slv_leng;
        }
        if(_config.i2cSlave1Ctrl.i2c_slv_en) {
            _slaveDataAddr[1] = _slaveDataAddr[0] + _config.i2cSlave0Ctrl.i2c_slv_leng;
        } else {
            _slaveDataAddr[1] = _slaveDataAddr[0] - _config.i2cSlave1Ctrl.i2c_slv_leng;
        }
        if(_config.i2cSlave2Ctrl.i2c_slv_en) {
            _slaveDataAddr[2] = _slaveDataAddr[1] + _config.i2cSlave1Ctrl.i2c_slv_leng;
        } else {
            _slaveDataAddr[2] = _slaveDataAddr[1] - _config.i2cSlave2Ctrl.i2c_slv_leng;
        }
        if(_config.i2cSlave3Ctrl.i2c_slv_en) {
            _slaveDataAddr[3] = _slaveDataAddr[2] + _config.i2cSlave2Ctrl.i2c_slv_leng;
        } else {
            _slaveDataAddr[3] = _slaveDataAddr[2] - _config.i2cSlave3Ctrl.i2c_slv_leng;
        }
    }

} // namespace mpu9250
