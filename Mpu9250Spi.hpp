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

#ifndef MPU9250_SPI_H
#define MPU9250_SPI_H

#include "Mpu9250.hpp"
#include "SPI.h"

namespace mpu9250 {
    /*
     * In SPI mode, magnetometer can only be
     * accessed by the MPU9250 I2C master.
     * It will be set as slave 0.
     */

    class Mpu9250Spi : public Mpu9250 {
        public :
            Mpu9250Spi(int slaveSelectPin, int interruptPin, uint32_t spiClock) :
                                _slaveSelectPin(slaveSelectPin),
                                _interruptPin(interruptPin),
                                _spiSettings(spiClock, MSBFIRST, SPI_MODE3)
                                {};
            Mpu9250Spi(int slaveSelectPin, int interruptPin) :
                                Mpu9250Spi(slaveSelectPin, interruptPin, 1000000)
                                {};

            /*
             * General
             */
            void init();


            void downloadConfig();
            void downloadStatus();
            void uploadConfig();

            void resetAccelSignalPath();
            void resetFifo();
            void resetGyroSignalPath();
            void resetI2cMaster();
            void resetI2cSlave();
            void resetSignalPathAndRegister();
            void resetTempSignalPath();

            bool testConnection();


            /*
             * Sensors getters
             */

            // Accelerometer
            void getAccelX(int16_t * x) ;
            void getAccelY(int16_t * y) ;
            void getAccelZ(int16_t * z) ;
            void getAccel(int16_t * x, int16_t * y, int16_t * z) ;
            void getAccel(struct Int16Vect3D * values) ;

            // Gyroscope
            void getGyroX(int16_t * x) ;
            void getGyroY(int16_t * y) ;
            void getGyroZ(int16_t * z) ;
            void getGyro(int16_t * x, int16_t * y, int16_t * z) ;
            void getGyro(struct Int16Vect3D * values) ;

            // Magnetometer
            void getMagX(int16_t * x) ;
            void getMagY(int16_t * y) ;
            void getMagZ(int16_t * z) ;
            void getMag(int16_t * x, int16_t * y, int16_t * z) ;
            void getMag(struct Int16Vect3D * values) ;

             // Tempereture
            void getTemp(int16_t * temp) ;

            // Get all sensors data
            void getSensors(struct Int16Vect3D * accel, struct Int16Vect3D * gyro, struct Int16Vect3D * mag, int16_t * temp) ;
            void getAccelOffset(struct Int16Vect3D * offset);
            uint16_t getFifoCount();
            bool readFifo(uint8_t * data, uint16_t size);
            bool readFifo16(uint16_t * data, uint16_t size);
            bool readSlaveData(enum Slave num, uint8_t * data);
            bool readSlaveData16(enum Slave num, uint16_t * data);

        private :
            int _interruptPin;
            int _slaveSelectPin;
            SPISettings _spiSettings;

            void calibrate();
            void hardwareReset();
            void selfTest(float * result);

            /*
             * IO Functions
             */

             // read
            void readRegister(const uint8_t addr, uint8_t * val);
            void readRegister16(const uint8_t addr, uint16_t * val);
            void readMultipleRegister(const uint8_t addr, uint8_t * buf, const size_t count);
            void readMultipleRegister16(const uint8_t addr, uint16_t * buf, const size_t count);

            // Write
            void writeRegister(const uint8_t addr, const uint8_t val);
            void writeRegister16(const uint8_t addr, const uint16_t val);
            void writeMultipleRegister(const uint8_t addr, const uint8_t * val, const size_t count);
            void writeMultipleRegister16(const uint8_t addr, const uint16_t * val, const size_t count);
    };


} // namespace mpu9250


#endif // MPU9250_SPI_H
