#include <Wire.h>
#include <Arduino.h>

class MAX_M10S_I2C{
    public:
        
        MAX_M10S_I2C(uint8_t I2C_Address, TwoWire *I2C_BUS){
            this->I2C_Address = I2C_Address;
            this->I2C_BUS = I2C_BUS;
        }
        bool setup(){
            // ping the device to see if it is connected
            this->I2C_BUS->beginTransmission(this->I2C_Address);
            if (this->I2C_BUS->endTransmission() != 0){
                return false;
            }
            return true;
        }



    private:
        uint8_t I2C_Address;
        TwoWire *I2C_BUS;

        
};