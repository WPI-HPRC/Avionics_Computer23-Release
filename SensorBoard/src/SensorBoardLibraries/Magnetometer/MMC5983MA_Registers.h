#include <Arduino.h>

const uint8_t Xout0 = 0x00; //Xout [17:10] 
const uint8_t Xout1= 0x01; //Xout [9:2] 
const uint8_t Yout0 = 0x02; //Yout [17:10] 
const uint8_t Yout1 = 0x03; //Yout [9:2] 
const uint8_t Zout0 = 0x04; //Zout [17:10] 
const uint8_t Zout1 = 0x05; //Zout [9:2] 
const uint8_t XYZout2 = 0x06; //Xout[1:0], Yout[1:0], Zout[1:0] 
const uint8_t Tout = 0x07; //Temperature output 
const uint8_t Status = 0x08; //Device status 
const uint8_t Internal_control_0 = 0x09; //Control register 0 
const uint8_t Internal_control_1 = 0x0A; //Control register 1 
const uint8_t Internal_control_2 = 0x0B; //Control register 2 
const uint8_t Internal_control_3 = 0x0C; //Control register 3 
const uint8_t Product_ID_1 = 0x2F; //Product ID 
const uint8_t MMC5983MA_Product_ID = 0x30; //MMC5983MA Product ID