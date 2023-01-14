#include <Arduino.h>
#include <SPI.h>
#include "Flash.h"
#include "W25Q128JV_Registers.h"


/*
    @brief setup the flash chip
    @return true if the flash chip is detected, false otherwise
*/
bool W25Q128::setup(){
    SPI.begin();
    SPI.beginTransaction(this->SPI_Settings);
    digitalWrite(this->CS,LOW);
    SPI.transfer(READ_JEDEC_ID);
    uint8_t manufacturerID = SPI.transfer(0x00);
    uint8_t memtype = SPI.transfer(0x00);
    uint8_t capacity = SPI.transfer(0x00);
    digitalWrite(this->CS,HIGH);
    if (manufacturerID == 0xEF && memtype == 0x40 && capacity == 0x18){ // 0xEF is Winbond, 0x40 is memory type, 0x18 is 2 ^ 24 bytes
        Serial.println("Correct Flash Chip Detected");
        return true;
    }
    else{
        Serial.print("Flash Chip Detected with incorrect ID/Type/Capacity: ");
        Serial.print("Manufacturer ID: ");
        Serial.print(manufacturerID,HEX);
        Serial.print(" Memory Type: ");
        Serial.print(memtype,HEX);
        Serial.print(" Capacity: ");
        Serial.println(capacity,HEX);
        return false;
    }
}

/*
    @brief read a byte from the flash chip
    @param buffer the buffer to store the read byte
    @param page the page to read from
    @param offset the offset to read from
*/
void W25Q128::read(uint8_t *buffer, uint16_t page, uint8_t offset){
    digitalWrite(this->CS,LOW);
    SPI.transfer(READ_DATA);
    this->mode = SingleRead;
    SPI.transfer((page >> 8) & 0xFF);
    SPI.transfer(page & 0xFF);
    SPI.transfer(offset);
    *buffer = SPI.transfer(0x00);
    digitalWrite(this->CS,HIGH);
}

/*
    @brief Enables writing to the flash chip must be called before writing to the flash chip every time you want to write to the flash chip
*/
void W25Q128::WriteEnable(){
    digitalWrite(this->CS,LOW);
    SPI.transfer(WRITE_ENABLE);
    digitalWrite(this->CS,HIGH);
}

/*
    @brief Disables writing to the flash chip must be called after writing to the flash chip every time you want to write to the flash chip
*/
void W25Q128::WriteDisable(){
    digitalWrite(this->CS,LOW);
    SPI.transfer(WRITE_DISABLE);
    digitalWrite(this->CS,HIGH);
}

/*
    @brief Waits for the flash chip to finish writing to the flash chip
*/
void W25Q128::notBusy(){
    digitalWrite(this->CS,LOW);
    SPI.transfer(READ_STATUS_REGISTER_1);
    while(bitRead(SPI.transfer(0x00),0) & 1){
    }
    digitalWrite(this->CS,HIGH);
}

/*
    @brief Dumps the buffer to the flash chip
*/
void W25Q128::DumpBuffer(){
    digitalWrite(this->CS,LOW);
    this -> WriteEnable();
    SPI.transfer(PAGE_PROGRAM);
    SPI.transfer((this->CurrPage >> 8) & 0xFF);
    SPI.transfer(this->CurrPage & 0xFF);
    SPI.transfer(0x00U & 0xFF); // Offset is always 0 since we are writing to the beginning of the page each time
    for (uint8_t i = 0; i < 256; i++){
        SPI.transfer(this->Buffer[i]);
    }
    digitalWrite(this->CS,HIGH);
    this->CurrPage++;
    this->CurrOffset = 0;
    this->notBusy();
    this ->WriteDisable();
}

/*
    @brief Adds data to the buffer locally and dumps the buffer to the flash chip if the buffer is full
    @param data the data to add to the buffer
    @param length the length of the data to add to the buffer
*/
void W25Q128::AddToBuffer(uint8_t *data, uint8_t length){
    if (this -> CurrOffset + length > 256){ // If the data to be added to the buffer will overflow the buffer then dump the buffer and add the data to the new buffer
        DumpBuffer();
        for (uint8_t i = 0; i < length; i++){
            this->Buffer[this -> CurrOffset] = data[i];
            this->CurrOffset++;
        }
    }
    else{ // If the data to be added to the buffer will not overflow the buffer then add the data to the buffer
        for (uint8_t i = 0; i < length; i++){
            this->Buffer[this -> CurrOffset] = data[i];
            this->CurrOffset++;
        }
    }
}