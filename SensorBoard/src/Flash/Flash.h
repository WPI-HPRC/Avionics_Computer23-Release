#include <Arduino.h>
#include <SPI.h>
enum Mode{
    SingleRead,
    SingleWrite,
    StreamRead,
    StreamWrite,
    Idle
};

class W25Q128{
    private:
        uint8_t CS;
        Mode mode = Idle;
        uint16_t Max_Page_Index = 0xFFFFU; // 2^16 - 1 since the number of pages in the flash chip is 2^16
        uint8_t Max_Offset_Index = 0xFFU; // 2^8 - 1 since the number of offsets in a page is 2^8
        uint16_t CurrPage = 0; // Keep track of the current page
        uint8_t CurrOffset = 0; // Keep track of the current offset
        SPISettings SPI_Settings = SPISettings(1000000, MSBFIRST, SPI_MODE0);
        uint8_t Buffer[256] = {0};
        void WriteEnable();
        void WriteDisable();
        void notBusy();
        void DumpBuffer();
    public:
        void read(uint8_t *buffer, uint16_t page, uint8_t offset);
        W25Q128(uint8_t CS){
            this->CS = CS;
            pinMode(CS, OUTPUT);
            digitalWrite(CS, HIGH);
        }
        bool setup();
        void AddToBuffer(uint8_t *data, uint8_t length);
};