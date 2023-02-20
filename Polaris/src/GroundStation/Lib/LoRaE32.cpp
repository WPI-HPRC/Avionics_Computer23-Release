/*
 * LoRaE32.cpp
 *
 * Created on: Jan 29, 2022
 * Author: Peter Dentch
 *
 * Code originally from library: github.com/KrisKasprzak/EBYTE
 */

#include "LoRaE32.h"

/*
 * Constructor for this class LoRaE32,
 * should only be one instance from only one LoRaE32 board per processor
 * @param
 * @param
 * @param
 * @param
 */
LoRaE32::LoRaE32(Stream *s, uint8_t PIN_M0, uint8_t PIN_M1, uint8_t PIN_AUX) {

	_s = s;
	_M0 = PIN_M0;
	_M1 = PIN_M1;
	_AUX = PIN_AUX;

}

/*
 * Initialize the unit--basically this reads the modules parameters and stores the parameters
 * for potential future module programming
 * @param
 * @return
 */
bool LoRaE32::init(uint8_t _Attempts) {

	bool ok = true;

	pinMode(_AUX, INPUT);
	pinMode(_M0, OUTPUT);
	pinMode(_M1, OUTPUT);

	delay(10);				// it's an init function not looped so delay() is ok

	if (_Attempts < 1){
		_Attempts = 1;
	}
	if (_Attempts > 10){
		_Attempts = 10;
	}

	SetMode(MODE_NORMAL);

	// first get the module data (must be called first for some odd reason

	ok = ReadModelData();

	if (!ok){
		return false;
	}

	// now get parameters to put unit defaults into the class variables
	ok = ReadParameters();

	if (!ok){
		return false;
	}

	return true;
}

/*
 * Method to indicate availability
 */
bool LoRaE32::available() {

	return _s->available();
}

/*
 * Method to indicate availability
 */
void LoRaE32::flush() {

	_s->flush();
}

/*
 * Method to write a single byte...not sure how useful this really is. If you need to send
 * more that one byte, put the data into a data structure and send it in a big chunk
 * @param
*/

void LoRaE32::SendByte( uint8_t TheByte) {

	_s->write(TheByte);
}

/*
 * Method to get a single byte...not sure how useful this really is. If you need to get
 * more that one byte, put the data into a data structure and send/receive it in a big chunk
 */

uint8_t LoRaE32::GetByte() {

	return _s->read();
}

/*
 * Method to send a chunk of data provided data is in a struct--my personal favorite as you
 * need not parse or worry about sprintf() inability to handle floats
 * TTP: put your structure definition into a .h file and include in both the sender and reciever
 * sketches
 * NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
 * types each handle ints floats differently
 * @param
 * @param
 * @return
 */
bool LoRaE32::SendStruct(const void *TheStructure, uint16_t size_) {

		_buf = _s->write((uint8_t *) TheStructure, size_);

		//CompleteTask(1000);

		return (_buf == size_);
}

/*
 * Method to get a chunk of data provided data is in a struct--my personal favorite as you
 * need not parse or worry about sprintf() inability to handle floats
 * TTP: put your structure definition into a .h file and include in both the sender and receiver
 * sketches
 * NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
 * types each handle ints floats differently
 * @param
 * @param
 * @return
 */
bool LoRaE32::GetStruct(const void *TheStructure, uint16_t size_) {

	_buf = _s->readBytes((uint8_t *) TheStructure, size_);

	CompleteTask(1000);

	return (_buf == size_);
}

/*
 * Utility method to wait until module is done transmitting
 * a timeout is provided to avoid an infinite loop
 * @param
 */
void LoRaE32::CompleteTask(unsigned long timeout) {

	unsigned long t = millis();

	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}

	// if AUX pin was supplied and look for HIGH state
	// note you can omit using AUX if no pins are available, but you will have to use delay() to let module finish

	// per data sheet control after aux goes high is 2ms so delay for at least that long
	// some MCU are slow so give 50 ms

	if (_AUX != -1) {

		while (digitalRead(_AUX) == LOW) {
			delay(PIN_RECOVER);
			if ((millis() - t) > timeout){
				break;
			}
		}
	}
	else {
		// if you can't use aux pin, use 4K7 pullup with Arduino
		// you may need to adjust this value if transmissions fail
		delay(1000);

	}

	// delay(PIN_RECOVER);
}

/*
 * method to set the mode (program, normal, etc.)
 * @param
 */

void LoRaE32::SetMode(uint8_t mode) {

	// data sheet claims module needs some extra time after mode setting (2ms)
	// most of my projects uses 10 ms, but 40ms is safer

	delay(PIN_RECOVER);

	if (mode == MODE_NORMAL) {

		digitalWrite(_M0, LOW);
		digitalWrite(_M1, LOW);
	}
	else if (mode == MODE_WAKEUP) {

		digitalWrite(_M0, HIGH);
		digitalWrite(_M1, LOW);

	}
	else if (mode == MODE_POWERDOWN) {
		digitalWrite(_M0, LOW);
		digitalWrite(_M1, HIGH);
	}
	else if (mode == MODE_PROGRAM) {
		digitalWrite(_M0, HIGH);
		digitalWrite(_M1, HIGH);
	}

	// data sheet says 2ms later control is returned, let's give just a bit more time
	// these modules can take time to activate pins
	delay(PIN_RECOVER);

	// clear out any junk
	// added rev 5
	// i've had some issues where after programming, the returned model is 0, and all settings appear to be corrupt
	// i imagine the issue is due to the internal buffer full of junk, hence clearing
	// Reset() *MAY* work but this seems better.
	ClearBuffer();

	// wait until aux pin goes back low
	CompleteTask(4000);

}

/*
 * i've asked EBYTE what's supposed to happen--got an unclear answer
 * but my testing indicates it clears buffer
 * I use this when needing to restart the EBYTE after programming while data is still streaming in
 * to let the unit start reading from a cleared internal buffer
 *
 * it does NOT return the ebyte back to factory defaults
 * if your unit gets corrupt or you need to restore values, you will have to do brute force
 * example for and E44-915
 * look at the data sheet for default values
 * Trans.SetAddressH(0);
 * Trans.SetAddressL(0);
 * Trans.SetSpeed(0b00011100);
 * Trans.SetChannel(1);
 * Trans.SetOptions(0b01000100);
 * Trans.SaveParameters(PERMANENT);
 */
void LoRaE32::Reset() {

	SetMode(MODE_PROGRAM);

	_s->write(0xC4);
	_s->write(0xC4);
	_s->write(0xC4);

	CompleteTask(4000);

	SetMode(MODE_NORMAL);

}


void LoRaE32::SetSpeed(uint8_t val) {
	_Speed = val;
}


void LoRaE32::SetOptions(uint8_t val) {
	_Options = val;
}


uint8_t LoRaE32::GetSpeed() {
	return _Speed ;
}


uint8_t LoRaE32::GetOptions() {
	return _Options;
}

/*
 * method to set the high bit of the address
*/
void LoRaE32::SetAddressH(uint8_t val) {
	_AddressHigh = val;
}


uint8_t LoRaE32::GetAddressH() {
	return _AddressHigh;
}

/*
 * method to set the lo bit of the address
*/
void LoRaE32::SetAddressL(uint8_t val) {
	_AddressLow = val;
}


uint8_t LoRaE32::GetAddressL() {
	return _AddressLow;
}


/*
method to set the channel
*/

void LoRaE32::SetChannel(uint8_t val) {
	_Channel = val;
}


uint8_t LoRaE32::GetChannel() {
	return _Channel;
}

/*
 * method to set the air data rate
 */
void LoRaE32::SetAirDataRate(uint8_t val) {

	_AirDataRate = val;
	BuildSpeedByte();
}

uint8_t LoRaE32::GetAirDataRate() {
	return _AirDataRate;
}

/*
 * method to set the parity bit
 */
void LoRaE32::SetParityBit(uint8_t val) {
	_ParityBit = val;
	BuildSpeedByte();
}

uint8_t LoRaE32::GetParityBit( ) {
	return _ParityBit;
}

/*
 * method to set the options bits
 */
void LoRaE32::SetTransmissionMode(uint8_t val) {
	_OptionTrans = val;
	BuildOptionByte();
}

uint8_t LoRaE32::GetTransmissionMode( ) {
	return _OptionTrans;
}

void LoRaE32::SetPullupMode(uint8_t val) {
	_OptionPullup = val;
	BuildOptionByte();
}

uint8_t LoRaE32::GetPullupMode( ) {
	return _OptionPullup;
}

void LoRaE32::SetWORTIming(uint8_t val) {
	_OptionWakeup = val;
	BuildOptionByte();
}

uint8_t LoRaE32::GetWORTIming() {
	return _OptionWakeup;
}

void LoRaE32::SetFECMode(uint8_t val) {
	_OptionFEC = val;
	BuildOptionByte();
}

uint8_t LoRaE32::GetFECMode( ) {
	return _OptionFEC;
}

void LoRaE32::SetTransmitPower(uint8_t val) {

	_OptionPower = val;
	BuildOptionByte();
}

uint8_t LoRaE32::GetTransmitPower() {
	return _OptionPower;
}

/*
 * method to compute the address based on high and low bits
 */
void LoRaE32::SetAddress(uint16_t Val) {
	_AddressHigh = ((Val & 0xFFFF) >> 8);
	_AddressLow = (Val & 0xFF);
}


/*
 * method to get the address which is a collection of hi and lo bytes
 */
uint16_t LoRaE32::GetAddress() {
	return (_AddressHigh << 8) | (_AddressLow );
}

/*
 * set the UART baud rate
 */
void LoRaE32::SetUARTBaudRate(uint8_t val) {
	_UARTDataRate = val;
	BuildSpeedByte();
}

uint8_t LoRaE32::GetUARTBaudRate() {
	return _UARTDataRate;
}

/*
 * method to build the byte for programming (notice it's a collection of a few variables)
 */
void LoRaE32::BuildSpeedByte() {
	_Speed = 0;
	_Speed = ((_ParityBit & 0xFF) << 6) | ((_UARTDataRate & 0xFF) << 3) | (_AirDataRate & 0xFF);
}

/*
 * method to build the option byte for programming (notice it's a collection of a few variables)
 */
void LoRaE32::BuildOptionByte() {
	_Options = 0;
	_Options = ((_OptionTrans & 0xFF) << 7) | ((_OptionPullup & 0xFF) << 6) | ((_OptionWakeup & 0xFF) << 3) | ((_OptionFEC & 0xFF) << 2) | (_OptionPower&0b11);
}

bool LoRaE32::GetAux() {
	return digitalRead(_AUX);
}

/*
 * method to save parameters to the module
 */
void LoRaE32::SaveParameters(uint8_t val) {

	SetMode(MODE_PROGRAM);

	// ClearBuffer();

	/*
	Serial.print("val: ");
	Serial.println(val);
	Serial.print("_AddressHigh: ");
	Serial.println(_AddressHigh);
	Serial.print("_AddressLow: ");
	Serial.println(_AddressLow);
	Serial.print("_Speed: ");
	Serial.println(_Speed);
	Serial.print("_Channel: ");
	Serial.println(_Channel);
	Serial.print("_Options: ");
	Serial.println(_Options);
	*/

	_s->write(val);
	_s->write(_AddressHigh);
	_s->write(_AddressLow);
	_s->write(_Speed);
	_s->write(_Channel);
	_s->write(_Options);

	delay(PIN_RECOVER);

	CompleteTask(4000);

	SetMode(MODE_NORMAL);

}

/*
 * method to print parameters, this can be called anytime after init(), because init gets parameters
 * and any method updates the variables
 */
void LoRaE32::PrintParameters() {

	_ParityBit = (_Speed & 0XC0) >> 6;
	_UARTDataRate = (_Speed & 0X38) >> 3;
	_AirDataRate = _Speed & 0X07;

	_OptionTrans = (_Options & 0X80) >> 7;
	_OptionPullup = (_Options & 0X40) >> 6;
	_OptionWakeup = (_Options & 0X38) >> 3;
	_OptionFEC = (_Options & 0X07) >> 2;
	_OptionPower = (_Options & 0X03);

	Serial.println("----------------------------------------");
	Serial.print(F("Model no.: "));  Serial.println(_Model, HEX);
	Serial.print(F("Version  : "));  Serial.println(_Version, HEX);
	Serial.print(F("Features : "));  Serial.println(_Features, HEX);
	Serial.println(F(" "));
	Serial.print(F("Mode (HEX/DEC/BIN): "));  Serial.print(_Save, HEX); Serial.print(F("/"));  Serial.print(_Save, DEC); Serial.print(F("/"));  Serial.println(_Save, BIN);
	Serial.print(F("AddH (HEX/DEC/BIN): "));  Serial.print(_AddressHigh, HEX); Serial.print(F("/")); Serial.print(_AddressHigh, DEC); Serial.print(F("/"));  Serial.println(_AddressHigh, BIN);
	Serial.print(F("AddL (HEX/DEC/BIN): "));  Serial.print(_AddressLow, HEX); Serial.print(F("/")); Serial.print(_AddressLow, DEC); Serial.print(F("/"));  Serial.println(_AddressLow, BIN);
	Serial.print(F("Sped (HEX/DEC/BIN): "));  Serial.print(_Speed, HEX); Serial.print(F("/")); Serial.print(_Speed, DEC); Serial.print(F("/"));  Serial.println(_Speed, BIN);
	Serial.print(F("Chan (HEX/DEC/BIN): "));  Serial.print(_Channel, HEX); Serial.print(F("/")); Serial.print(_Channel, DEC); Serial.print(F("/"));  Serial.println(_Channel, BIN);
	Serial.print(F("Optn (HEX/DEC/BIN): "));  Serial.print(_Options, HEX); Serial.print(F("/")); Serial.print(_Options, DEC); Serial.print(F("/"));  Serial.println(_Options, BIN);
	Serial.print(F("Addr (HEX/DEC/BIN): "));  Serial.print(GetAddress(), HEX); Serial.print(F("/")); Serial.print(GetAddress(), DEC); Serial.print(F("/"));  Serial.println(GetAddress(), BIN);
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit (HEX/DEC/BIN)    : "));  Serial.print(_ParityBit, HEX); Serial.print(F("/"));  Serial.print(_ParityBit, DEC); Serial.print(F("/"));  Serial.println(_ParityBit, BIN);
	Serial.print(F("SpeedUARTDataRate (HEX/DEC/BIN) : "));  Serial.print(_UARTDataRate, HEX); Serial.print(F("/"));  Serial.print(_UARTDataRate, DEC); Serial.print(F("/"));  Serial.println(_UARTDataRate, BIN);
	Serial.print(F("SpeedAirDataRate (HEX/DEC/BIN)  : "));  Serial.print(_AirDataRate, HEX); Serial.print(F("/"));  Serial.print(_AirDataRate, DEC); Serial.print(F("/"));  Serial.println(_AirDataRate, BIN);
	Serial.print(F("OptionTrans (HEX/DEC/BIN)       : "));  Serial.print(_OptionTrans, HEX); Serial.print(F("/"));  Serial.print(_OptionTrans, DEC); Serial.print(F("/"));  Serial.println(_OptionTrans, BIN);
	Serial.print(F("OptionPullup (HEX/DEC/BIN)      : "));  Serial.print(_OptionPullup, HEX); Serial.print(F("/"));  Serial.print(_OptionPullup, DEC); Serial.print(F("/"));  Serial.println(_OptionPullup, BIN);
	Serial.print(F("OptionWakeup (HEX/DEC/BIN)      : "));  Serial.print(_OptionWakeup, HEX); Serial.print(F("/"));  Serial.print(_OptionWakeup, DEC); Serial.print(F("/"));  Serial.println(_OptionWakeup, BIN);
	Serial.print(F("OptionFEC (HEX/DEC/BIN)         : "));  Serial.print(_OptionFEC, HEX); Serial.print(F("/"));  Serial.print(_OptionFEC, DEC); Serial.print(F("/"));  Serial.println(_OptionFEC, BIN);
	Serial.print(F("OptionPower (HEX/DEC/BIN)       : "));  Serial.print(_OptionPower, HEX); Serial.print(F("/"));  Serial.print(_OptionPower, DEC); Serial.print(F("/"));  Serial.println(_OptionPower, BIN);

	Serial.println("----------------------------------------");

}

/*
 * method to read parameters,
 */
bool LoRaE32::ReadParameters() {

	_Params[0] = 0;
	_Params[1] = 0;
	_Params[2] = 0;
	_Params[3] = 0;
	_Params[4] = 0;
	_Params[5] = 0;

	SetMode(MODE_PROGRAM);

	_s->write(0xC1);

	_s->write(0xC1);

	_s->write(0xC1);

	_s->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));

	_Save = _Params[0];
	_AddressHigh = _Params[1];
	_AddressLow = _Params[2];
	_Speed = _Params[3];
	_Channel = _Params[4];
	_Options = _Params[5];

	_Address =  (_AddressHigh << 8) | (_AddressLow);
	_ParityBit = (_Speed & 0XC0) >> 6;
	_UARTDataRate = (_Speed & 0X38) >> 3;
	_AirDataRate = _Speed & 0X07;

	_OptionTrans = (_Options & 0X80) >> 7;
	_OptionPullup = (_Options & 0X40) >> 6;
	_OptionWakeup = (_Options & 0X38) >> 3;
	_OptionFEC = (_Options & 0X07) >> 2;
	_OptionPower = (_Options & 0X03);

	SetMode(MODE_NORMAL);

	if (0xC0 != _Params[0]){

		return false;
	}

	return true;

}


bool LoRaE32::ReadModelData() {

	_Params[0] = 0;
	_Params[1] = 0;
	_Params[2] = 0;
	_Params[3] = 0;
	_Params[4] = 0;
	_Params[5] = 0;

	bool found = false;
	int i = 0;

	SetMode(MODE_PROGRAM);

	_s->write(0xC3);
	_s->write(0xC3);
	_s->write(0xC3);
	_s->readBytes((uint8_t*)& _Params, (uint8_t) sizeof(_Params));

	_Save = _Params[0];
	_Model = _Params[1];
	_Version = _Params[2];
	_Features = _Params[3];

	if (0xC3 != _Params[0]) {

		// i'm not terribly sure this is the best way to retry
		// may need to set the mode back to normal first....
		for (i = 0; i < 5; i++){
			// Serial.print("trying: "); Serial.println(i);
			_Params[0] = 0;
			_Params[1] = 0;
			_Params[2] = 0;
			_Params[3] = 0;
			_Params[4] = 0;
			_Params[5] = 0;

			_s->write(0xC3);
			_s->write(0xC3);
			_s->write(0xC3);

			_s->readBytes((uint8_t*)& _Params, (uint8_t) sizeof(_Params));

			/*
			Serial.print("_Attempts ");Serial.println(_Attempts);
			Serial.print("_Params[0] ");Serial.println(_Params[0]);
			Serial.print("_Params[1] ");Serial.println(_Params[1]);
			Serial.print("_Params[2] ");Serial.println(_Params[2]);
			Serial.print("_Params[3] ");Serial.println(_Params[3]);
			Serial.print("_Params[4] ");Serial.println(_Params[4]);
			Serial.print("_Params[5] ");Serial.println(_Params[5]);
			*/

			if (0xC3 == _Params[0]){
				found = true;
				break;
			}

			delay(100);
		}
	}
	else {
		found = true;
	}

	SetMode(MODE_NORMAL);

	return found;

}

/*
 * method to get module model and E50-TTL-100 will return 50
 */
uint8_t LoRaE32::GetModel() {

	return _Model;

}

/*
 * method to get module version (undocumented as to the value)
 */
uint8_t LoRaE32::GetVersion() {

	return _Version;
}

/*
 * method to get module version (undocumented as to the value)
 */
uint8_t LoRaE32::GetFeatures() {

	return _Features;
}

/*
 * method to clear the serial buffer
 * without clearing the buffer, i find getting the parameters very unreliable after programming.
 * I suspect stuff in the buffer affects programming hence, let's clean it out
 * This is called as part of the setmode
 */
void LoRaE32::ClearBuffer(){

	unsigned long amt = millis();

	while(_s->available()) {
		_s->read();
		if ((millis() - amt) > 5000) {
          Serial.println("runaway");
          break;
        }
	}

}


