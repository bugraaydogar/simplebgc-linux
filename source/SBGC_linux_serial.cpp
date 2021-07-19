
#include "sbgc/SBGC_Linux.h"

/* Defines serial port routines required for SBGC_parser, here */
class LinuxSerialPort : public SBGC_ComObj {
	SerialPort *serialPort;
	
  public:
	inline void init(SerialPort *s) {
		serialPort = s;
	}

	virtual uint16_t getBytesAvailable() {
		return 0;
    //return serialPort->available();
	}
	
	virtual uint8_t readByte() {
		std::string data;
    serialPort->Read(data);
    std::cout<<"ReadByte: "<< data << std::endl;
    return 0;
	}
	
	virtual void writeByte(uint8_t b) {
    std::cout<<"writeByte: "<< std::hex << b << std::endl;
    serialPort->Write(std::to_string(b));
	}
	
	// Arduino com port is not buffered, so empty space is unknown.
	virtual uint16_t getOutEmptySpace() {
		return 0xFFFF;
	}

};


/* Global variables */
SBGC_Parser sbgc_parser;  // SBGC command parser. Define one for each port.
LinuxSerialPort com_obj; // COM-port wrapper required for parser



// Prepare hardware, used in examples
void SBGC_Demo_setup(SerialPort *serial) {
  com_obj.init(serial);
  sbgc_parser.init(&com_obj);
}
