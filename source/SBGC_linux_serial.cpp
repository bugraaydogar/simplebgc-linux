
#include "sbgc/SBGC_Linux.h"
#include <iostream>
/* Defines serial port routines required for SBGC_parser, here */
class LinuxSerialPort : public SBGC_ComObj {
	int fd;
	
  public:
	inline void init(int _fd) {
		fd = _fd;
	}

	virtual uint16_t getBytesAvailable() {
		return 0;
    //return serialPort->available();
	}
	
	virtual uint8_t readByte() {
    //std::cout<<"Reading from Serial" << std::endl;
    uint8_t buf;
    int retval = read(fd, &buf, sizeof(uint8_t));
    if( retval < 0 ) {
      std::cout<<"Error during readByte" << std::endl;
      return 0;
    }

    return buf;
	}
	
	virtual void writeByte(uint8_t b) {
    //std::cout<<"Written Data is: " << b << std::endl;
    int retval = write(fd, &b, sizeof(b));
    if( retval < 0 ) {
      std::cout<<"Error during writeByte" << std::endl;
    } else {
      //std::cout<<"Written Byte is: " << retval <<std::endl;
    }
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
void SBGC_Demo_setup(int fd) {
  com_obj.init(fd);
  sbgc_parser.init(&com_obj);
}
