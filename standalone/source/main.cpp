#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <sys/time.h>

#include <sbgc/SBGC.h>
#include <sbgc/SBGC_Linux.h>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <unistd.h>
#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

#define REALTIME_DATA_REQUEST_INTERAL_MS 50 

static SBGC_cmd_realtime_data_t rt_data;
static uint16_t cur_time_ms, rt_req_last_time_ms;

typedef struct {
	adjustable_var_cfg_t cfg;
	int16_t step;
	int32_t val;
	uint8_t need_update;
} adjustable_var_t;

/* Custom types of adjustable variables. Take any big ID that is not used by the SBGC API */
#define ADJ_VAR_CUSTOM_ID  200
enum {
	ADJ_VAR_ROLL_TRIM = ADJ_VAR_CUSTOM_ID,
};
/* A set of adjustable variables that can be changed by encoder knob.
* You may add any variables listed in the SBGC_adj_vars.h
* Be carefull, this structure is placed in to the RAM, that may be a problem with the low memory for boards like UNO
*/
adjustable_var_t adj_vars[] = {
	{ { ADJ_VAR_ROLL_TRIM, "ROLL_TRIM", -900, 900 }, 1, 0, 0 },
	{ ADJ_VAR_DEF_RC_SPEED_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_RC_SPEED_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_SPEED_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_SPEED_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_LPF_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_LPF_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_DEADBAND, 1, 0, 0 },
};

#define ADJ_VARS_NUM  (sizeof(adj_vars)/sizeof(adjustable_var_t))

/* 
* Sets adj. variable in the local set  by id
*/
void set_local_adj_var(uint8_t id, int32_t val) {
	for(uint8_t i=0; i<ADJ_VARS_NUM; i++) {
		if(adj_vars[i].cfg.id == id) {
			adj_vars[i].val = val;
			break;
		}
	}
}


// Process incoming commands. Call it as frequently as possible, to prevent overrun of serial input buffer.
void process_in_queue() {
	while(sbgc_parser.read_cmd()) {
		SerialCommand &cmd = sbgc_parser.in_cmd;
		
		uint8_t error = 0;
		
    //std::cout<<"process_in_queue cmd id: " << cmd.id << std::endl; 

		switch(cmd.id) {
		// Receive realtime data
		case SBGC_CMD_REALTIME_DATA_3:
		case SBGC_CMD_REALTIME_DATA_4:
			error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
			if(!error) {
        std::cout<<"ROLL: " << rt_data.imu_angle[ROLL] << " PITCH: " << rt_data.imu_angle[ROLL] << " YAW: " << rt_data.imu_angle[YAW] << std::endl;
        std::cout<<"Target ROLL: " << rt_data.target_angle[ROLL] << " Target PITCH: " << rt_data.target_angle[ROLL] << " Target YAW: " << rt_data.target_angle[YAW] << std::endl;

			} else {
				sbgc_parser.onParseError(error);
			}
			break;
			
		
		// Receive the actual values of adjustable variables
		case SBGC_CMD_SET_ADJ_VARS_VAL:
		{
			SBGC_cmd_set_adj_vars_var_t buf[ADJ_VARS_NUM];	
			uint8_t vars_num = ADJ_VARS_NUM;
			error = SBGC_cmd_set_adj_vars_unpack(buf, vars_num, cmd);
			if(!error) {
				// Assign received values to our local set of variables
				for(uint8_t i=0; i<vars_num; i++) {
					set_local_adj_var(buf[i].id, buf[i].val);
				}
			} else {
				sbgc_parser.onParseError(error);
			}
		}
			break;
		
		}
	}
}

auto main(int argc, char** argv) -> int {

  bool isInitalized = false;

	// Create serial port object and open serial port
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(100); // Block when reading until any data is received
	int fd = serialPort.Open();
  std::cout<<"FD :" << fd << std::endl;

	SBGC_Demo_setup(fd);

  // wait for gimbal controller to be initialized.
  sleep(3);

  while(true) {

    struct timeval  tv;
    gettimeofday(&tv, NULL);
    cur_time_ms = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    //std::cout << "Hi from Bugra's 3 program" << std::endl;
    process_in_queue();
	  ////////// Request realtime data with the fixed rate
	  if((cur_time_ms - rt_req_last_time_ms) > REALTIME_DATA_REQUEST_INTERAL_MS) {
      std::cout<<"The diff is : " << cur_time_ms - rt_req_last_time_ms << std::endl;
		  SerialCommand cmd;
      if (!isInitalized) {
        cmd.init(SBGC_CMD_BOARD_INFO);
        isInitalized = true;
      } else {
        //std::cout<<"Request the RealTime Info"<<std::endl;
        cmd.init(SBGC_CMD_REALTIME_DATA_4);
      }
			
		  sbgc_parser.send_cmd(cmd, 0);
		
		  rt_req_last_time_ms = cur_time_ms;
	  }
  }

  return 0;
}
