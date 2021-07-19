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
#include <termios.h>
#include <unistd.h>

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

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
          std::cout<<"error from tcgetattr" <<std::endl;
          return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 1;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
          std::cout<<"error from tcsetattr" <<std::endl;
          return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
          std::cout<<"error from tggetattr" <<std::endl;
          return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 1;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
          std::cout<<"error setting term attributes" <<std::endl;
}

auto main(int argc, char** argv) -> int {

  bool isInitalized = false;

  char *portname = "/dev/ttyUSB2";
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    std::cout<<"error opening serial port" <<std::endl;
    return 0;
  }

  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity) 
  set_blocking (fd, 0);                // set no blocking

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
