/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "njr4234.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

static njr4234_pack_s njr4234_pack {
.range = 0, .vel = 0, .size = 0, .snr = 0, .pack_type = 0, .cmd1 = 0, .cmd2 = 0, .cmd3 = 0, .parser.decode_state = 0, .parser.header_msg = 0
};

NJR4234::NJR4234(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_NJR4234;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

NJR4234::~NJR4234()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
NJR4234::init()
{

	_px4_rangefinder.set_min_distance(0.4f);
	_px4_rangefinder.set_max_distance(12.0f);
	_interval = 20000;

	start();

	return PX4_OK;
}

int
NJR4234::collect()
{
	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	unsigned char readbuf[sizeof(_linebuf)] {};
	unsigned readlen = sizeof(readbuf) - 1;

	int ret = 0;
	float distance_m = -1.0f;

	// Check the number of cs available in the buffer
	int cs_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&cs_available);

	if (!cs_available) {
		perf_end(_sample_perf);
		return 0;
	}

	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (_interval * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// parse buffer
		for (int i = 0; i < ret; i++) {
			NJR4234_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m);
		}

		// cs left to parse
		cs_available -= ret;

	} while (cs_available > 0);

	// no valid measurement after parsing buffer
	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void
NJR4234::start()
{
	ScheduleNow();
}

void
NJR4234::stop()
{
	ScheduleClear();
}

void
NJR4234::Run()
{
	if (_fd > 0 ) {
		return;
	}
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config); //to access the termios structure

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR; //turn off the conversion of newline characters to carriage return and newline \r\n

	/* no parity, only 1 stop bit, no hardware flow control */
	uart_config.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

	/* ignore modem control lines and enable receiver (allow to read) */
	uart_config.c_cflag |= (CLOCAL | CREAD);

	unsigned speed = B115200;
	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
	}
}

void
NJR4234::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/**
 * @brief NJR4234 message decoder
 */
int  NJR4234_parser(unsigned char c, char *parserbuf, unsigned *parserbuf_index, NJR4234_PARSE_STATE *state, float *dist)
{
	int ret = -1;

	switch (*state)
	{
		case PREAMBLE:
			// check preamble 0xCC|CC|55|55

			*parserbuf_index++;
			if(c == 0xCC && (*parserbuf_index == 1 || *parserbuf_index == 2))
			{
			}
			else if(c == 0x55 && *parserbuf_index == 3)
			{
			}
			else if(c == 0x55 && *parserbuf_index == 4)
			{
				*parserbuf_index = 0;
				*state = HEADER;
			}
			else
			{
				*parserbuf_index = 0;
			}
			break;
		case HEADER:
			if(*parserbuf_index == 0) //第一個byte決定模式
			{
				//check all three notification responses

				*parserbuf_index++;
				if(c == OUTDATA_HEADER1_MEAS_DIST)
				{
					njr4234_pack.parser.header_msg = 0;
				}
				else if(c == RUNCMD_HEADER1)
				{
					*state = COMMAND;
					*parserbuf_index = 0;
				}
				else if(c == READ_ALL_PARAM_HEADER1)
				{
					njr4234_pack.parser.header_msg = 3;
				}
			}
			else if(*parserbuf_index >= 1)
			{
				*parserbuf_index++;
				if(((c & 0x02) == OUTDATA_HEADER2_STATIONARY && njr4234_pack.parser.header_msg == 0) || njr4234_pack.parser.header_msg == 1)
				{
					njr4234_pack.parser.header_msg = 1;
					if(*parserbuf_index == 4)
					{
						*state = STATIONARY_OBJ;
						*parserbuf_index = 0;
					}
				}
				else if(((c & 0x02) == OUTDATA_HEADER2_MOVING && njr4234_pack.parser.header_msg == 0) || njr4234_pack.parser.header_msg == 2)
				{
					njr4234_pack.parser.header_msg = 2;
					if(*parserbuf_index == 4)
					{
						*state = MOVING_OBJ;
						*parserbuf_index = 0;
						njr4234_pack.parser.data_buf[0] = 0;
						njr4234_pack.parser.data_buf[1] = 0;
						njr4234_pack.parser.data_buf[2] = 0;
						njr4234_pack.parser.data_buf[3] = 0;
					}
				}
				else if(njr4234_pack.parser.header_msg == 3)
				{
					if(*parserbuf_index == 2 && c != READ_ALL_PARAM_HEADER2)
					{
						njr4234_pack.parser.header_msg = 0;
					}
					else if(*parserbuf_index == 4)
					{
						*state = ALL_PARAM;
						*parserbuf_index = 0;
					}
				}
				else
				{
					*state = PREAMBLE;
					*parserbuf_index = 0;
				}
			}
			break;
		case COMMAND: //3 bytes in total
			*parserbuf_index++;
			if(*parserbuf_index == 1)
			{
				njr4234_pack.cmd1 = c;
			}
			else if(*parserbuf_index == 2)
			{
				njr4234_pack.cmd2 = c;
			}
			else
			{
				njr4234_pack.cmd3 = c;
				*parserbuf_index = 0;
				*state = PREAMBLE;
			}
			break;
		case ALL_PARAM:
			db_printf(DB_ALWAYS, "%x ", c);
			*parserbuf_index++;
			if(*parserbuf_index == 23)
			{
				*parserbuf_index = 0;
				*state = PREAMBLE;
			}
			break;
		case STATIONARY_OBJ:
			*parserbuf_index++;
			if(*parserbuf_index == 28)
			{
				*parserbuf_index = 0;
				*state = PREAMBLE;
			}
			break;
		case MOVING_OBJ:
			*parserbuf_index++;
			if(*parserbuf_index <= 8)
			{
				njr4234_pack.parser.data_buf[*parserbuf_index - 1] = c;
			}
			if(*parserbuf_index == 8)
			{
				uint16_t tmp_val = njr4234_pack.parser.data_buf[0] << 8;
				tmp_val |= njr4234_pack.parser.data_buf[1];
				njr4234_pack.range = ((float) tmp_val) * 0.01f;
				*parserbuf_index = 0;
				*state = PREAMBLE;
				tmp_val = njr4234_pack.parser.data_buf[6] << 8;
				tmp_val |= njr4234_pack.parser.data_buf[7];
				return true;
			}
			break;
		case END_SEQ:
			break;
	}
}

