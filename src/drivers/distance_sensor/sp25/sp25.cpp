/****************************************************************************
 *
 *   Copyright (c) 2014-2024 PX4 Development Team. All rights reserved.
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

#include "sp25.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <string.h>

static sp25_package sp25_pack {.vel = 0.0f, .size = 0, .snr = 0, .pack_type = 0};

SP25::SP25(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	//* store port name */
	strncpy(_port, port, sizeof(_port) - 1); //copy port to _port

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SP25;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

SP25::~SP25()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SP25::init()
{
	_px4_rangefinder.set_min_distance(0.1f);
	_px4_rangefinder.set_max_distance(30.0f);

	int ret = 0;
	do {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		unsigned speed = B115200;
		termios uart_config {};
		int termios_state {};

		tcgetattr(_fd, &uart_config);

		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;			// turn off existing size setting in terminal settings
		uart_config.c_cflag |= CS8;			// 8-bit characters (per byte)
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}

	} while(0);

	::close(_fd);
	_fd = -1;
	if (ret == PX4_OK) {
		start();
	}

	return ret;
}


int SP25::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen(緩衝區長度) minus null termination */
	unsigned char readbuf[sizeof(_linebuf)] {};
	unsigned readlen = sizeof(readbuf) - 1;

	int ret = 0;
	float distance_m = -1.0f;

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			/* only throw an error if we time out */
			if (read_elapsed > (_interval * 2)) {
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}

		}

		_last_read = hrt_absolute_time();

		for ( int i = 0; i < ret; i++ ) {
			//TODO: need to check overflow
			SP25_parser(readbuf[i],_linebuf, &_linebuf_index,  &_parse_state, &distance_m);
		}
		bytes_available -= ret;
	} while (bytes_available > 0);

	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	//message publish
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void SP25::start()
{
	ScheduleOnInterval(_interval);
}

void SP25::stop()
{
	ScheduleClear();
}

void SP25::Run()
{

	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}
	}

	if (collect() == -EAGAIN) {
		/* reschedule to grab the missing bits, time to transmit 14 bytes @ 115200 bps */
		ScheduleClear();
		ScheduleDelayed(_interval);

		return;
	}
}

void SP25::print_info()
{
	PX4_INFO("Using port: %s\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/**
 * @brief SP25 message decoder
 *
 * @param sp25_pack data handle structure
 * @param byte received data
 * @return int 0 if success, otherwise -1
 */
int SP25::SP25_parser(unsigned char c, char *parserbuf, unsigned *parserbuf_index, SP25_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	static uint8_t msg_upper = 0, msg_lower = 0; //msg id low and high bytes
	static uint8_t rag_upper = 0, rag_lower = 0; //range high and low bytes
        static uint8_t vel_upper = 0, vel_lower = 0; //target velocity high and low 3 bits
	static uint16_t msg_info = 0, surface_info = 0, range_info = 0;
	static uint8_t is_target_sensor_update = false;
	float velocity_info;

	switch (*state) {
	case SP25_PARSE_STATE::START_SEQ: //2 bytes in total
		if (*parserbuf_index == 0)
		{
			if (c == SP25_HEADER) {
				*parserbuf_index = 1;
				break;
			}
		}
		else if (*parserbuf_index == 1)
		{
			if (c == SP25_HEADER)
			{
				*parserbuf_index = 0;
				*state = SP25_PARSE_STATE::MSG_ID;
			}
		}
		break;

	case SP25_PARSE_STATE::MSG_ID: //2 bytes in total
		if (*parserbuf_index == 0)
		{
			msg_upper = c;
			*parserbuf_index = 1;
			break;
		}
		else if (*parserbuf_index == 1)
		{
			msg_lower = c;
			*parserbuf_index = 0;
			msg_info = msg_upper + msg_lower * 256;
			if (msg_info == SP25_SENSOR_STATUS)
			{
				*state = SP25_PARSE_STATE::MSG_SENSOR_STATUS;
			}
			else if (msg_info == SP25_TARGET_STATUS)
			{
				*state = SP25_PARSE_STATE::MSG_TARGET_STATUS;
			}
			else if (msg_info == SP25_TARGET_INFO)
			{
				*state = SP25_PARSE_STATE::MSG_TARGET_INFO;
			}
			else
			{
				*state = SP25_PARSE_STATE::START_SEQ;
			}
		}
		break;

	case SP25_PARSE_STATE::MSG_SENSOR_STATUS: //可透過 status 得知 sp25 是否還活著
		(*parserbuf_index)++;
		if (*parserbuf_index == SP25_PACKET_NUM)
		{
			if(is_target_sensor_update)
			{
				*dist = 30.0f;
			}
			*state = SP25_PARSE_STATE::END_SEQ;
			*parserbuf_index = 0;
			is_target_sensor_update = true;
		}
		break;

	case SP25_PARSE_STATE::MSG_TARGET_STATUS:
		(*parserbuf_index)++;
		if (*parserbuf_index == 1) //本次測量到多少物體(目前都只能偵測到一個)
		{ }
		else if (*parserbuf_index == SP25_PACKET_NUM)
		{
			*state = SP25_PARSE_STATE::END_SEQ;
			*parserbuf_index = 0;
		}
		break;

	case SP25_PARSE_STATE::MSG_TARGET_INFO:
		(*parserbuf_index)++;
		if (*parserbuf_index == 1) //目標ID
		{ }
		else if (*parserbuf_index == 2) //反射物體截面積(upper)
		{
			surface_info = c * 0.5 - 50;
			sp25_pack.size = surface_info;
		}
		else if (*parserbuf_index == 3) //反射距離(upper)
		{
			rag_upper = c;
		}
		else if (*parserbuf_index == 4) //反射距離(lower)
		{
			rag_lower = c;
			range_info = (rag_upper << 8) | rag_lower;
			*dist = range_info * 0.01;
		}
		else if (*parserbuf_index == 5) //reserved
		{
		}
		else if (*parserbuf_index == 6) //目標速度3b(upper)
		{
			vel_upper = (c & 0xE0) >> 5;
		}
		else if (*parserbuf_index == 7) //目標速度(lower)
		{
			vel_lower = c;
			velocity_info = (vel_upper * 256 + vel_lower) * 0.05 - 35;
			sp25_pack.vel = velocity_info;
		}
		else if (*parserbuf_index == SP25_PACKET_NUM) //信噪比
		{
			is_target_sensor_update = false;
			sp25_pack.snr = c - 127;
			*state = SP25_PARSE_STATE::END_SEQ;
			*parserbuf_index = 0;
		}
		break;

	case SP25_PARSE_STATE::END_SEQ:
		if (*parserbuf_index == 0)
		{
			if (c == SP25_END)
			{
				*parserbuf_index = 1;
			}
			else
			{
				*parserbuf_index = 0;
				*state = SP25_PARSE_STATE::START_SEQ;
				sp25_pack.pack_type = pack_none;
			}
			break;
		}
		else if (*parserbuf_index == 1)
		{
			if (c == SP25_END)
			{
				//is_sp25_ready = true;
				if(sp25_pack.pack_type == pack_info)
				{
					ret = 0;
				}
				sp25_pack.pack_type = pack_none;
			}
			else
			{
				sp25_pack.pack_type = pack_none;
			}
			*parserbuf_index = 0;
			*state = SP25_PARSE_STATE::START_SEQ;
		}
		break;
	}
	return ret;
}
