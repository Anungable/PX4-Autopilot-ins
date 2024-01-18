/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
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

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>

static sp25_package sp25_pack {.range = 0.0f, .vel = 0.0f, .size = 0, .snr = 0, .pack_type = 0};

SP25::SP25(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	//* store port name */
	strncpy(_port, port, sizeof(_port) - 1); //copy port to _port

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_SP25);
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
	_interval = 20000; //50 Hz //TODO: double check update frequency
	start();

	return PX4_OK;
}


int SP25::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen(緩衝區長度) minus null termination */
	unsigned char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	int ret = 0;
	float distance_m = -1.0f;

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		PX4_INFO("cannot run parser\n");
		return 0;
	}

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	bool valid = false;
	do {
		ret = ::read(_fd, &readbuf[0], readlen);
		if (ret < 0) {
			PX4_DEBUG("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			/* only throw an error if we time out */
			if (read_elapsed > (_interval * 2)) {
				return ret;

			} else {
				return -EAGAIN;
			}

		} else if (ret == 0) {
			return -EAGAIN;
		}

		_last_read = hrt_absolute_time();

		for ( int i = 0; i < ret; i++ ) {
			//TODO: need to check overflow
			if (OK == SP25_parser(readbuf[i],_linebuf, &_linebuf_index)) {
				valid = true;
				PX4_INFO("run parser\n");
			}
		}
		bytes_available -= ret;
	} while (bytes_available > 0);


	if (!valid) {
		return -EAGAIN;
	}
	distance_m = sp25_pack.range;
	int8_t signal_quality = -1;

	//message publish
	_px4_rangefinder.update(timestamp_sample, distance_m,signal_quality);

	perf_end(_sample_perf);

	return PX4_OK;
}

void SP25::start()
{
	ScheduleNow();
}

void SP25::stop()
{
	ScheduleClear();
}

void SP25::Run()
{

	if (_fd > 0) {
		// PX4_INFO("serial port already open");
		return;
	}

	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config); //to access the termios structure

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR; //turn off the conversion of newline characters to carriage return and newline \r\n

		/* no parity, no hardware flow control */ // TODO: check if only 1 stop bit
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


	/* perform collection */
	int collect_ret = collect();

	if (collect_ret == -EAGAIN) {
		/* reschedule to grab the missing bits, time to transmit 14 bytes @ 115200 bps */
		ScheduleDelayed(87 * 14);

		return;
	}
	/* error count */
	// if (OK != collect_ret) {

	// 	/* we know the sensor needs about four seconds to initialize */
	// 	if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
	// 		PX4_ERR("collection error #%u", _consecutive_fail_count);
	// 	}

	// 	_consecutive_fail_count++;

	// 	/* restart the measurement state machine */
	// 	start();
	// 	return;

	// } else {
	// 	/* apparently success */
	// 	_consecutive_fail_count = 0;
	// }

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void SP25::print_info()
{
	PX4_INFO("Using port: %s\n", _port);
	PX4_INFO("range=%f\n",(double) sp25_pack.range);
	PX4_INFO("signal noise ratio=%f\n",(double) sp25_pack.snr);
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
int SP25::SP25_parser(unsigned char c, char *parserbuf, unsigned *parserbuf_index)
{
	int ret = -1;
	static uint8_t count = 0;		     //iterate through bits
	static uint8_t msg_upper = 0, msg_lower = 0; //msg id low and high bytes
	static uint8_t rag_upper = 0, rag_lower = 0; //range high and low bytes
        static uint8_t vel_upper = 0, vel_lower = 0; //target velocity high and low 3 bits
	static uint16_t msg_info = 0, surface_info = 0, range_info = 0;
	static uint8_t is_target_sensor_update = false;
	float velocity_info;

	static SP25_PARSE_STATE state = START_SEQ;

	switch (state) {
	case START_SEQ: //2 bytes in total
		if (count == 0)
		{
			if (c == SP25_HEADER) {
				count = 1;
				break;
			}
		}
		else if (count == 1)
		{
			if (c == SP25_HEADER)
			{
				count = 0;
				state = MSG_ID;
			}
		}
		break;

	case MSG_ID: //2 bytes in total
		if (count == 0)
		{
			msg_upper = c;
			count = 1;
			break;
		}
		else if (count == 1)
		{
			msg_lower = c;
			count = 0;
			msg_info = msg_upper + msg_lower * 256;
			if (msg_info == SP25_SENSOR_STATUS)
			{
				state = MSG_SENSOR_STATUS;
			}
			else if (msg_info == SP25_TARGET_STATUS)
			{
				state = MSG_TARGET_STATUS;
			}
			else if (msg_info == SP25_TARGET_INFO)
			{
				state = MSG_TARGET_INFO;
			}
			else
			{
				state = START_SEQ;
			}
		}
		break;

	case MSG_SENSOR_STATUS: //可透過 status 得知 sp25 是否還活著
		count++;
		if (count == SP25_PACKET_NUM)
		{
			if(is_target_sensor_update)
			{
				sp25_pack.range = 30.0f;
			}
			state = END_SEQ;
			count = 0;
			is_target_sensor_update = true;
		}
		break;

	case MSG_TARGET_STATUS:
		count++;
		if (count == 1) //本次測量到多少物體(目前都只能偵測到一個)
		{ }
		else if (count == SP25_PACKET_NUM)
		{
			state = END_SEQ;
			count = 0;
		}
		break;

	case MSG_TARGET_INFO:
		count++;
		if (count == 1) //目標ID
		{ }
		else if (count == 2) //反射物體截面積(upper)
		{
			surface_info = c * 0.5 - 50;
			sp25_pack.size = surface_info;
		}
		else if (count == 3) //反射距離(upper)
		{
			rag_upper = c;
		}
		else if (count == 4) //反射距離(lower)
		{
			rag_lower = c;
			range_info = (rag_upper << 8) | rag_lower;
			sp25_pack.range = range_info * 0.01;
		}
		else if (count == 5) //reserved
		{
		}
		else if (count == 6) //目標速度3b(upper)
		{
			vel_upper = (c & 0xE0) >> 5;
		}
		else if (count == 7) //目標速度(lower)
		{
			vel_lower = c;
			velocity_info = (vel_upper * 256 + vel_lower) * 0.05 - 35;
			sp25_pack.vel = velocity_info;
		}
		else if (count == SP25_PACKET_NUM) //信噪比
		{
			is_target_sensor_update = false;
			sp25_pack.snr = c - 127;
			state = END_SEQ;
			count = 0;
		}
		break;

	case END_SEQ:
		if (count == 0)
		{
			if (c == SP25_END)
			{
				count = 1;
			}
			else
			{
				count = 0;
				state = START_SEQ;
				sp25_pack.pack_type = pack_none;
			}
			break;
		}
		else if (count == 1)
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
			count = 0;
			state = START_SEQ;
		}
		break;
	}
	return ret;
}
