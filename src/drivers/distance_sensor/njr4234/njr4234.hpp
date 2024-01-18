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

/**
 * @file njr4234.cpp
 * @author Inung Shen <inung.shen@coretronic-robotics.com>
 *
 * Driver for the NJR4234 microwave distance sensor
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#define NJR4234_DEFAULT_PORT	"/dev/ttyS3" //set to telem2

#define OUTDATA_HEADER1_MEAS_DIST   0x21
#define OUTDATA_HEADER2_STATIONARY  0x00
#define OUTDATA_HEADER2_MOVING      0x02
#define OUTDATA_HEADER3             0x00
#define OUTDATA_HEADER4             0x00
#define RUNCMD_HEADER1              0xE0    //response of run command
#define READ_ALL_PARAM_HEADER1      0x23    //response for read default and all param H1
#define READ_ALL_PARAM_HEADER2      0x17    //response for read default and all param H2

using namespace time_literals;

enum njr234_package {
	PREAMBLE = 0,
	HEADER,
	COMMAND,
	ALL_PARAM,
	STATIONARY_OBJ, //output data
	MOVING_OBJ,     //output data
	END_SEQ
};

enum NJR4234_PARSE_STATE {
	njr234_package decode_state;
	uint8_t header_msg;
	uint8_t data_buf[8];
};

struct njr4234_pack_s
{
    float range;
    float vel;
    uint8_t size;
    uint8_t snr;
    uint8_t pack_type;
    uint8_t cmd1, cmd2, cmd3;
    NJR4234_PARSE_STATE parser;
};

class NJR4234 : public px4::ScheduledWorkItem
{
public:
	NJR4234(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING);
	virtual ~NJR4234();

	int 				init();
	void 				print_info();

private:

	int  				collect();

	void 				Run() override;

	void 				start();
	void 				stop();
	int  NJR4234_parser(unsigned char c, char *parserbuf, unsigned *parserbuf_index, NJR4234_PARSE_STATE *state, float *dist);

	PX4Rangefinder			_px4_rangefinder;

	char 				_linebuf[10] {};
	char 				_port[20] {};

	int				_fd{-1};
	char				_linebuf[8] {};
	unsigned int			_linebuf_index {0};
	NJR4234_PARSE_STATE		_parse_state {};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
