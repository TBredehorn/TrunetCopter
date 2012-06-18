#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "gps_mtk.h"
#include "../main.h"

extern gps_data_t gps_data;

float cosine (float x) {
	return sine(x+M_PI/2);
}

float sine(float x) {
	const float B = 4/M_PI;
	const float C = -4/(M_PI*M_PI);
	while (x > M_PI) x -= 2*M_PI;
	while (x < -M_PI) x += 2*M_PI;
	float y = B * x + C * x * fabs(x);
	const float P = 0.225;
	y = P * (y * fabs(y) - y) + y;
	return y;
}

void swap_endian_long (uint8_t *x) {
  uint8_t c;
  c = x[0];
  x[0] = x[3];
  x[3] = c;
  c = x[1];
  x[1] = x[2];
  x[2] = c;
}

void parse_gps_sentence (char *s, gps_data_t *data)
{
  uint32_t tmp;
  uint32_t time_tmp;
  uint16_t header_msg_id;
  uint8_t ck_a=0, ck_b=0;
  int8_t i;

  for (i=2; i<35; ++i) {
    ck_a = ck_a + s[i];
    ck_b = ck_b + ck_a;
  }

  memcpy(&header_msg_id, &s[0], 2);
  if (header_msg_id == 0xddd0 && ck_a == s[35] && ck_b == s[36]) {
    memcpy(&data->latitude, &s[3], 4);

    memcpy(&data->longitude, &s[7], 4);

    memcpy(&tmp, &s[11], 4);
    data->altitude = (float)tmp/100.0;

    memcpy(&tmp, &s[15], 4);
    data->speed = (float)tmp/100.0;

	memcpy(&tmp, &s[19], 4);
	//data->heading = ((float)tmp/1.0e6)*M_PI/180.0;
	data->heading = (float)tmp/100.0;

	memcpy(&data->satellites, &s[23], 1);

	memcpy(&data->fix, &s[24], 1);

	memcpy(&data->utc_date, &s[25], 4);

	// time from gps is UTC, but we want milliseconds from epoch
	memcpy(&data->utc_time, &s[29], 4);
	time_tmp = data->utc_time;
	tmp = (time_tmp/10000000);
	time_tmp -= tmp*10000000;
	data->time = tmp * 3600000;
	tmp = (time_tmp/100000);
	time_tmp -= tmp*100000;
	data->time += tmp * 60000 + time_tmp;

	memcpy(&data->hdop, &s[33], 2);

	data->v_lat = cosine(data->heading)*data->speed;

	data->v_lon = sine(data->heading)*data->speed;

    if (data->fix == FIX_3D) {
    	data->valid = 1;
    } else {
    	data->valid = 0;
    }
  }
  else data->valid = 0;
}

/**
 * Polling thread
 */
static WORKING_AREA(PollGPSThreadWA, 512);
static msg_t PollGPSThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollGPS");

	#define NMEA_BUF_SIZE 80
	char nmea_buf[NMEA_BUF_SIZE];
	int ptr=0;
	char c;
	/*
	long target_lat=0, target_lon=0;
	static float I_delta_lat=0, I_delta_lon=0;
	static float last_T;
	float dt;
	*/

	while (TRUE) {
		c = chIOGetTimeout(&SERIAL_GPS, TIME_IMMEDIATE);
		if ((ptr==0 && c==0xd0) || (ptr==1 && c==0xdd) || (ptr > 1 && ptr < 36))
			nmea_buf[ptr++] = c;
		else if (ptr==36) {
			nmea_buf[ptr++] = c;
			nmea_buf[ptr] = 0;
			ptr = 0;
			parse_gps_sentence(nmea_buf, &gps_data);
			/*
			if (!gps_data.valid) gps_data.alt = -1e6;
			if (gps_data.valid) {
				if (AP==AP_GPS && (abs(rc_controls[PIT])<0.2) && abs(rc_controls[ROL])<0.2) { // do position hold calculations
					dt = T - last_T;
					last_T = T;
					if (dt <= 0) dt = 0;

					if (target_lat==0 || target_lon==0) {
						target_lat = gps_data.lat;
						target_lon = gps_data.lon;
					}

					float delta_lat = gps_data.lat - target_lat;
					float delta_lon = gps_data.lon - target_lon;

					I_delta_lat = GPS_I_EXP*I_delta_lat + dt*delta_lat;
					I_delta_lon = GPS_I_EXP*I_delta_lon + dt*delta_lon;

					nav_target_pit = -GPS_P*(delta_lat*m_rot[1] - delta_lon*m_rot[0]);
					nav_target_rol = -GPS_P*(delta_lat*m_rot[0] + delta_lon*m_rot[1]);

					nav_target_pit += -GPS_I*(I_delta_lat*m_rot[1] - I_delta_lon*m_rot[0]);
					nav_target_rol += -GPS_I*(I_delta_lat*m_rot[0] + I_delta_lon*m_rot[1]);

					nav_target_pit += -GPS_D*(gps_data.v_lat*m_rot[1] - gps_data.v_lon*m_rot[0]);
					nav_target_rol += -GPS_D*(gps_data.v_lat*m_rot[0] + gps_data.v_lon*m_rot[1]);

					cap(&nav_target_pit, GPS_PID_CAP);
					cap(&nav_target_rol, GPS_PID_CAP);
				} else {
					target_lat = gps_data.lat;
					target_lon = gps_data.lon;
					I_delta_lat = I_delta_lon = 0;
					if (!AP) nav_target_pit = nav_target_rol = 0;
				}
			} else {
				if (!AP) {
					nav_target_pit = nav_target_rol = 0;
				}
			}
			*/
		}
	}
	return 0;
}

void gps_mtk_start(void) {

	//sdStart(&SERIAL_GPS, &GPS_Serial_Config);
	sdStart(&SERIAL_GPS, NULL);

	chThdSleepMilliseconds(1000);

	// initialize serial port for binary protocol use
	chprintf((BaseChannel *)&SERIAL_GPS, MTK_SET_BINARY);

	// set 4Hz update rate
	chprintf((BaseChannel *)&SERIAL_GPS, MTK_OUTPUT_4HZ);

	chThdCreateStatic(PollGPSThreadWA,
			sizeof(PollGPSThreadWA),
			GPS_THREAD_PRIO,
			PollGPSThread,
			NULL);
}
