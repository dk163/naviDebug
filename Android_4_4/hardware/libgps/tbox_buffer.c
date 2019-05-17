#include "tbox_gps.h"
#include "protocolnmea.h"
#include "tbox_buffer.h"

//--------------------------------------------------------------------------------
void tbox_gps_buffer_init(GpsState* state)
{		
    state->raw_used = 0;
	state->one_frame_count = 0;
}

//--------------------------------------------------------------------------------
void tbox_gps_buffer_deinit(GpsState* state)
{
	state->raw_used = 0;
	state->one_frame_count = 0;
}

int tbox_gps_buffer_get_frame_count(GpsState* state)
{
	return state->one_frame_count;
}

char* tbox_gps_buffer_get_frame(GpsState* state, int index)
{
	if (index < state->one_frame_count) {
		return state->one_frame_nmea[index];
	}
	
	return NULL;
}

void tbox_gps_buffer_append(GpsState* state, char *buffer, int size)
{
	if ((state->raw_used + size) >= GPS_MSG_RXBUF_SIZE)
	{
		ALOGE("gps tbox_gps_buffer_append: buffer is overload, clear buffer!");
		memset(state->rawbuf, 0, GPS_MSG_RXBUF_SIZE);
		state->raw_used = 0;
	}

	memcpy(state->rawbuf + state->raw_used, buffer, size);
	state->raw_used += size;
}

void tbox_gps_buffer_remove(GpsState* state, int size)
{
	//ALOGE("gps size - %d , %d ---OK", size, state->raw_used);

	if (state->raw_used > size)
	{
		state->raw_used -= size;
		memmove(state->rawbuf, state->rawbuf + size, state->raw_used);
	}
	else
	{
		state->raw_used = 0;
	}
}

int tbox_gps_buffer_frame_checkout(GpsState* state)
{
	int i;
	int isExistRMC = 0;
	int isExistGLL = 0;	

	state->one_frame_count = 0;

//	ALOGE("gps state->rawbuf[%d]:%s", state->raw_used, state->rawbuf);
	
	for (i = 0; i < state->raw_used; i++)
	{
		int iTemp = NmeaParse(state->rawbuf + i, state->raw_used - i);

		if (iTemp > 0)
        {
        	char *pBuf = state->rawbuf + i;
			
			if ('R' == *(pBuf + 3) && 'M' == *(pBuf + 4) && 'C' == *(pBuf + 5))
			{
				isExistRMC = 1;
				//ALOGE("gps isExistRMC");
			}

			if (isExistRMC != 0)
			{
				if ('G' == *(pBuf + 3) && 'L' == *(pBuf + 4) && 'L' == *(pBuf + 5))
				{
					isExistGLL = 1;
					//ALOGE("gps isExistGLL");
				}

				if (state->one_frame_count < GPS_MSG_FIELD_SIZE)
				{
					memset(state->one_frame_nmea[state->one_frame_count], 0, GPS_NMEA_MAX_SIZE);
					memcpy(state->one_frame_nmea[state->one_frame_count], pBuf, iTemp);
					state->one_frame_count++;
				}
				else
				{
					ALOGE("[ERROR]: gps state->one_frame_count = %d", state->one_frame_count);
				}
			}
        }

		if (isExistRMC != 0 && isExistGLL != 0)
		{
			break;
		}
	}

	if (isExistRMC != 0 && isExistGLL != 0)
	{
		//ALOGE("gps ==============found frame");
		tbox_gps_buffer_remove(state, i);
		return 1;
	}

	return 0;
}
