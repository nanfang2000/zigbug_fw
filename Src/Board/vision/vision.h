#ifndef __VISION_H
#define __VISION_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

void vision_init(void);

void vision_play(const uint8_t * p_audio_data, uint16_t audio_length);

void vision_stop(void);
	
#ifdef  __cplusplus
}  
#endif

#endif
