#ifndef __AUDIO_H
#define __AUDIO_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

void audio_init(void);

void audio_play(const uint8_t * p_audio_data, uint16_t audio_length);

void audio_stop(void);
	
#ifdef  __cplusplus
}  
#endif

#endif
