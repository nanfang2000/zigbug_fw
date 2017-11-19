#ifndef __VISION_H
#define __VISION_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

typedef enum 
{
    VISION_FRONT,
    VISION_UPSIDE,
    VISION_LEFT_EYE,
    VISION_RIGHT_EYE,
    VISION_LEFT_SIDE,
    VISION_RIGHT_SIDE,
    VISION_BACK
}Vision_Orientation;

void vision_init(void);

void vision_play(const uint8_t * p_audio_data, uint16_t audio_length);

void vision_stop(void);
void vision_start_measure(Vision_Orientation orientation);
int16_t vision_get_measure(Vision_Orientation orientation);
	
#ifdef  __cplusplus
}  
#endif

#endif
