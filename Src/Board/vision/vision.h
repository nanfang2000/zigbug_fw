#ifndef __VISION_H__
#define __VISION_H__
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

#define    VISION_FRONT        (1 << 0)
#define    VISION_UPSIDE       (1 << 1)
#define    VISION_LEFT_EYE     (1 << 2)
#define    VISION_RIGHT_EYE    (1 << 3)
#define    VISION_LEFT_SIDE    (1 << 4)
#define    VISION_RIGHT_SIDE   (1 << 5)
#define    VISION_BACK         (1 << 6)
#define    VISION_ALL          (VISION_FRONT | VISION_UPSIDE | VISION_LEFT_EYE | VISION_RIGHT_EYE | VISION_LEFT_SIDE | VISION_RIGHT_SIDE | VISION_BACK)


typedef struct
{
    int16_t dist_front;
    int16_t dist_upside;
    int16_t dist_left_eye;
    int16_t dist_right_eye;
    int16_t dist_left_side;
    int16_t dist_right_side;
    int16_t dist_back;
}vision_t;

void vision_init(void);
void vision_stop(void);

void vision_start_measure(int32_t orientations);
void vision_get_measure(int32_t orientations, vision_t *vision);
	
#ifdef  __cplusplus
}  
#endif

#endif
