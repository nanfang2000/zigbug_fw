#ifndef __MIC_H
#define __MIC_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

void mic_init(void);
void mic_start(void);
void mic_stop(void);
void mic_listen(uint8_t channel,uint16_t *rx_buffer, uint32_t micro_seconds);

#ifdef  __cplusplus
}  
#endif

#endif
