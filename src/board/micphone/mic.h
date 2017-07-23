#ifndef __MIC_H
#define __MIC_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

void mic_init(void);
void mic_start(void);
void mic_stop(void);

#ifdef  __cplusplus
}  
#endif

#endif
