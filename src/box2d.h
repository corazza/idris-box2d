#ifndef BOX2DA_H__
#define BOX2DA_H__

#include <idris_rts.h>

#ifdef __cplusplus
extern "C" {
#endif

int hello();

void *createWorld(double x, double y);
void destroyWorld(void *world);

#ifdef __cplusplus
}
#endif

#endif
