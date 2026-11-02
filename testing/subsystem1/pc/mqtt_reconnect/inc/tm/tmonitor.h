#ifndef FAKE_TM_H
#define FAKE_TM_H
typedef unsigned char UB;
int tm_printf(const UB *fmt, ...);
int tm_sprintf(UB *str, const UB *fmt, ...);
#endif
