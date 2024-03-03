#ifndef _UTILS_STRING_H_
#define _UTILS_STRING_H_

#include "stdint.h"

uint32_t strlen(char *str);
int itoa(int num, char* str, int len, int base);
int atoi(char* str);
void strrev(char *str);
int strcmp(const char *s1, const char *s2);

#endif /* _UTILS_STRING_H_ */