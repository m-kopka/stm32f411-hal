#ifndef _UTILS_STRING_H_
#define _UTILS_STRING_H_

#include "stdint.h"

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// returns a length of a null-terminated string
uint32_t strlen(char *str);

// converts an integer to string
int itoa(int num, char* str, int len, int base);

// converts a string to integer
int atoi(char* str);

// reverses a string
void strrev(char *str);

// compares two strings; if the strings are equal, a 0 is returned
int strcmp(const char *s1, const char *s2);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _UTILS_STRING_H_ */