#include "utils/string.h"

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

void *memcpy (void *dest, const void *src, uint32_t len) {

	char *d = dest;
	const char *s = src;

	while (len--) *d++ = *s++;
	return dest;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void *memset (void *dest, int val, uint32_t len) {

	unsigned char *ptr = dest;

	while (len-- > 0) *ptr++ = val;
	return dest;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// returns a length of a null-terminated string
uint32_t strlen(char *str) {

    uint32_t size = 0;

    while(*str++ != '\0') size++;

    return size;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// converts an integer to string
int itoa(int num, char* str, int len, int base) {

	int sum = num;
	int i = 0;
	int digit;

	if (len == 0) return -1;

	do {

		digit = sum % base;

		if (digit < 0xA) str[i++] = '0' + digit;
		else str[i++] = 'A' + digit - 0xA;

		sum /= base;

	} while (sum && (i < (len - 1)));

	if (i == (len - 1) && sum) return -1;

	str[i] = '\0';
	strrev(str);

	return 0;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// converts a string to integer
int atoi(char* str) {
	
    int res = 0;
    for (int i = 0; str[i] != '\0'; ++i) res = res * 10 + str[i] - '0';
    return res;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// reverses a string
void strrev(char *str) {

	int i;
	int j;
	unsigned char a;
	unsigned len = strlen((char *)str);

	for (i = 0, j = len - 1; i < j; i++, j--) {

		a = str[i];
		str[i] = str[j];
		str[j] = a;
	}
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// compares two strings; if the strings are equal, a 0 is returned
int strcmp(const char *s1, const char *s2) {
	
	while (*s1 == *s2++)
		if (*s1++ == '\0')
			return (0);
			
	return (*(const unsigned char *)s1 - *(const unsigned char *)(s2 - 1));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
