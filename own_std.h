#ifndef __OWN_STD_H
#define __OWN_STD_H

#include <inttypes.h>

// Find terminating 0 from a string, or terminate after n characters,
// whichever comes first. Returns the length of a null-terminated string,
// not including the null character, or at most n.
int o_strnlen(const char* str, int n);
int o_pow(int b, int e);
char* o_utoa16(uint16_t val, char* str);
char* o_utoa32(uint32_t val, char* str);
char* o_itoa16(int16_t val, char* str);
char* o_itoa32(int32_t val, char* str);
char* o_str_append(char* str1, char* str2);
char* o_str_cmp(char* str1, char* str2);
char* o_atoi_append(char* str, int* val_out);



#endif
