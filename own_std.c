#include <inttypes.h>

int o_strnlen(const char* str, int n)
{
	int i = 0;
	while(*(str++) != 0)
		if(++i == n) break;
	return i;
}

int o_pow(int b, int e)
{
	if(e<1) return 1;
	while(--e)
	{
		b*=b;
	}
	return b;
}

// o_itoa, o_utoa and o_str_append allow easy cstring construction
// by returning a pointer to where the generated new string ends at.
// Terminating zero is always added by the functions, and the
// returning value points to that place, so it's easily overwritten
// by more text. For example:
// uint16_t value = 420;
// char buffer[100];
// char* p_buf = buffer;
// p_buf = o_str_append(p_buf, "value is ");
// p_buf = o_utoa16(value, p_buf);
// p_buf = o_str_append(p_buf, "at the moment.\n");
// puts(buffer);

char* o_utoa16(uint16_t val, char* str)
{
	static const uint16_t div[5] = {10000,1000,100,10,1};
	int outp_ena = 0;
	int i;
	for(i = 0; i < 5; i++)
	{
		char c = val/div[i];
		val -= div[i]*c;
		c += '0';

		if(c != '0' || i == 4)
			outp_ena = 1;
		if(outp_ena)
			*str++ = c;
	}
	*str = 0;
	return str;
}

char* o_utoa32(uint32_t val, char* str)
{
	static const uint32_t div[10] = {1000000000UL,100000000UL,10000000UL,1000000UL,100000UL,10000UL,1000UL,100UL,10UL,1UL};
	int outp_ena = 0;
	int i;
	for(i = 0; i < 10; i++)
	{
		char c = val/div[i];
		val -= div[i]*c;
		c += '0';

		if(c != '0' || i == 9)
			outp_ena = 1;
		if(outp_ena)
			*str++ = c;
	}
	*str = 0;
	return str;
}

char* o_itoa16(int16_t val, char* str)
{
	if(val < 0)
	{
		*str++ = '-';
		val *= -1;
	}

	return o_utoa16(val, str);
}

char* o_itoa32(int32_t val, char* str)
{
	if(val < 0)
	{
		*str++ = '-';
		val *= -1;
	}

	return o_utoa32(val, str);
}

char* o_str_append(char* str1, char* str2)
{
	while(*str2)
		*str1++ = *str2++;
	*str1 = 0;
	return str1;
}

char* o_str_cmp(char* str1, char* str2)
{
	while(*str2 != 0)
	{
		if(*str1 != *str2)
			return 0;
		str1++;
		str2++;
	}

	return str1;
}

// o_atoi_append searches for the next actual number.
// o_atoi_append returns the pointer to the very next
// location after the number, so you can convert series
// of stuff like this:
// char message[100] = "The number 420 is more popular now than 42 ever was."
// char* p_msg = message;
// int first, second;
// p_msg = o_atoi_append(p_msg, &first); // first = 420
// p_msg = o_atoi_append(p_msg, &second); // second = 42
// puts(p_msg); // prints " ever was."

char* o_atoi_append(char* str, int* val_out)
{
	char* first;
	char* last;
	char* pnt;
	int val;
	int multiplier;

	first = str;
	while(*first < '0' || *first > '9')
	{
		if(first == 0)
			return first;
		first++;
	}

	last = first;
	while(*last >= '0' && *last <= '9')
		last++;

	if(first == last) return 0;

	pnt = last-1;
	multiplier = 1;
	val = 0;

	while(pnt >= first)
	{
		val += ((*pnt)-'0')*multiplier;
		multiplier *= 10;
		pnt--;
	}
	if(pnt >= str && *pnt == '-')
		val *= -1;

	*val_out = val;
	return last;
}
