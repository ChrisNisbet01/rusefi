/**
 * @file	efilib.h
 *
 * @date Feb 21, 2014
 * @author Andrey Belomutskiy, (c) 2012-2017
 */

#ifndef EFILIB_H_
#define EFILIB_H_

#include <stdint.h>
#include "global.h"

#define _MAX_FILLER 11

// http://en.wikipedia.org/wiki/Endianness

#define SWAP_UINT16(x) ((x) << 8) | ((x) >> 8)

#define SWAP_UINT32(x) (((x) >> 24) & 0xff) | (((x) << 8) & 0xff0000) | (((x) >> 8) & 0xff00) | (((x) << 24) & 0xff000000)

// human-readable IDs start from 1 while computer-readbale indexes start from 0
#define ID2INDEX(id) ((id) - 1)

// number of milliseconds in one period of given frequency (per second)
#define frequency2periodMs(freq) ((1000.0f) / (freq))

// number of microseconds in one period of given frequency (per second)
#define frequency2periodUs(freq) ((1000000.0f) / (freq))

#define ERROR_CODE 311223344

const char * boolToString(bool value);

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

char * efiTrim(char *param);
uint32_t efiStrlen(const char *param);
int efiPow10(int param);
bool startsWith(const char *line, const char *prefix);
int indexOf(const char *string, char ch);
float atoff(const char *string);
int atoi(const char *string);

#if defined(USE_HOMEGROWN_NAN)
#define cisnan(f) (*(((int*) (&f))) == 0x7FC00000)
#else
#define cisnan(f) ((f) == NAN)
#endif

#define UNUSED(x) (void)(x)
  
int absI(int32_t value);
float absF(float value);
float efiRound(float value, float precision);
int maxI(int i1, int i2);
int minI(int i1, int i2);
float maxF(float i1, float i2);
float minF(float i1, float i2);
char* itoa10(char *p, int num);
bool isSameF(float v1, float v2);

bool strEqualCaseInsensitive(const char *str1, const char *str2);
bool strEqual(const char *str1, const char *str2);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* EFILIB_H_ */
