#ifndef FX_FXTYPE_H_
#define FX_FXTYPE_H_

#ifdef _WIN32
#define FX_VOID void
#define FX_CSTR const char *
#define FX_BOOL unsigned char
#define FX_TRUE 1
#define FX_FALSE 0
#define FX_CHAR char
#define FX_UCHAR unsigned char
#define FX_INT8 char
#define FX_INT16 short
#define FX_INT32 int
#define FX_INT32L long
#define FX_INT64 long long
#define FX_UINT8 unsigned char
#define FX_UINT16 unsigned short
#define FX_UINT32 unsigned int
#define FX_UINT32L unsigned long
#define FX_UINT64 unsigned long long
#define FX_FLOAT float
#define FX_DOUBLE double

#elif defined(__linux__)
#define FX_VOID void
#define FX_CSTR const char *
#define FX_BOOL unsigned char
#define FX_TRUE 1
#define FX_FALSE 0
#define FX_CHAR char
#define FX_UCHAR unsigned char
#define FX_INT8 char
#define FX_INT16 short
#define FX_INT32 int
#define FX_INT64 long long
#define FX_UINT8 unsigned char
#define FX_UINT16 unsigned short
#define FX_UINT32 unsigned int
#define FX_UINT64 unsigned long long
#define FX_FLOAT float
#define FX_DOUBLE double
#if defined(__x86_64__) || defined(__aarch64__) || defined(__LP64__)
#define FX_INT32L int
#define FX_UINT32L unsigned int
#else
#define FX_INT32L long
#define FX_UINT32L unsigned long
#endif

#else
#error "Unsupported platform! Only Windows and Linux are supported."
#endif

#endif /* FX_FXTYPE_H_ */