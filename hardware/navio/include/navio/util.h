
#ifndef _UTIL_H_
#define _UTIL_H_

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define NAVIO2 3
#define NAVIO 1

int WriteFile(const char *path, const char *fmt, ...);
int ReadFile(const char *path, const char *fmt, ...);
bool CheckApm();
int GetNavioVersion();

#endif  // _UTIL_H_