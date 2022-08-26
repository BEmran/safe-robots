
#ifndef _UTIL_H_
#define _UTIL_H_

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

namespace navio 
{  
constexpr const char* MPU_SPI_PATH = "/dev/spidev0.1";
constexpr auto NAVIO = 1;
constexpr auto NAVIO2 = 3;

int WriteFile(const char *path, const char *fmt, ...);

int ReadFile(const char *path, const char *fmt, ...);

bool CheckApm();

int GetNavioVersion();

}  // namespace navio
#endif  // _UTIL_H_