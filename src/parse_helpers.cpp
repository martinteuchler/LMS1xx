#include "LMS1xx/parse_helpers.h"

#include <cstring>

void nextToken(char **buf, uint8_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%hhx", &val);
  *buf += strlen(str) + 1;
}

void nextToken(char **buf, uint16_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%hx", &val);
  *buf += strlen(str) + 1;
}

void nextToken(char **buf, uint32_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%x", &val);
  *buf += strlen(str) + 1;
}

void nextToken(char **buf, int32_t &val)
{
  uint32_t temp;
  nextToken(buf, temp);
  val = temp;
}

void nextToken(char **buf, int16_t &val)
{
  uint16_t temp;
  nextToken(buf, temp);
  val = temp;
}

void nextToken(char **buf, float &val)
{
  char *str = strtok(*buf, " ");
  val = *reinterpret_cast<float *>(str);
  *buf += strlen(str) + 1;
}

void nextToken(char **buf, std::string &val)
{
  char *str = strtok(*buf, " ");
  val = std::string(str);
  *buf += strlen(str) + 1;
}

void nextToken(char **buf)
{
  strtok(*buf, " ");
  *buf += strlen(*buf) + 1;
}
