#include "LMS1xx/parse_helpers.h"

#include <cstring>

void next_token(char **buf, uint8_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%hhx", &val);
  *buf += strlen(str) + 1;
}

void next_token(char **buf, uint16_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%hx", &val);
  *buf += strlen(str) + 1;
}

void next_token(char **buf, uint32_t &val)
{
  char *str = strtok(*buf, " ");
  sscanf(str, "%x", &val);
  *buf += strlen(str) + 1;
}

void next_token(char **buf, int32_t &val)
{
  uint32_t temp;
  next_token(buf, temp);
  val = temp;
}

void next_token(char **buf, int16_t &val)
{
  uint16_t temp;
  next_token(buf, temp);
  val = temp;
}

void next_token(char **buf, float &val)
{
  char *str = strtok(*buf, " ");
  val = *reinterpret_cast<float *>(str);
  *buf += strlen(str) + 1;
}

void next_token(char **buf, std::string &val)
{
  char *str = strtok(*buf, " ");
  val = std::string(str);
  *buf += strlen(str) + 1;
}

void next_token(char **buf)
{
  strtok(*buf, " ");
  *buf += strlen(*buf) + 1;
}
