#ifndef PARSE_HELPERS_H
#define PARSE_HELPERS_H

#include <stdint.h>
#include <string>

void next_token(char **buf, uint8_t &val);
void next_token(char **buf, uint16_t &val);
void next_token(char **buf, uint32_t &val);
void next_token(char **buf, int32_t &val);
void next_token(char **buf, int16_t &val);
void next_token(char **buf, float &val);
void next_token(char **buf, std::string &val);
void next_token(char **buf);

#endif // PARSE_HELPERS_H
