#ifndef PARSE_HELPERS_H
#define PARSE_HELPERS_H

#include <stdint.h>
#include <string>

/**
 * @brief Parse token of provided type and advance the buffer accordingly
 *
 * This is is not necessarily safe and may step over the end of the buffer.
 * @param buf pointer to the input buffer
 * @param val value to extract into
 */
void nextToken(char **buf, uint8_t &val);
void nextToken(char **buf, uint16_t &val);
void nextToken(char **buf, uint32_t &val);
void nextToken(char **buf, int32_t &val);
void nextToken(char **buf, int16_t &val);
void nextToken(char **buf, float &val);
void nextToken(char **buf, std::string &val);
void nextToken(char **buf);

#endif // PARSE_HELPERS_H
