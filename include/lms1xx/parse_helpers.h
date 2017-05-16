/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
