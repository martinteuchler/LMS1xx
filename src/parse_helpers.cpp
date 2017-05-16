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

#include "lms1xx/parse_helpers.h"

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
