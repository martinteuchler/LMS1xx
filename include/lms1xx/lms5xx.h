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

#ifndef LMS5XX_H
#define LMS5XX_H

#include <stdint.h>
#include <string>
#include <lms1xx/colaa.h>
#include <lms1xx/colaa_structs.h>

/**
 * @brief Specialises CoLaA base implementation for the LMS5xx series of scanners
 */
class LMS5xx : public CoLaA
{
public:

protected:
  /**
   * @brief Channel data is handled by set_echo_filter for this sensor
   * @param ch
   * @return
   */
  std::string buildScanDataCfgOutputChannel(int ch) const;
};

#endif // LMS5XX_H
