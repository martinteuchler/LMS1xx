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

#ifndef MRS1000_H
#define MRS1000_H
#include <lms1xx/colaa.h>

struct dataChannel
{
  int channel_nr;
  uint16_t data_len;
  uint16_t dist[1101];
  uint16_t rssi[1101];
};

struct scanDataLayerMRS
{
  bool first;
  int scan_nr;
  uint32_t time_since_startup;
  uint32_t transmission_duration;
  double layer_angle;
  int layer_nr; // starting from 0;
  dataChannel channel[3];
};


class MRS1000 : public CoLaA
{
public:

protected:
  void parseScanData(char *buffer, void *scan_data) const;
};

#endif // MRS1000_H
