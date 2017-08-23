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

#include "lms1xx/mrs1000.h"
#include <cstring>
#include <sstream>
#include <console_bridge/console.h>

MRS1000::MRS1000()
{
  SET_ACTIVE_APPLICATIONS = "sWN SetActiveApplications";
}

MRS1000::~MRS1000()
{

}

void MRS1000::enableRangingApplication()
{
  setActiveApplications("RANG", true);
}

void MRS1000::setActiveApplications(std::string application, bool active)
{
  std::stringstream ss;
  ss << SET_ACTIVE_APPLICATIONS << " 1 " << application << " " << (active ? 1 : 0);
  sendCommand(ss.str());
}
