/**
 * server/src/vehicle/rail/poweredrailvehicle.cpp
 *
 * This file is part of the traintastic source code.
 *
 * Copyright (C) 2023 Reinder Feenstra
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "poweredrailvehicle.hpp"
#include "../../utils/almostzero.hpp"

PoweredRailVehicle::PoweredRailVehicle(World& world, std::string_view id_)
  : RailVehicle(world, id_)
{
}

void PoweredRailVehicle::setDirection(Direction value)
{
  if(decoder)
    decoder->direction = value;
}

void PoweredRailVehicle::setSpeed(double kmph)
{
  if(!decoder)
    return;

  if(almostZero(kmph))
  {
    decoder->throttle.setValue(0);
    return;
  }

  //! \todo Implement speed profile

  // No speed profile -> linear
  {
    const double max = speedMax.getValue(SpeedUnit::KiloMeterPerHour);
    if(max > 0)
    {
      const uint8_t steps = decoder->speedSteps;
      if(steps == Decoder::speedStepsAuto)
        decoder->throttle.setValue(kmph / max);
      else
        decoder->throttle.setValue(std::round(kmph / max * steps) / steps);
    }
    else
      decoder->throttle.setValue(0);
  }
}
