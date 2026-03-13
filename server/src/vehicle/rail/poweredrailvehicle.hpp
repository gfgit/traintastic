/**
 * server/src/vehicle/rail/poweredrailvehicle.hpp
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

#ifndef TRAINTASTIC_SERVER_VEHICLE_RAIL_POWEREDRAILVEHICLE_HPP
#define TRAINTASTIC_SERVER_VEHICLE_RAIL_POWEREDRAILVEHICLE_HPP

#include <memory>
#include "railvehicle.hpp"
#include "../../core/powerproperty.hpp"
#include "../../core/method.hpp"
#include "../../core/objectproperty.hpp"

#include "vehiclespeedcurve.hpp" // TODO: i would like to forwad declare

class VehicleSpeedCurve;

class PoweredRailVehicle : public RailVehicle
{
  protected:
    PoweredRailVehicle(World& world, std::string_view id_);

    void loaded() override;
    void worldEvent(WorldState state, WorldEvent event) override;

    void onDecoderChanged(Decoder& decoderRef, DecoderChangeFlags flags, uint32_t functionNumber) override;

    friend class VehicleSpeedCurve;
    void updateMaxSpeed();

    friend class Train;
    float lastTrainSpeedStep = 0;

  public:
    PowerProperty power;
    ObjectProperty<VehicleSpeedCurve> speedCurve;
    Property<double> maxAccelerationRate; // m/s^2
    Property<double> maxBrakingRate; // m/s^2
};

#endif
