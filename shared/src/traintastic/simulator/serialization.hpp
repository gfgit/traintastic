/**
 * This file is part of Traintastic,
 * see <https://github.com/traintastic/traintastic>.
 *
 * Copyright (C) 2026 Filippo Gentile
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

#ifndef TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_SERIALIZATION_HPP
#define TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_SERIALIZATION_HPP

#include <QDataStream>
#include <QByteArray>

#include "simulator.hpp"

// QDataStream &operator<<(QDataStream &stream, const size_t &sz)
// {
//   stream << quint64(sz);
//   return stream;
// }

// QDataStream &operator>>(QDataStream &stream, size_t &sz)
// {
//   quint64 val = 0;
//   stream >> val;
//   sz = val;
//   return stream;
// }

QDataStream &operator<<(QDataStream &stream, const std::string &str)
{
  stream << quint64(str.size());
  for(const char ch : str)
    stream << ch;
  return stream;
}

QDataStream &operator>>(QDataStream &stream, std::string &str)
{
  quint64 sz = 0;
  stream >> sz;
  str.resize(sz);

  for(size_t i = 0; i < sz; i++)
    stream >> str[i];
  return stream;
}


QDataStream &operator<<(QDataStream &stream, const Simulator::Point &p)
{
  stream << p.x;
  stream << p.y;
  return stream;
}

QDataStream &operator>>(QDataStream &stream, Simulator::Point &p)
{
  stream >> p.x;
  stream >> p.y;
  return stream;
}

QDataStream &operator<<(QDataStream &stream, const Simulator::VehicleState::Face &face)
{
  stream << quint64(face.segmentIndex);
  stream << face.segmentDirectionInverted;
  stream << face.distance;
  return stream;
}

QDataStream &operator>>(QDataStream &stream, Simulator::VehicleState::Face &face)
{
  quint64 idx = 0;
  stream >> idx;
  face.segmentIndex = idx;

  stream >> face.segmentDirectionInverted;
  stream >> face.distance;
  return stream;
}

QDataStream &operator<<(QDataStream &stream, const Simulator::VehicleState &vehicleState)
{
  stream << vehicleState.front;
  stream << vehicleState.rear;
  return stream;
}

QDataStream &operator>>(QDataStream &stream, Simulator::VehicleState &vehicleState)
{
  stream >> vehicleState.front;
  stream >> vehicleState.rear;
  return stream;
}

QDataStream &operator<<(QDataStream &stream, const Simulator::Vehicle &vehicle)
{
  stream << vehicle.vehicleId;
  stream << quint64(vehicle.typeIdx);
  stream << vehicle.state;

  if(vehicle.typeIdx == Simulator::invalidIndex)
  {
    stream << vehicle.color;
    stream << vehicle.length;
  }
  return stream;
}

QDataStream &operator>>(QDataStream &stream, Simulator::Vehicle &vehicle)
{
  stream >> vehicle.vehicleId;

  quint64 idx = 0;
  stream >> idx;
  vehicle.typeIdx = idx;

  stream >> vehicle.state;

  if(vehicle.typeIdx == Simulator::invalidIndex)
  {
    stream >> vehicle.color;
    stream >> vehicle.length;
  }
  return stream;
}

void serializeVehicleDescr(QDataStream &stream, const Simulator::Vehicle &vehicle)
{
  stream << quint64(vehicle.typeIdx);

  if(vehicle.typeIdx == Simulator::invalidIndex)
  {
    stream << vehicle.color;
    stream << vehicle.length;
  }
}

void deserializeVehicleDescr(QDataStream &stream, Simulator::Vehicle &vehicle)
{
  quint64 idx = 0;
  stream >> idx;
  vehicle.typeIdx = idx;

  if(vehicle.typeIdx == Simulator::invalidIndex)
  {
    stream >> vehicle.color;
    stream >> vehicle.length;
  }
}

void serializeVehicleList(QDataStream &stream, const std::vector<Simulator::Train::VehicleItem> &vehicles, bool full)
{
  stream << quint64(vehicles.size());
  for(const auto &item : vehicles)
  {
    if(full)
      stream << *item.vehicle;
    else
      stream << item.vehicle->state;

    stream << item.reversed;
  }
}

void deserializeVehicleList(QDataStream &stream, std::vector<Simulator::Train::VehicleItemFull> &vehicles, bool full)
{
  quint64 sz = 0;
  stream >> sz;
  vehicles.reserve(sz);

  for(size_t i = 0; i < sz; i++)
  {
    Simulator::Train::VehicleItemFull item;

    if(full)
      stream >> item.vehicle;
    else
      stream >> item.vehicle.state;

    stream >> item.reversed;

    vehicles.push_back(item);
  }
}

QDataStream &operator<<(QDataStream &stream, const Simulator::TrainState &trainState)
{
  stream << trainState.speed;
  stream << trainState.targetSpeed;
  stream << trainState.waitToStart_ms;
  stream << trainState.reverse;
  stream << trainState.mode;
  stream << trainState.isOnStationStop;
  return stream;
}

QDataStream &operator>>(QDataStream &stream, Simulator::TrainState &trainState)
{
  stream >> trainState.speed;
  stream >> trainState.targetSpeed;
  stream >> trainState.waitToStart_ms;
  stream >> trainState.reverse;
  stream >> trainState.mode;
  stream >> trainState.isOnStationStop;
  return stream;
}

void serializeTrain(QDataStream &stream, const Simulator::Train &train, bool full)
{
  stream << train.trainId;

  if(full)
  {
    stream << train.name;
    stream << train.speedMax;
  }

  stream << train.state;
  serializeVehicleList(stream, train.vehicles, full);
}

void deserializeTrain(QDataStream &stream, std::vector<Simulator::Train::VehicleItemFull> &vehicles,
                      std::string &name, quint32 &trainId, Simulator::TrainState &trainState, float &speedMax,
                      bool full)
{
  stream >> trainId;

  if(full)
  {
    stream >> name;
    stream >> speedMax;
  }

  stream >> trainState;

  deserializeVehicleList(stream, vehicles, full);
}

void serializeTrainAddRequest(QDataStream &stream,
                              const std::string& name,
                              const std::vector<Simulator::Train::VehicleItemFull> &vehicles,
                              size_t segmentIndex, quint32 tokenId, float startPos, Simulator::TrainPlacement placement)
{
  stream << name;

  stream << quint64(vehicles.size());
  for(const auto &item : vehicles)
  {
    serializeVehicleDescr(stream, item.vehicle);
    stream << item.reversed;
  }

  stream << quint64(segmentIndex);
  stream << tokenId;
  stream << startPos;
  stream << uint8_t(placement);
}

void deserializeTrainAddRequest(QDataStream &stream,
                                std::string& name,
                                std::vector<Simulator::Train::VehicleItemFull> &vehicles,
                                size_t &segmentIndex, quint32 &tokenId, float &startPos, Simulator::TrainPlacement &placement)
{
  stream >> name;

  quint64 sz = 0;
  stream >> sz;
  vehicles.reserve(sz);

  for(size_t i = 0; i < sz; i++)
  {
    Simulator::Train::VehicleItemFull item;
    deserializeVehicleDescr(stream, item.vehicle);
    stream >> item.reversed;
    vehicles.push_back(item);
  }

  stream >> sz;
  segmentIndex = sz;
  stream >> tokenId;
  stream >> startPos;

  uint8_t tmp1 = 0;
  stream >> tmp1;
  placement = Simulator::TrainPlacement(tmp1);
}


#endif // TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_SERIALIZATION_HPP
