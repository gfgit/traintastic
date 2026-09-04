/**
 * This file is part of Traintastic,
 * see <https://github.com/traintastic/traintastic>.
 *
 * Copyright (C) 2025 Reinder Feenstra
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

#ifndef TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_PROTOCOL_HPP
#define TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_PROTOCOL_HPP

#include "../enum/decoderprotocol.hpp"
#include "../enum/direction.hpp"
#include "../utils/packed.hpp"

#include <QtGlobal>

namespace SimulatorProtocol
{

enum class OpCode : uint8_t
{
  Power = 1,
  LocomotiveSpeedDirection = 2,
  SensorChanged = 3,
  AccessorySetState = 4,
  Handshake = 5,
  HandshakeResponse = 6,
  SignalSetState = 7,
  OwnSignal = 8,
  RequestChannel = 9,
  OwnSpawn = 10,
  SpawnStateChange = 11,
  AuxSignalSetState = 12,
  SensorBatchState = 13,
  SensorEncodingChanged = 14,
  ReplicaGreeting = 15,
  TrainSpeedFactorChanged = 16,

  // Special messages with variable size
  TrainAdded = 17,
  TrainStateRefresh = 18,
  TrainChangedVehicles = 19,
  TrainRequestAdd = 20,

  // Again fixed size
  TrainRequestSetMode = 21,
  TrainRequestSetDirection = 22,
  TrainRequestChangeSpeed = 23,
  TrainRequestCoupleSplit = 24,
  TrainRequestRemove = 25,
  TrainRemoved = 26,
};

enum class TrainMode : uint8_t
{
  Manual = 0,
  SemiAutomatic = 1,
  Automatic = 2
};

PRAGMA_PACK_PUSH_1

struct Message
{
  OpCode opCode;
  uint16_t sizeLE; // Little endian

  Message(OpCode opCode_, uint16_t size_)
    : opCode{opCode_}
  {
    setSize(size_);
  }

  inline uint16_t size() const
  {
    uint16_t result = sizeLE;

    if constexpr (std::endian::native == std::endian::big)
    {
      // Swap bytes
      uint8_t *b = reinterpret_cast<uint8_t *>(&result);
      std::swap(b[0], b[1]);
    }

    return result;
  }

  inline void setSize(uint16_t size_)
  {
    sizeLE = size_;

    if constexpr (std::endian::native == std::endian::big)
    {
      // Swap bytes
      uint8_t *b = reinterpret_cast<uint8_t *>(&sizeLE);
      std::swap(b[0], b[1]);
    }
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(Message) == 3);

struct HandShake : Message
{
  HandShake(bool reply)
    : Message(reply ? OpCode::HandshakeResponse : OpCode::Handshake,
              sizeof(HandShake))
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(HandShake) == 3);

struct ReplicaGreeting : Message
{
  ReplicaGreeting()
    : Message(OpCode::ReplicaGreeting,
              sizeof(ReplicaGreeting))
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(ReplicaGreeting) == 3);

struct Power : Message
{
  uint8_t powerOn;

  Power(bool on)
    : Message(OpCode::Power, sizeof(Power))
    , powerOn(on ? 1 : 0)
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(Power) == 4);

struct LocomotiveSpeedDirection : Message
{
  uint16_t address;
  DecoderProtocol protocol;
  uint8_t emergencyStop;
  Direction direction;
  uint8_t speed;

  LocomotiveSpeedDirection(uint16_t addr, DecoderProtocol proto, uint8_t spd, Direction dir, bool eStop)
    : Message(OpCode::LocomotiveSpeedDirection, sizeof(LocomotiveSpeedDirection))
    , address{addr}
    , protocol{proto}
    , emergencyStop(eStop ? 1 : 0)
    , direction{dir}
    , speed{spd}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(LocomotiveSpeedDirection) == 9);

struct SensorChanged : Message
{
  uint16_t channel;
  uint16_t address;
  int32_t axleCount;
  uint8_t value;

  SensorChanged(uint16_t ch, uint16_t addr, int32_t count, bool val)
    : Message(OpCode::SensorChanged, sizeof(SensorChanged))
    , channel{ch}
    , address{addr}
    , axleCount(count)
    , value(val ? 1 : 0)
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(SensorChanged) == 12);

struct SensorEncodingChanged : Message
{
  uint16_t channel;
  uint16_t address;
  uint8_t encoding = 0;

  SensorEncodingChanged(uint16_t ch, uint16_t addr, uint8_t encoding_)
    : Message(OpCode::SensorEncodingChanged, sizeof(SensorEncodingChanged))
    , channel{ch}
    , address{addr}
    , encoding{encoding_}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(SensorEncodingChanged) == 8);

struct AccessorySetState : Message
{
  uint16_t channel;
  uint16_t address;
  uint8_t state;

  AccessorySetState(uint16_t ch, uint16_t addr, uint8_t st)
    : Message(OpCode::AccessorySetState, sizeof(AccessorySetState))
    , channel{ch}
    , address{addr}
    , state{st}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(AccessorySetState) == 8);

struct SignalSetState : Message
{
  uint16_t channel;
  uint16_t address;

  enum Color
  {
    Red = 0,
    Yellow,
    Green
  };

  enum State
  {
    Off = 0,
    On,
    Blink,
    BlinkReverse,
  };

  enum RappelState
  {
    Rappel_Off = 0,
    OneLine_60,
    TwoLines_100
  };

  static constexpr uint8_t StateMask = uint8_t(State::BlinkReverse);
  static constexpr uint8_t ArrowLight = 0x04;

  struct LightState
  {
    uint8_t color;
    uint8_t state;
  };

  LightState lights[3];
  float speed = 0.0;
  uint8_t advanceSignalStateAndArrow = 0;
  uint8_t rappelState = RappelState::Rappel_Off;
  char directionIndication = ' ';

  inline State getAdvanceSignalState() const { return State(advanceSignalStateAndArrow & StateMask); }
  inline void setAdvanceSignalState(State s)
  {
    advanceSignalStateAndArrow = (advanceSignalStateAndArrow & ~StateMask) | uint8_t(s);
  }

  inline bool isArrowLightOn() const { return (advanceSignalStateAndArrow & ArrowLight) == ArrowLight; }
  inline void setArrowLightOn(bool on)
  {
    if(on)
      advanceSignalStateAndArrow |= ArrowLight;
    else
      advanceSignalStateAndArrow &= ~ArrowLight;
  }

  SignalSetState(uint16_t ch, uint16_t addr)
    : Message(OpCode::SignalSetState, sizeof(SignalSetState))
    , channel{ch}
    , address{addr}
  {
    lights[0] = {Color::Red, State::Off};
    lights[1] = {Color::Red, State::Off};
    lights[2] = {Color::Red, State::Off};
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(SignalSetState) == 20);

struct AuxSignalSetState : Message
{
  uint16_t channel;
  uint16_t address;
  uint8_t lights = 0;
  uint8_t position = 0;

  AuxSignalSetState(uint16_t ch, uint16_t addr)
    : Message(OpCode::AuxSignalSetState, sizeof(AuxSignalSetState))
    , channel{ch}
    , address{addr}

  {
  }

  inline void setLightOn(uint8_t n, bool on)
  {
    assert(n >= 0 && n < 8);
    if(on)
      lights |= (1u << n);
    else
      lights &= ~(1u << n);
  }

  inline bool isLightOn(uint8_t n)
  {
    assert(n >= 0 && n < 8);
    return lights & (1u << n);
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(AuxSignalSetState) == 9);

struct OwnSignal : Message
{
  uint16_t channel;
  uint16_t address;
  uint8_t isMain = 1;

  OwnSignal(uint16_t ch, uint16_t addr, bool main)
    : Message(OpCode::OwnSignal, sizeof(OwnSignal))
    , channel{ch}
    , address{addr}
    , isMain{main}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(OwnSignal) == 8);

struct RequestChannel : Message
{
  //! NOTE: pass invalidAddress to get all channels state
  uint16_t channel;

  RequestChannel(uint16_t ch)
    : Message(OpCode::RequestChannel, sizeof(RequestChannel))
    , channel{ch}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(RequestChannel) == 5);

struct OwnSpawn : Message
{
  uint16_t address;

  OwnSpawn(uint16_t addr)
    : Message(OpCode::OwnSpawn, sizeof(OwnSpawn))
    , address{addr}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(OwnSpawn) == 5);

struct SpawnStateChange : Message
{
  enum State
  {
    Ready = 1,
    WaitingReset = 2,

    RequestActivate = 20,
    Reset = 21
  };

  uint16_t address;
  uint8_t state;


  SpawnStateChange(uint16_t addr, uint8_t s)
    : Message(OpCode::SpawnStateChange, sizeof(SpawnStateChange))
    , address{addr}
    , state{s}
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(SpawnStateChange) == 6);

struct SensorBatchState : Message
{
  uint16_t channel;
  uint16_t batchIdx;
  uint16_t state;

  SensorBatchState(uint16_t ch, uint16_t batchIdx_, uint16_t state_)
    : Message(OpCode::SensorBatchState, sizeof(SensorBatchState))
    , channel{ch}
    , batchIdx{batchIdx_}
    , state(state_)
  {
  }
} ATTRIBUTE_PACKED;
static_assert(sizeof(SensorBatchState) == 9);

struct TrainSpeedFactorChanged : Message
{
  TrainSpeedFactorChanged(float trainSpeedFactor_)
    : Message(OpCode::TrainSpeedFactorChanged,
              sizeof(TrainSpeedFactorChanged))
    , trainSpeedFactor(trainSpeedFactor_)
  {
  }

  float trainSpeedFactor;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainSpeedFactorChanged) == 7);

struct TrainRequestSetMode : Message
{
  TrainRequestSetMode(quint32 trainId_, TrainMode mode_)
    : Message(OpCode::TrainRequestSetMode,
              sizeof(TrainRequestSetMode))
    , trainId(trainId_)
    , mode(mode_)
  {
  }

  quint32 trainId;
  TrainMode mode;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainRequestSetMode) == 8);

struct TrainRequestSetDirection : Message
{
  TrainRequestSetDirection(quint32 trainId_, bool reverse_)
    : Message(OpCode::TrainRequestSetDirection,
              sizeof(TrainRequestSetDirection))
    , trainId(trainId_)
    , reverse(reverse_)
  {
  }

  quint32 trainId;
  bool reverse;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainRequestSetDirection) == 8);

struct TrainRequestChangeSpeed : Message
{
  enum SpeedStep : uint8_t
  {
    Stop = 0,
    IncreaseStep = 1,
    DecreaseStep = 2
  };

  TrainRequestChangeSpeed(quint32 trainId_, SpeedStep step_)
    : Message(OpCode::TrainRequestChangeSpeed,
              sizeof(TrainRequestChangeSpeed))
    , trainId(trainId_)
    , step(step_)
  {
  }

  quint32 trainId;
  SpeedStep step;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainRequestChangeSpeed) == 8);

struct TrainRequestCoupleSplit : Message
{
  enum CoupleOp : uint8_t
  {
    CoupleFront = 0,
    CoupleRear = 1,
    SplitFront = 2,
    SplitRear = 3
  };

  TrainRequestCoupleSplit(quint32 trainId_, size_t vehicleIdx_, CoupleOp coupleOp_)
    : Message(OpCode::TrainRequestCoupleSplit,
              sizeof(TrainRequestCoupleSplit))
    , trainId(trainId_)
    , vehicleIdx(vehicleIdx_)
    , coupleOp(coupleOp_)
  {
  }

  quint32 trainId;
  size_t vehicleIdx = 0;
  CoupleOp coupleOp = CoupleOp::CoupleFront;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainRequestCoupleSplit) == 16);

struct TrainRemoved : Message
{
  TrainRemoved(quint32 trainId_, bool request, quint32 nextSelectedTrainId_ = 0)
    : Message(request ? OpCode::TrainRequestRemove : OpCode::TrainRemoved,
              sizeof(TrainRemoved))
    , trainId(trainId_)
    , nextSelectedTrainId(request ? 0 : nextSelectedTrainId_)
  {
  }

  quint32 trainId;
  quint32 nextSelectedTrainId = 0;
} ATTRIBUTE_PACKED;
static_assert(sizeof(TrainRemoved) == 11);

PRAGMA_PACK_POP

}

#endif
