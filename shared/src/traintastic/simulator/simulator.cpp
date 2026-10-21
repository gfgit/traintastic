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

#include "simulator.hpp"
#include "simulatorconnection.hpp"
#include <ranges>
#include <bit>
#include <format>
#include <random>
#include <algorithm>
#include <chrono>
#include <iostream>
#include "protocol.hpp"

#include "serialization.hpp"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

static std::mt19937 rng = std::mt19937(std::random_device{}());

constexpr static float ADJ_MARGIN = 0.000001;

inline bool almostEqual(float a, float b)
{
  return std::abs(a - b) < 0.00001;
}

template <typename T>
inline bool vec_contains(const std::vector<T>& vec, const T& item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

void Simulator::updateView(Simulator::StaticData::View& view, Simulator::Point point)
{
  if(point.x < view.left)
  {
    view.left = point.x;
  }
  if(point.x > view.right)
  {
    view.right = point.x;
  }
  if(point.y < view.top)
  {
    view.top = point.y;
  }
  if(point.y > view.bottom)
  {
    view.bottom = point.y;
  }
}

void Simulator::updateView(Simulator::StaticData::View& view, const Simulator::TrackSegment::Curve& curve, float startAngle)
{
  if(curve.angle < 0)
  {
    startAngle += pi;
  }

  float endAngle = startAngle + curve.angle;

  if(endAngle < startAngle)
  {
    std::swap(startAngle, endAngle);
  }

  // Start and end points:
  {
    const float x = curve.center.x + curve.radius * std::sin(startAngle);
    const float y = curve.center.y - curve.radius * std::cos(startAngle);
    updateView(view, {x, y});
  }
  {
    const float x = curve.center.x + curve.radius * std::sin(endAngle);
    const float y = curve.center.y - curve.radius * std::cos(endAngle);
    updateView(view, {x, y});
  }

  // Check critical angles (0, 90, 180, 270 degrees):
  const int start = static_cast<int>(std::ceil(startAngle / (0.5f * pi)));
  const int end = static_cast<int>(std::floor(endAngle / (0.5f * pi)));

  for(int i = start; i <= end; ++i)
  {
    const float angle = i * 0.5f * pi;
    const float x = curve.center.x + curve.radius * std::sin(angle);
    const float y = curve.center.y - curve.radius * std::cos(angle);
    updateView(view, {x, y});
  }
}

namespace
{

#ifndef NDEBUG
constexpr size_t getStraightCount(Simulator::TrackSegment::Type type)
{
  using Type = Simulator::TrackSegment::Type;
  switch(type)
  {
  case Type::Curve:
  case Type::TurnoutCurved:
    return 0;

  case Type::Straight:
  case Type::Turnout:
  case Type::Turnout3Way:
    return 1;

  case Type::SingleSlipTurnout:
  case Type::DoubleSlipTurnout:
    return 2;
  }
  return 0;
}
#endif

constexpr size_t getCurveCount(Simulator::TrackSegment::Type type)
{
  using Type = Simulator::TrackSegment::Type;
  switch(type)
  {
  case Type::Straight:
    return 0;

  case Type::Curve:
  case Type::Turnout:
  case Type::SingleSlipTurnout:
    return 1;

  case Type::TurnoutCurved:
  case Type::Turnout3Way:
  case Type::DoubleSlipTurnout:
    return 2;
  }
  return 0;
}

Simulator::Point straightEnd(const Simulator::TrackSegment& segment)
{
  assert(1 == getStraightCount(segment.type));
  return {
          segment.points[0].x + segment.straight.length * std::cos(segment.rotation), segment.points[0].y + segment.straight.length * std::sin(segment.rotation)};
}

Simulator::Point curveEnd(const Simulator::TrackSegment& segment, size_t curveIndex = 0)
{
  assert(curveIndex < getCurveCount(segment.type));

  const auto& curve = segment.curves[curveIndex];
  const float angle = (curve.angle < 0) ? (segment.rotation + pi) : segment.rotation;

  return {curve.center.x + curve.radius * std::sin(angle + curve.angle),
          curve.center.y - curve.radius * std::cos(angle + curve.angle)};
}

#ifndef _MSC_VER // std::abs is not constexpr :(
constexpr
#endif
    bool pointsClose(Simulator::Point a, Simulator::Point b)
{
  return std::abs(a.x - b.x) <= 1.0f && std::abs(a.y - b.y) <= 1.0f;
}

constexpr size_t getPointCount(Simulator::TrackSegment::Type type)
{
  using Type = Simulator::TrackSegment::Type;
  switch(type)
  {
  case Type::Straight:
  case Type::Curve:
    return 2;

  case Type::Turnout:
  case Type::TurnoutCurved:
    return 3;

  case Type::Turnout3Way:
  case Type::SingleSlipTurnout:
  case Type::DoubleSlipTurnout:
    return 4;
  }
  return 0;
}

float getPointRotation(const Simulator::TrackSegment& segment, size_t pointIndex)
{
  using Type = Simulator::TrackSegment::Type;
  assert(pointIndex < getPointCount(segment.type));

  if(pointIndex == 0)
  {
    return segment.rotation + pi;
  }
  else if(pointIndex == 1 && (segment.type == Type::Straight || segment.type == Type::Turnout || segment.type == Type::Turnout3Way))
  {
    return segment.rotation;
  }
  else if((pointIndex == 1 && (segment.type == Type::Curve || segment.type == Type::TurnoutCurved)) ||
           (pointIndex == 2 && (segment.type == Type::Turnout || segment.type == Type::Turnout3Way)))
  {
    return segment.rotation + segment.curves[0].angle;
  }
  else if((pointIndex == 2 && segment.type == Type::TurnoutCurved) ||
           (pointIndex == 3 && segment.type == Type::Turnout3Way))
  {
    return segment.rotation + segment.curves[1].angle;
  }
  else if(segment.type == Type::SingleSlipTurnout || segment.type == Type::DoubleSlipTurnout)
  {
    if(pointIndex == 1)
      return segment.rotation;
    if(pointIndex == 2)
      return segment.rotation + segment.curves[0].angle;
    if(pointIndex == 3)
      return segment.rotation + segment.curves[0].angle + pi;
  }

  assert(false);
  return std::numeric_limits<float>::signaling_NaN();
}

float getSegmentLength(const Simulator::TrackSegment& segment, const Simulator::StateData& stateData)
{
  using Type = Simulator::TrackSegment::Type;
  using State = Simulator::TurnoutState::State;

  switch(segment.type)
  {
  case Type::Straight:
    return segment.straight.length;

  case Type::Curve:
    return segment.curves[0].length;

  case Type::Turnout:
    switch(stateData.turnouts[segment.turnout.index].state)
    {
    case State::Closed:
      return segment.straight.length;

    case State::Thrown:
      return segment.curves[0].length;

    default: // all others are invalid
      break;
    }
    break;

  case Type::TurnoutCurved:
    switch(stateData.turnouts[segment.turnout.index].state)
    {
    case State::Closed:
      return segment.curves[0].length;

    case State::Thrown:
      return segment.curves[1].length;

    default: // all others are invalid
      break;
    }
    break;

  case Type::Turnout3Way:
    switch(stateData.turnouts[segment.turnout.index].state)
    {
    case State::Closed:
      return segment.straight.length;

    case State::ThrownLeft:
      return segment.curves[0].length;

    case State::ThrownRight:
      return segment.curves[1].length;

    default: // all others are invalid
      break;
    }
    break;

  case Type::SingleSlipTurnout:
  case Type::DoubleSlipTurnout:
    switch(stateData.turnouts[segment.turnout.index].state)
    {
    case State::Closed:
    case State::ClosedLeft:
    case State::ClosedRight:
      return segment.straight.length;

    case State::Thrown:
    case State::ThrownLeft:
    case State::ThrownRight:
      return segment.curves[0].length;

    default: // all others are invalid
      break;
    }
    break;
  }
  return 0.0f;
}

size_t getNextSegmentIndex(const Simulator::TrackSegment& segment, bool directionPositive, const Simulator::StateData& stateData)
{
  using State = Simulator::TurnoutState::State;
  if(!directionPositive)
  {
    return segment.nextSegmentIndex[0];
  }
  if((segment.type == Simulator::TrackSegment::Type::Turnout || segment.type == Simulator::TrackSegment::Type::TurnoutCurved) &&
      stateData.turnouts[segment.turnout.index].state == Simulator::TurnoutState::State::Thrown)
  {
    return segment.nextSegmentIndex[2];
  }
  if(segment.type == Simulator::TrackSegment::Type::Turnout3Way)
  {
    const auto state = stateData.turnouts[segment.turnout.index].state;
    if(state == State::ThrownLeft)
    {
      return segment.nextSegmentIndex[2];
    }
    else if(state == State::ThrownRight)
    {
      return segment.nextSegmentIndex[3];
    }
    assert(state == State::Closed);
  }
  return segment.nextSegmentIndex[1];
}

template<typename T>
std::optional<T> stringToEnum(std::string_view value)
{
  auto it = std::find_if(EnumValues<T>::value.begin(), EnumValues<T>::value.end(),
                         [&value](const auto& n)
                         {
                           return n.second == value;
                         });

  if(it != EnumValues<T>::value.end())
  {
    return it->first;
  }
  return std::nullopt;
}

}

void clearTrainSignalCache(Simulator::StateData &data, Simulator::Train &train, bool next)
{
  Simulator::TrainState::SignalCache &cache = next ? train.state.nextSignal : train.state.prevSignal;

  cache.signal = nullptr;
  cache.segmentIdx = Simulator::invalidIndex;
  cache.distance = 0.0;
  cache.posInSegment = 0.0;
  cache.dirty = false;

  for(const size_t idx : cache.turnouts)
  {
    std::erase_if(data.turnouts[idx].watchingTrainIDs,
                  [train](quint32 trainId) -> bool
                  {
                    return trainId == train.trainId;
                  });
  }
  cache.turnouts.clear();
}

Simulator::Simulator(const nlohmann::json& world, TrainTypeInterface *iface)
  : staticData(load(world, m_stateData, iface))
  , m_tickTimer{m_coreContext}
  , m_garbageCollectTimer{m_coreContext}
  , m_turnoutMoveTimer{m_coreContext}
  , m_acceptor{m_networkContext}
  , m_socketUDP{m_networkContext}
  , m_handShakeSendTimer{m_networkContext}
  , m_handShakeRecvTimer{m_networkContext}
  , m_syncSensorStateTimer{m_networkContext}
  , m_syncReplicaStateTimer{m_networkContext}
  , m_syncReplicaTrainFullStateTimer{m_networkContext}
  , m_replicaDiscoverSendTimer{m_networkContext}
  , trainTypeInterface(iface)
{
  loadTrains(world);
}

Simulator::~Simulator()
{
  stop();

  {
    std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

    m_stateData.mainSignals.clear();
    m_stateData.auxSignals.clear();

    {
      auto it = m_stateData.trains.begin();
      while(it != m_stateData.trains.end())
      {
        Train &train = it->second;
        destroyTrain(train, true);
        it = m_stateData.trains.erase(it);
      }
    }

    {
      while(!m_stateData.vehicles.empty())
      {
        removeVehicle(&m_stateData.vehicles.begin()->second);
      }
    }
  }
}

const Simulator::StateData& Simulator::stateData() const
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  return m_stateData;
}

void Simulator::enableServer(bool localhostOnly, uint16_t port)
{
  m_serverEnabled = true;
  m_serverLocalHostOnly = localhostOnly;
  m_serverPort = port;
}

uint16_t Simulator::serverPort() const
{
  return m_acceptor.local_endpoint().port();
}

void Simulator::start(bool discoverable)
{
  m_coreThread = std::thread(
      [this]()
      {
        tick();
        garbageCollectSegmentState();

        m_coreContext.run();
      });

  m_networkThread = std::thread(
      [this, discoverable]()
      {
        if(m_serverEnabled)
        {
          boost::system::error_code ec;
          const boost::asio::ip::tcp::endpoint endpoint(m_serverLocalHostOnly ? boost::asio::ip::address_v4::loopback() : boost::asio::ip::address_v4::any(), m_serverPort);

          m_acceptor.open(endpoint.protocol(), ec);

          m_acceptor.bind(endpoint, ec);

          m_acceptor.listen(5, ec);

          if(discoverable)
          {
            m_socketUDP.open(boost::asio::ip::udp::v4(), ec);
            if(ec)
              assert(false);

            m_socketUDP.set_option(boost::asio::socket_base::reuse_address(true), ec);
            if(ec)
              assert(false);

            m_socketUDP.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), defaultPort), ec);
            if(ec)
              assert(false);

            doReceive();
          }

          accept();
        }

        handShakeSend();
        handShakeRecv();
        syncSensorState();

        m_networkContext.run();
      });
}

void Simulator::stop()
{
  boost::asio::post(
      m_networkContext, [this]()
      {
        // Stop UDP discovery
        boost::system::error_code ec;
        m_socketUDP.close(ec);

        while(!m_connections.empty())
        {
          auto connection =  m_connections.back();
          m_connections.pop_back();
          connection->stop();
        }

        while(!m_repliacas.empty())
        {
          auto connection =  m_repliacas.back();
          m_repliacas.pop_back();
          connection->stop();
        }

        // Stop TCP server
        m_acceptor.cancel(ec);
        m_acceptor.close(ec);

        m_handShakeSendTimer.cancel();
        m_handShakeRecvTimer.cancel();
        m_syncSensorStateTimer.cancel();
        m_syncReplicaStateTimer.cancel();
        m_syncReplicaTrainFullStateTimer.cancel();
        m_replicaDiscoverSendTimer.cancel();
      });

  m_tickTimer.cancel();
  m_garbageCollectTimer.cancel();
  m_turnoutMoveTimer.cancel();

  m_coreContext.stop();
  m_networkContext.stop();

  if(m_coreThread.joinable())
  {
    m_coreThread.join();
  }

  if(m_networkThread.joinable())
  {
    m_networkThread.join();
  }
}

void Simulator::setPowerOn(bool powerOn)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  if(getSimModeUnlocked() == SimMode::Replica)
  {
    // Send request to master
    boost::asio::post(
      m_networkContext, [this, powerOn]()
      {
        send(SimulatorProtocol::Power(powerOn));
      });
  }
  else
  {
    if(m_stateData.powerOn != powerOn)
    {
      m_stateData.powerOn = powerOn;

      boost::asio::post(
        m_networkContext, [this, powerOn=m_stateData.powerOn]()
        {
          // Cherry pick value
          m_networkStateData.powerOn = powerOn;
          send(SimulatorProtocol::Power(m_stateData.powerOn));
          sendReplicas(SimulatorProtocol::Power(m_stateData.powerOn));
        });
    }
  }
}

void Simulator::togglePowerOn()
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  setPowerOn(!m_stateData.powerOn);
}

std::pair<Simulator::Point, Simulator::Point> Simulator::getTrainExtents(const StateData &stateData_, const Train &train)
{
  if(train.vehicles.empty())
    return {};

  std::pair<Simulator::Point, Simulator::Point> ret;

  const auto frontVehicleItem = train.vehicles.front();
  const auto rearVehicleItem = train.vehicles.back();

  const Simulator::Vehicle *frontVehicle = stateData_.getVehicleById(frontVehicleItem.vehicleId);
  const Simulator::Vehicle *rearVehicle = stateData_.getVehicleById(rearVehicleItem.vehicleId);

  ret.first = frontVehicleItem.reversed ? frontVehicle->state.rear.position : frontVehicle->state.front.position;
  ret.second = !rearVehicleItem.reversed ? rearVehicle->state.rear.position : rearVehicle->state.front.position;

  return ret;
}

void Simulator::setTrainDirection(Train &train, bool reverse)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica && train.state.reverse != reverse)
  {
    boost::asio::post(
      m_networkContext, [this, trainId=train.trainId, reverse]()
      {
        send(SimulatorProtocol::TrainRequestSetDirection(trainId, reverse));
      });
    return;
  }

  if(train.state.reverse != reverse)
  {
    train.state.reverse = reverse;
    setTrainSpeed(train, 0.0, true); // Stop train when changing direction
    train.state.speedOrDirectionChanged = true;
    train.state.nextSignal.dirty = true;
    train.state.prevSignal.dirty = true;
    train.state.skipCollisionDetection = false; // Check again in opposite direction

    train.state.resetSignalSpeeds();
  }
}

void Simulator::setTrainSpeed(Train &train, float speed, bool immediate)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(!train.isPowered)
    speed = 0.0f;

  speed = std::clamp(speed, 0.0f, train.speedMax);
  if(immediate && train.state.speed != speed)
  {
    // std::cout << "TRAIN SET IMM: " << speed / SpeedKmHtoTick << " km/h    FROM: "
    //           << train.state.speed / SpeedKmHtoTick << " km/h" << std::endl;

    train.state.speed = speed;
    train.state.targetSpeed = speed; // Also adjust target if immediate
    train.state.waitToStart_ms = 0.0;
    train.state.speedOrDirectionChanged = true;
  }
  else if(!immediate && train.state.targetSpeed != speed)
  {
    const float oldTargetSpeed = train.state.targetSpeed;
    train.state.targetSpeed = speed;

    // std::cout << "TRAIN SET TARGET: " << train.state.targetSpeed / SpeedKmHtoTick << " km/h    FROM: "
    //           << oldTargetSpeed / SpeedKmHtoTick << " km/h" << std::endl;

    if(train.state.targetSpeed > 0.0 && train.state.waitToStart_ms == 0 &&
        (train.state.speed == 0.0 || oldTargetSpeed == 0.0))
    {
      // When train was stopped or about to stop and signal becomes clear, add a bit of delay
      train.state.waitToStart_ms = TrainState::SecondsWaitFromStandstill * 1000.0;
    }
    else if(train.state.targetSpeed < train.state.speed)
    {
      train.state.waitToStart_ms = 0.0;
    }

    train.state.speedOrDirectionChanged = true;
  }
}

void Simulator::setTrainMode(Train &train, TrainState::Mode mode)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica && train.state.mode != mode)
  {
    boost::asio::post(
      m_networkContext, [this, trainId=train.trainId, mode]()
      {
        send(SimulatorProtocol::TrainRequestSetMode(trainId, mode));
      });
    return;
  }

  if(!train.isPowered)
    mode = TrainState::Mode::Manual;

  if(train.state.mode != mode)
  {
    train.state.mode = mode;
    train.state.nextSignal.dirty = true;
    train.state.prevSignal.dirty = true;
    train.state.speedOrDirectionChanged = true;

    train.state.resetSignalSpeeds();

    if(train.state.mode == TrainState::Mode::Manual)
    {
      // Reset acceleration and signals
      train.state.targetSpeed = train.state.speed;
      train.state.isOnStationStop = false;
      clearTrainSignalCache(m_stateData, train, false);
    }
  }
}

void Simulator::applyTrainSpeedDelta(Train &train, float delta)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    const auto step = delta > 0 ?
                          SimulatorProtocol::TrainRequestChangeSpeed::IncreaseStep :
                          SimulatorProtocol::TrainRequestChangeSpeed::DecreaseStep;
    boost::asio::post(
      m_networkContext, [this, trainId=train.trainId, step]()
      {
        send(SimulatorProtocol::TrainRequestChangeSpeed(trainId, step));
      });
    return;
  }

  const float speed = std::clamp(train.state.speed + delta, 0.0f, train.speedMax);
  setTrainSpeed(train, speed);
}

void Simulator::stopTrainAndSetManual(Train &train)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    boost::asio::post(
      m_networkContext, [this, trainId=train.trainId]()
      {
        const auto step = SimulatorProtocol::TrainRequestChangeSpeed::Stop;
        send(SimulatorProtocol::TrainRequestChangeSpeed(trainId, step));
      });
    return;
  }

  setTrainMode(train, Simulator::TrainState::Mode::Manual);
  setTrainSpeed(train, 0.0f, true);
}

void Simulator::doSetTurnoutState(const TurnoutState::State newState, TurnoutState &turnoutState, size_t turnoutIdx)
{
  if(newState == turnoutState.state)
    return;
  // Turnout changed state!
  turnoutState.state = newState;

  for(quint32 trainId : turnoutState.watchingTrainIDs)
  {
    Train &train = m_stateData.trains.at(trainId);
    train.state.nextSignal.dirty = true;
    train.state.prevSignal.dirty = true;
  }

  boost::asio::post(
      m_networkContext, [this, state=turnoutState.state,
       turnoutIdx]()
      {
        // Cherry pick state
        m_networkStateData.turnouts[turnoutIdx].state = state;

        uint8_t val = 0;
        if(state == TurnoutState::State::Thrown)
          val = 2;
        else if(state == TurnoutState::State::Closed)
          val = 1;

        const auto &turnout = staticData.turnouts.at(turnoutIdx);

        auto msg = SimulatorProtocol::AccessorySetState(turnout.channel, turnout.addresses[0], val);
        send(msg);
        sendReplicas(msg);
      });
}

void Simulator::stopAllTrains()
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  for(auto& it : m_stateData.trains)
  {
    auto &train = it.second;
    setTrainSpeed(train, 0.0f, true);
    setTrainMode(train, TrainState::Mode::Manual);
  }
}

bool Simulator::segmentOccupied(size_t segmentIdx) const
{
  if(segmentIdx >= staticData.trackSegments.size())
    return false;

  const auto& segment = staticData.trackSegments.at(segmentIdx);
  if(!segment.hasSensor())
    return false;

  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  const auto &sensor = m_stateData.sensors.at(segment.sensor.index);
  return sensor.value;
}

void Simulator::requestTurnoutState(size_t segmentIndex, TurnoutState::State state, bool immediate)
{
  using Type = TrackSegment::Type;

  if(segmentIndex >= staticData.trackSegments.size() || staticData.trackSegments[segmentIndex].turnout.index == invalidIndex)
    return;

  const auto& segment = staticData.trackSegments[segmentIndex];

  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    // Send request to master
    boost::asio::post(
        m_networkContext, [this, state, turnoutIdx=segment.turnout.index]()
        {
          uint8_t val = 0;
          if(state == TurnoutState::State::Thrown)
            val = 2;
          else if(state == TurnoutState::State::Closed)
            val = 1;

          const auto& turnout = staticData.turnouts[turnoutIdx];

          auto msg = SimulatorProtocol::AccessorySetState(turnout.channel, turnout.addresses[0], val);
          send(msg);
        });
    return;
  }

  auto& turnoutState = m_stateData.turnouts[segment.turnout.index];

  uint8_t newRequestedPosition = TurnoutState::MiddlePosition; // Middle, unknown
  if(state == TurnoutState::State::Closed)
    newRequestedPosition = TurnoutState::ClosedPosition;
  else if(state == TurnoutState::State::Thrown)
    newRequestedPosition = TurnoutState::ThrownPosition;

  if(newRequestedPosition == turnoutState.requestedPosition)
    return; // Already in requested state

  if(segment.type == Type::Turnout || segment.type == Type::TurnoutCurved)
  {
    const bool wasCompleted = (turnoutState.currentPosition == turnoutState.requestedPosition);
    turnoutState.requestedPosition = newRequestedPosition;

    if(immediate)
    {
      turnoutState.currentPosition = turnoutState.requestedPosition;

      if(!wasCompleted)
        setTurnoutIsMoving(false);

      doSetTurnoutState(state, turnoutState, segment.turnout.index);
    }
    else
    {
      if(wasCompleted)
        setTurnoutIsMoving(true); // Start turnout movement if needed
    }
  }
  else if(segment.type == Type::Turnout3Way)
  {



    return; // FIXME
  }
}

void Simulator::toggleTurnoutState(size_t segmentIndex, bool setUnknown)
{
  using Type = TrackSegment::Type;
  using State = TurnoutState::State;

  if(segmentIndex < staticData.trackSegments.size() && staticData.trackSegments[segmentIndex].turnout.index != invalidIndex)
  {
    const auto& segment = staticData.trackSegments[segmentIndex];

    Simulator::TurnoutState::State state;
    {
      std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
      state = m_stateData.turnouts[segment.turnout.index].state;
    }

    if(segment.type == Type::Turnout || segment.type == Type::TurnoutCurved)
    {
      if(setUnknown)
        state = State::Unknown;
      else
        state = (state != State::Closed) ? State::Closed : State::Thrown;
    }
    else if(segment.type == Type::Turnout3Way)
    {
      if(setUnknown)
      {
        state = State::Unknown;
      }
      else if(state == State::Closed)
      {
        state = State::ThrownLeft;
      }
      else if(state == State::ThrownLeft)
      {
        state = State::ThrownRight;
      }
      else
      {
        assert(state == State::ThrownRight || state == State::Unknown);
        state = State::Closed;
      }
    }

    requestTurnoutState(segmentIndex, state);
  }
}

void Simulator::send(const SimulatorProtocol::Message& message)
{
  assert(isNetworkThread());

  for(const auto& connection : m_connections)
  {
    connection->send(message);
  }
}

void Simulator::sendTo(size_t toConnId, const SimulatorProtocol::Message &message)
{
  assert(isNetworkThread());

  for(const auto& connection : m_connections)
  {
    if(toConnId != connection->connectionId())
      continue;
    connection->send(message);
    return;
  }

  // Ensure valid toConnId
  assert(false);
}

void Simulator::sendToReplica(size_t toConnId, const SimulatorProtocol::Message &message)
{
  assert(isNetworkThread());

  for(const auto& connection : m_repliacas)
  {
    if(toConnId != connection->connectionId())
      continue;
    connection->send(message);
    return;
  }

  // Ensure valid toConnId
  assert(false);
}

void Simulator::sendReplicas(const SimulatorProtocol::Message& message)
{
  assert(isNetworkThread());

  if(getSimModeUnlocked() != SimMode::Master)
    return;

  for(const auto& connection : m_repliacas)
  {
    connection->send(message);
  }
}

void Simulator::sendReplicas(QByteArray &buf)
{
  assert(isNetworkThread());

  if(getSimModeUnlocked() != SimMode::Master)
    return;

  SimulatorProtocol::Message& message = *reinterpret_cast<SimulatorProtocol::Message *>(buf.data());
  message.setSize(buf.size());

  for(const auto& connection : m_repliacas)
  {
    connection->send(message);
  }
}

void Simulator::sendToMaster(QByteArray &buf)
{
  assert(isNetworkThread());

  if(getSimModeUnlocked() != SimMode::Replica)
    return;

  SimulatorProtocol::Message& message = *reinterpret_cast<SimulatorProtocol::Message *>(buf.data());
  message.setSize(buf.size());

  for(const auto& connection : m_connections)
  {
    if(connection->send(message))
      break;
  }
}

void Simulator::receive(const SimulatorProtocol::Message& message, size_t fromConnId)
{
  assert(isNetworkThread());

  using namespace SimulatorProtocol;

  const bool isReplica = getSimModeNetwork() == SimMode::Replica;
  const bool isMaster = getSimModeNetwork() == SimMode::Master;

  switch(message.opCode)
  {
  case OpCode::Power:
  {
    const auto &m = static_cast<const Power &>(message);
    boost::asio::post(
        m_coreContext, [this, isReplica, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
          const bool wasPowerOn = m_stateData.powerOn;

          // NOTE: for replica we set local value here matching master value
          m_stateData.powerOn = m.powerOn;

          if(!isReplica && wasPowerOn != m_stateData.powerOn)
          {
            boost::asio::post(
                m_networkContext, [this, m=m]()
                {
                  send(m);
                  sendReplicas(m);
                });
          }
        });
    break;
  }
  case OpCode::LocomotiveSpeedDirection:
  {
    if(isReplica)
      break;

    boost::asio::post(
        m_coreContext, [this, message=message]()
        {
          const auto &m = static_cast<const LocomotiveSpeedDirection &>(message);
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
          for(auto &it : m_stateData.trains)
          {
            Train &train = it.second;

            if(m.address == train.address && (m.protocol == train.protocol || train.protocol == DecoderProtocol::None))
            {
              const float speed = std::clamp(train.speedMax * (m.speed / 255.0f), 0.0f, train.speedMax);
              const bool reverse = m.direction == Direction::Reverse;

              auto &trainState = train.state;
              if(trainState.speed != speed || trainState.reverse != reverse)
              {
                setTrainSpeed(train, speed);
                setTrainDirection(train, reverse);
              }
              break;
            }
          }
        });
    break;
  }
  case OpCode::AccessorySetState:
  {
    const auto& m = static_cast<const AccessorySetState&>(message);
    const auto count = staticData.trackSegments.size();
    for(size_t i = 0; i < count; ++i)
    {
      const auto& segment = staticData.trackSegments[i];
      if(segment.turnout.index == invalidIndex)
        continue;

      const auto& turnout = staticData.turnouts[segment.turnout.index];
      if(turnout.channel != m.channel)
        continue;

      if((segment.type == Simulator::TrackSegment::Type::Turnout || segment.type == Simulator::TrackSegment::Type::TurnoutCurved) &&
          m.address == turnout.addresses[0])
      {

        Simulator::TurnoutState::State newState = Simulator::TurnoutState::State::Unknown;
        if(m.getState() == 1)
        {
          newState = Simulator::TurnoutState::State::Closed;
        }
        else if(m.getState() == 2)
        {
          newState = Simulator::TurnoutState::State::Thrown;
        }

        const bool isImmediate = m.isImmediate();

        boost::asio::post(
            m_coreContext, [this, i, isReplica, newState=newState, turnoutIdx=segment.turnout.index, isImmediate]()
            {
              if(isReplica)
              {
                // Obey to master and set turnout position
                std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
                auto& turnoutState = m_stateData.turnouts[turnoutIdx];
                turnoutState.state = newState;
              }
              else
              {
                requestTurnoutState(i, newState, isImmediate);
              }
            });
        break;
      }
      else if(segment.type == Simulator::TrackSegment::Type::Turnout3Way && (m.address == turnout.addresses[0] || m.address == turnout.addresses[1]))
      {
        // Left has preference over right, if other behavior is required add it to the json config as option.
        //
        //  |/ = 2nd address
        // \|  = 1st address
        //
        // TurnoutState::coils = 0 0 0 0 RC RT LC LT

        // TODO: implement
        break;

        std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
        auto& turnoutState = m_stateData.turnouts[segment.turnout.index];
        if(m.address == turnout.addresses[0])
        {
          turnoutState.coils &= 0x0C;
          turnoutState.coils |= (m.state & 0x03);
        }
        else if(m.address == turnout.addresses[1])
        {
          turnoutState.coils &= 0x03;
          turnoutState.coils |= (m.state & 0x03) << 2;
        }

        switch(turnoutState.coils)
        {
        case 0x0A:
          turnoutState.state = Simulator::TurnoutState::State::Closed;
          break;

        case 0x05:
        case 0x09:
          turnoutState.state = Simulator::TurnoutState::State::ThrownLeft;
          break;

        case 0x06:
          turnoutState.state = Simulator::TurnoutState::State::ThrownRight;
          break;
        }
        break;
      }
    }
    break;
  }
  case OpCode::SensorChanged:
  {
    if(!isReplica)
      break; // Only sent by Master simulator

    const auto& m = static_cast<const SensorChanged&>(message);

    auto chan = staticData.sensorChannelMap.find(m.channel);
    if(chan == staticData.sensorChannelMap.end())
      break;

    auto sensor = chan->second.find(m.address);
    if(sensor == chan->second.end())
      break;

    boost::asio::post(
        m_coreContext, [this, sensorIdx=sensor->second, newAxleCount=m.axleCount, newValue=m.value]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto &sensorState = m_stateData.sensors[sensorIdx];
          const auto sensorType = staticData.sensors.at(sensorIdx).type;
          if(sensorType == Sensor::Type::PositionSensor)
          {
            sensorState.curTime = sensorState.maxTime;
          }
          else if(sensorType == Sensor::Type::AxleCounter)
          {
            sensorState.axleCount += newAxleCount;
            sensorState.curTime = sensorState.axleCount > 0 ? sensorState.maxTime : -sensorState.maxTime;
          }

          sensorState.value = newValue;
        });
    break;
  }
  case OpCode::SensorBatchState:
  {
    if(!isReplica)
      break; // Only sent by Master simulator

    const auto& m = static_cast<const SensorBatchState&>(message);

    if(m.channel != 0)
      break; // Protocol error

    const size_t firstIdx = m.batchIdx * 16;
    const size_t idxCount = std::min(firstIdx + 16, staticData.sensors.size());

    boost::asio::post(
        m_coreContext, [this, firstIdx, idxCount, newState=m.state]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          for(size_t idx = firstIdx; idx < idxCount; idx++)
          {
            const auto& sensor = staticData.sensors[idx];

            // Axle counter sensor send diffs so there is no point in querying them
            if(sensor.type == Sensor::Type::AxleCounter)
            {
              continue;
            }

            auto& sensorState = m_stateData.sensors[idx];
            sensorState.value = bitRead(newState, idx - firstIdx);
          }
        });
    break;
  }
  case OpCode::Handshake:
  case OpCode::HandshakeResponse:
  case OpCode::ReplicaGreeting:
    break; // handled by SimulatorConnection already
  case OpCode::SignalSetState:
  {
    const auto& m = static_cast<const SignalSetState&>(message);
    for(const auto &it : m_networkStateData.mainSignals)
    {
      const MainSignal &signalCopy = it.second;
      if(!isReplica && signalCopy.ownerConnectionId != fromConnId)
        continue;

      if(signalCopy.address != m.address || signalCopy.channel != m.channel)
        continue;

      if(getSimModeNetwork() == SimMode::Master)
      {
        sendReplicas(m);
      }

      boost::asio::post(
          m_coreContext, [this, m=m, signalName=signalCopy.name]()
          {
            std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
            MainSignal &mainSignal = m_stateData.mainSignals.at(signalName);

            for(size_t i = 0; i < mainSignal.lights.size(); i++)
            {
              if(i < (sizeof(m.lights) / sizeof(m.lights[0])))
              {
                mainSignal.lights[i].color = MainSignal::Light::Color(m.lights[i].color);
                mainSignal.lights[i].state = MainSignal::State(m.lights[i].state);
              }
              else
              {
                mainSignal.lights[i].color = MainSignal::Light::Color::Red;
                mainSignal.lights[i].state = MainSignal::State::Off;
              }
            }

            using FixedLimit = MainSignal::FixedLimit;
            using RappelState = MainSignal::RappelState;

            mainSignal.maxCurrentSpeed = m.getSignalSpeed();
            mainSignal.maxNextSpeed = m.getNextSignalSpeed();

            mainSignal.setArrowLightOn(m.isArrowLightOn());
            mainSignal.directionIndicatorText = m.directionIndication;
            mainSignal.rappelState = RappelState(m.rappelState);
            mainSignal.setAdvanceSignalState(MainSignal::State(m.getAdvanceSignalState()));

            const bool rappelLimit = mainSignal.hasRappel &&
                                     (mainSignal.rappelState != RappelState::Off ||
                                      mainSignal.maxCurrentSpeed == MainSignal::SpecialSpeeds::VARIABLE_30_60_100);

            if(mainSignal.fixedLimit != FixedLimit::NoLimit || rappelLimit)
            {
              uint16_t rappelSpeed = 30;
              if(mainSignal.fixedLimit == FixedLimit::Limit60 || mainSignal.rappelState == RappelState::OneLine_60)
                rappelSpeed = 60;
              if(mainSignal.rappelState == RappelState::OneLine_60)
                rappelSpeed = 60;
              else if(mainSignal.rappelState == RappelState::TwoLines_100)
                rappelSpeed = 100;

              mainSignal.maxCurrentSpeed = std::min(mainSignal.maxCurrentSpeed, rappelSpeed);
            }
          });
      break;
    }
    break;
  }
  case OpCode::AuxSignalSetState:
  {
    const auto& m = static_cast<const AuxSignalSetState&>(message);
    for(const auto &it : m_networkStateData.auxSignals)
    {
      const AuxSignal &signalCopy = it.second;
      if(!isReplica && signalCopy.ownerConnectionId != fromConnId)
        continue;

      if(signalCopy.address != m.address || signalCopy.channel != m.channel)
        continue;

      if(getSimModeNetwork() == SimMode::Master)
      {
        sendReplicas(m);
      }

      boost::asio::post(
          m_coreContext, [this, m=m, signalName=signalCopy.name]()
          {
            std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
            AuxSignal &auxSignal = m_stateData.auxSignals.at(signalName);

            auxSignal.mLights = m.lights;
            auxSignal.mPosition = m.position;
          });
      break;
    }
    break;
  }
  case OpCode::OwnSignal:
  {
    const auto& m = static_cast<const OwnSignal&>(message);

    if(m.isMain)
    {
      for(const auto &it : m_networkStateData.mainSignals)
      {
        const MainSignal &signalCopy = it.second;
        if(signalCopy.address != m.address || signalCopy.channel != m.channel)
          continue;

        boost::asio::post(
            m_coreContext, [this, signalName=signalCopy.name, fromConnId]()
            {
              std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
              MainSignal &mainSignal = m_stateData.mainSignals.at(signalName);

              if(mainSignal.ownerConnectionId != invalidIndex)
                return;
              mainSignal.ownerConnectionId = fromConnId;
            });
        break;
      }
    }
    else
    {
      for(const auto &it : m_networkStateData.auxSignals)
      {
        const AuxSignal &signalCopy = it.second;
        if(signalCopy.address != m.address || signalCopy.channel != m.channel)
          continue;

        boost::asio::post(
            m_coreContext, [this, signalName=signalCopy.name, fromConnId]()
            {
              std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
              AuxSignal &auxSignal = m_stateData.auxSignals.at(signalName);

              if(auxSignal.ownerConnectionId != invalidIndex)
                return;
              auxSignal.ownerConnectionId = fromConnId;
            });
        break;
      }
    }
    break;
  }
  case OpCode::RequestChannel:
  {
    const auto& m = static_cast<const RequestChannel&>(message);

    if(staticData.sensorChannelMap.find(m.channel) == staticData.sensorChannelMap.end())
      break;

    for(const std::shared_ptr<SimulatorConnection>& connection : m_connections)
    {
      if(connection->connectionId() != fromConnId)
        continue;

      // Subscribe client to this channel
      if(!vec_contains(connection->subscribedChannels, m.channel))
        connection->subscribedChannels.push_back(m.channel);

      const auto& channelMap = staticData.sensorChannelMap.at(m.channel);

      size_t batchIdx = 0;
      bool sensorFound = false;

      uint16_t stateBuf = 0;
      std::map<size_t, size_t>::const_iterator it;

      while(true)
      {
        const size_t firstAddress = batchIdx * 16 + 1;
        const size_t lastAddress = (batchIdx + 1) * 16;

        it = channelMap.lower_bound(firstAddress);
        stateBuf = 0;
        sensorFound = false;

        while(it != channelMap.end())
        {
          if(it->first > lastAddress)
            break;

          const auto& sensor = staticData.sensors[it->second];

          // Axle counter sensor send diffs so there is no point in querying them
          if(sensor.type == Sensor::Type::AxleCounter)
          {
            it++;
            continue;
          }

          const auto& sensorState = m_networkStateData.sensors[it->second];
          if(sensorState.value)
            bitSet(stateBuf, sensor.address - firstAddress);

          sensorFound = true;
          it++;
        }

        if(sensorFound)
        {
          connection->send(SimulatorProtocol::SensorBatchState(m.channel, batchIdx, m_networkStateData.powerOn ? stateBuf : 0));
        }

        if(it == channelMap.end())
          break;

        // Go to next batch
        batchIdx = it->first / 16;
      }
      break;
    }
    break;
  }
  case OpCode::SensorEncodingChanged:
  {
    const auto& m = static_cast<const SensorEncodingChanged&>(message);
    if(m.encoding > uint8_t(SensorState::Encoding::Code270))
      break;

    auto chan = staticData.sensorChannelMap.find(m.channel);
    if(chan == staticData.sensorChannelMap.end())
      break;

    auto sensor = chan->second.find(m.address);
    if(sensor == chan->second.end())
      break;

    if(getSimModeNetwork() == SimMode::Master)
    {
      for(const auto& connection : m_repliacas)
      {
        connection->send(m);
      }
    }

    boost::asio::post(
        m_coreContext, [this, sensorIdx=sensor->second, encoding=m.encoding]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
          m_stateData.sensors[sensorIdx].encoding = SensorState::Encoding(encoding);
        });
    break;
  }
  case OpCode::OwnSpawn:
  {
    const auto& m = static_cast<const OwnSpawn&>(message);

    boost::asio::post(
        m_coreContext, [this, fromConnId, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto it = m_stateData.spawns.find(m.address);
          if(it == m_stateData.spawns.end())
            return;

          SpawnState &spawnState = it->second;
          if(spawnState.ownerConnectionId != invalidIndex)
            return;

          spawnState.ownerConnectionId = fromConnId;
          spawnState.state = (spawnState.waitingResetCount > 0) ?
                                 SpawnState::State::WaitingReset :
                                 SpawnState::State::Ready;

          boost::asio::post(
              m_networkContext, [this, spawnState=spawnState, address=m.address, fromConnId]()
              {
                // Cherry pick state
                m_networkStateData.spawns.at(address) = spawnState;

                uint8_t state = SpawnStateChange::Reset;
                if(spawnState.state == SpawnState::State::Ready)
                  state = SpawnStateChange::Ready;
                else if(spawnState.state == SpawnState::State::WaitingReset)
                  state = SpawnStateChange::WaitingReset;
                sendTo(fromConnId, SimulatorProtocol::SpawnStateChange(address, state));
              });
        });
    break;
  }
  case OpCode::SpawnStateChange:
  {
    const auto& m = static_cast<const SpawnStateChange&>(message);

    boost::asio::post(
        m_coreContext, [this, fromConnId, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto it = m_stateData.spawns.find(m.address);
          if(it == m_stateData.spawns.end())
            return;

          SpawnState &spawnState = it->second;
          if(spawnState.ownerConnectionId != fromConnId)
            return;

          if(spawnState.state == SpawnState::State::WaitingReset && spawnState.waitingResetCount == SpawnState::MAX_WAITING_RESET)
          {
            if(m.state == SpawnStateChange::Reset) // Start resetting
              spawnState.waitingResetCount = SpawnState::START_WAITING_RESET;
            return;
          }

          if(spawnState.state != SpawnState::State::Ready || m.state != SpawnStateChange::RequestActivate)
            return;

          if(getSimModeUnlocked() != SimMode::Replica)
            doSpawnTrain(m.address);

          boost::asio::post(
              m_networkContext, [this, spawnState=spawnState, address=m.address, fromConnId]()
              {
                // Cherry pick state
                m_networkStateData.spawns.at(address) = spawnState;

                uint8_t state = SpawnStateChange::Reset;
                if(spawnState.state == SpawnState::State::Ready)
                  state = SpawnStateChange::Ready;
                else if(spawnState.state == SpawnState::State::WaitingReset)
                  state = SpawnStateChange::WaitingReset;
                sendTo(fromConnId, SimulatorProtocol::SpawnStateChange(address, state));
              });
        });
    break;
  }
  case OpCode::TrainSpeedFactorChanged:
  {
    const auto& m = static_cast<const TrainSpeedFactorChanged&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          if(getSimModeUnlocked() == SimMode::Replica)
          {
            // If replica, just set the value received
            m_stateData.trainSpeedFactor = m.trainSpeedFactor;
            return;
          }

          // If master propagate
          setTrainSpeedFactor(m.trainSpeedFactor);
        });
    break;
  }
  case OpCode::TrainRequestSetMode:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const TrainRequestSetMode&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto train = m_stateData.getTrainById(m.getTrainId());
          if(!train)
            return;

          setTrainMode(*train, m.mode);
        });
    break;
  }
  case OpCode::TrainRequestSetDirection:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const TrainRequestSetDirection&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto train = m_stateData.getTrainById(m.getTrainId());
          if(!train)
            return;

          setTrainDirection(*train, m.reverse);
        });
    break;
  }
  case OpCode::TrainRequestChangeSpeed:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const TrainRequestChangeSpeed&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto train = m_stateData.getTrainById(m.getTrainId());
          if(!train)
            return;

          // NOTE: match what SimulatorView does
          switch (m.step)
          {
          case TrainRequestChangeSpeed::Stop:
          {
            setTrainMode(*train, Simulator::TrainState::Mode::Manual);
            setTrainSpeed(*train, 0.0f, true);
            break;
          }
          case TrainRequestChangeSpeed::IncreaseStep:
          {
            applyTrainSpeedDelta(*train, train->speedMax / 20);
            break;
          }
          case TrainRequestChangeSpeed::DecreaseStep:
          {
            applyTrainSpeedDelta(*train, - train->speedMax / 20);
            break;
          }
          default:
            break;
          }
        });
    break;
  }
  case OpCode::TrainRequestCoupleSplit:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const TrainRequestCoupleSplit&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          auto train = m_stateData.getTrainById(m.getTrainId());
          if(!train)
            return;

          quint32 otherTrainId = invalidTrainId;

          // NOTE: match what SimulatorView does
          switch (m.coupleOp)
          {
          case TrainRequestCoupleSplit::CoupleFront:
          {
            coupleTrain(train->trainId, true, otherTrainId);
            break;
          }
          case TrainRequestCoupleSplit::CoupleRear:
          {
            coupleTrain(train->trainId, false, otherTrainId);
            break;
          }
          case TrainRequestCoupleSplit::SplitFront:
          {
            splitTrain(train->trainId, m.getVehicleIdx(),
                       true, otherTrainId);
            break;
          }
          case TrainRequestCoupleSplit::SplitRear:
          {
            splitTrain(train->trainId, m.getVehicleIdx(),
                       false, otherTrainId);
            break;
          }
          }
        });
    break;
  }
  case OpCode::TrainRequestRemove:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const TrainRemoved&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          removeTrain(m.getTrainId(), true);
        });
    break;
  }
  case OpCode::TrainRemoved:
  {
    if(!isReplica)
      break;

    const auto& m = static_cast<const TrainRemoved&>(message);
    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          removeTrain(m.getTrainId(), true, m.getNextSelectedTrainId());
        });
    break;
  }
  case OpCode::TrainRequestAddReply:
  {
    if(!isReplica)
      break;

    const auto& m = static_cast<const TrainRequestAddReply&>(message);

    boost::asio::post(
        m_coreContext, [this, m=m]()
        {
          if(trainTypeInterface)
            trainTypeInterface->onAddTrainReply(m.getTrainId(), m.getToken(), m.isError);
        });
    break;
  }
  case OpCode::SetMasterFollowTrain:
  {
    if(!isMaster)
      break;

    const auto& m = static_cast<const SetMasterFollowTrain&>(message);

    boost::asio::post(
        m_coreContext, [this, trainId=m.getTrainId()]()
        {
          if(trainTypeInterface)
            trainTypeInterface->onSetMasterFollowTrain(trainId);
        });
    break;
  }
  case OpCode::TrainAdded:
  case OpCode::TrainStateRefresh:
  case OpCode::TrainChangedVehicles:
  {
    // Variable size messages sent by Master
    if(!isReplica)
      break;

    receiveVariableSize(message, fromConnId);
    break;
  }
  case OpCode::TrainRequestAdd:
  {
    // Variable size messages sent by replica
    if(!isMaster)
      break;

    receiveVariableSize(message, fromConnId);
    break;
  }
  }
}

void Simulator::receiveVariableSize(const SimulatorProtocol::Message& message, size_t fromConnId)
{
  using namespace SimulatorProtocol;

  QByteArray buf = QByteArray::fromRawData(reinterpret_cast<const char *>(&message) + sizeof(SimulatorProtocol::Message),
                                           message.size() - sizeof(SimulatorProtocol::Message));
  QDataStream stream(buf);
  stream.setByteOrder(QDataStream::LittleEndian);

  switch(message.opCode)
  {
  case OpCode::TrainAdded:
  case OpCode::TrainChangedVehicles:
  {
    bool isNewTrain = (message.opCode == OpCode::TrainAdded);

    std::vector<Simulator::Train::VehicleItemFull> vehicles;
    std::string trainName;
    quint32 trainId = 0;
    Simulator::TrainState trainState;
    float speedMax = 0.0f;

    deserializeTrain(stream, vehicles, trainName,
                     trainId, trainState, speedMax,
                     true);

    boost::asio::post(
        m_coreContext, [this, isNewTrain, vehicles, trainName, trainId, trainState, speedMax]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          Train *train = m_stateData.getTrainById(trainId);
          if(train && (isNewTrain || vehicles.empty()))
          {
            removeTrain(trainId, true, trainId);
            train = nullptr;
          }

          if(vehicles.empty())
            return;

          std::unique_ptr<Train> guard;

          if(!train)
          {
            train = new Train;
            guard.reset(train);
          }

          train->address = 3;
          train->protocol = DecoderProtocol::DCCShort;
          train->trainId = trainId;
          train->name = trainName;
          train->speedMax = speedMax;
          train->state = trainState;

          std::unordered_set<quint32> vehiclesToRemove;

          if(train->vehicles.size() < vehicles.size())
            train->vehicles.reserve(vehicles.size());
          else
          {
            size_t nVehiclesToRemove = train->vehicles.size() - vehicles.size();
            while (nVehiclesToRemove > 0)
            {
              Vehicle *vehicle = m_stateData.getVehicleById(train->vehicles.back().vehicleId);
              assert(vehicle);
              vehicle->activeTrainId = invalidTrainId;
              vehiclesToRemove.insert(vehicle->vehicleId);
              train->vehicles.pop_back();
              nVehiclesToRemove--;
            }
          }

          size_t idx = 0;
          for(const Simulator::Train::VehicleItemFull &v : vehicles)
          {
            Vehicle *result = m_stateData.getVehicleById(v.vehicle.vehicleId);
            if(result)
            {
              result->typeIdx = v.vehicle.typeIdx;
              result->color = v.vehicle.color;
              result->length = v.vehicle.length;
            }
            else
            {
              result = addVehicle(v.vehicle.length, v.vehicle.color, v.vehicle.typeIdx,
                                  v.vehicle.vehicleId);
            }

            if(vehiclesToRemove.contains(result->vehicleId))
              vehiclesToRemove.erase(vehiclesToRemove.find(result->vehicleId));

            if(trainTypeInterface && result->typeIdx != invalidIndex)
              trainTypeInterface->fillVehicleProperties(result);

            const auto oldState = result->state;
            result->state = v.vehicle.state;
            updateReplicaTracking(result, oldState);

            if(guard.get() || idx >= train->vehicles.size()) // If guard has Train, it's new
              train->vehicles.push_back({result->vehicleId, v.reversed});
            else
            {
              Train::VehicleItem &item = train->vehicles[idx];
              if(item.vehicleId != result->vehicleId)
              {
                vehiclesToRemove.insert(item.vehicleId);
                item.vehicleId = result->vehicleId;
              }

              item.reversed = v.reversed;
            }
            train->length += result->length + (idx == 0 ? 0.0f : staticData.trainCouplingLength);
            idx++;
          }

          if(idx == 0)
            return;

          for(const Simulator::Train::VehicleItem &v : train->vehicles)
          {
            Vehicle *vehicle = m_stateData.getVehicleById(v.vehicleId);
            if(vehicle->activeTrainId != invalidTrainId && vehicle->activeTrainId != train->trainId)
            {
              Train &otherTrain = m_stateData.trains.at(vehicle->activeTrainId);
              auto &otherVehicles = otherTrain.vehicles;
              if(otherVehicles.size() == 1 && otherVehicles.front().vehicleId == v.vehicleId)
              {
                // After removing this vehicle, train will be empty
                removeTrain(vehicle->activeTrainId, false, train->trainId);
                vehicle->activeTrainId = invalidTrainId;
              }
              else
              {
                for(auto it = otherVehicles.begin(); it != otherVehicles.end(); it++)
                {
                  if(it->vehicleId == v.vehicleId)
                  {
                    otherVehicles.erase(it);
                    vehicle->activeTrainId = invalidTrainId;
                    break;
                  }
                }
              }
            }

            vehicle->activeTrainId = train->trainId;
          }

          for(quint32 vehicleId : vehiclesToRemove)
          {
            auto vehicle = m_stateData.getVehicleById(vehicleId);
            if(vehicle)
              removeVehicle(vehicle);
          }
          vehiclesToRemove.clear();

          if(trainTypeInterface)
          {
            trainTypeInterface->setupTrainSpeedPowered(this, train);
          }

          m_stateData.trains.insert({train->trainId, *train});
          onTrainAddedRemoved(true, train->trainId, invalidTrainId);
        });
    break;
  }
  case OpCode::TrainStateRefresh:
  {
    std::vector<Simulator::Train::VehicleItemFull> vehicles;
    std::string dummyTrainName; // Dummy when deserialize full = false
    quint32 trainId = 0;
    Simulator::TrainState trainState;
    float dummySpeedMax = 0.0f; // Dummy when deserialize full = false

    deserializeTrain(stream, vehicles, dummyTrainName,
                     trainId, trainState, dummySpeedMax,
                     false);

    boost::asio::post(
        m_coreContext, [this, vehicles, trainId, trainState]()
        {
          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          Train *train = m_stateData.getTrainById(trainId);
          if(!train)
            return;

          train->state.speed = trainState.speed;
          train->state.targetSpeed = trainState.targetSpeed;
          train->state.waitToStart_ms = trainState.waitToStart_ms;
          train->state.reverse = trainState.reverse;
          train->state.mode = trainState.mode;
          train->state.isOnStationStop = trainState.isOnStationStop;

          const size_t maxIdx = std::min(train->vehicles.size(), vehicles.size());
          for(size_t i = 0; i < maxIdx; i++)
          {
            Train::VehicleItem &item = train->vehicles[i];
            const Train::VehicleItemFull &source = vehicles.at(i);
            Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);

            item.reversed = source.reversed;
            const auto oldState = vehicle->state;
            vehicle->state = source.vehicle.state;
            updateReplicaTracking(vehicle, oldState);
          }
        });
    break;
  }
  case OpCode::TrainRequestAdd:
  {
    std::vector<Simulator::Train::VehicleItemFull> vehicles_;
    std::string trainName;
    size_t segmentIndex = invalidIndex;
    quint32 token = 0;
    float startPos = 0.0f;
    TrainPlacement placement = TrainPlacement::PlaceCenter;
    bool invertDir = false;

    deserializeTrainAddRequest(stream, trainName, vehicles_,
                               segmentIndex, token, startPos,
                               placement, invertDir);

    boost::asio::post(
        m_coreContext, [this, trainName, vehicles_=vehicles_,
         segmentIndex, token, startPos,
         placement, invertDir, fromConnId]()
        {
          std::vector<Simulator::Train::VehicleItemFull> vehicles = vehicles_; // TODO: it's captured const
          if(trainTypeInterface)
          {
            for(Train::VehicleItemFull &item : vehicles)
            {
              if(item.vehicle.typeIdx != invalidIndex)
                trainTypeInterface->fillVehicleProperties(&item.vehicle);
            }
          }

          if(trainName.empty() || segmentIndex == invalidIndex || segmentIndex >= staticData.trackSegments.size()
              || vehicles.empty())
          {
            boost::asio::post(
                m_networkContext, [this, fromConnId, token]()
                {
                  sendToReplica(fromConnId, SimulatorProtocol::TrainRequestAddReply(token, true));
                });
            return;
          }

          std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

          quint32 trainId = invalidTrainId;
          if(!addTrain(trainName, DecoderProtocol::DCCShort, 3,
                        vehicles, segmentIndex, trainId, startPos, placement, invertDir))
          {
            boost::asio::post(
                m_networkContext, [this, fromConnId, token]()
                {
                  sendToReplica(fromConnId, SimulatorProtocol::TrainRequestAddReply(token, true));
                });
            return;
          }

          // Success
          boost::asio::post(
              m_networkContext, [this, fromConnId, token, trainId]()
              {
                sendToReplica(fromConnId, SimulatorProtocol::TrainRequestAddReply(token, false, trainId));
              });
        });
    break;
  }
  default:
    break;
  }
}

void Simulator::setConnectionAsReplica(const std::shared_ptr<SimulatorConnection>& connection)
{
  if(auto it = std::find(m_connections.begin(), m_connections.end(), connection); it != m_connections.end())
  {
    // Move this connection to replicas list
    m_repliacas.push_back(connection->shared_from_this());
    m_connections.erase(it);

    // Sync power state
    connection->send(SimulatorProtocol::Power(m_networkStateData.powerOn));
    connection->send(SimulatorProtocol::TrainSpeedFactorChanged(m_networkStateData.trainSpeedFactor));
  }
}

void Simulator::removeConnection(const std::shared_ptr<SimulatorConnection>& connection, bool replica, bool remove)
{
  if(replica)
  {
    if(remove)
    {
      if(auto it = std::find(m_repliacas.begin(), m_repliacas.end(), connection); it != m_repliacas.end())
      {
        m_repliacas.erase(it);
      }
      else
      {
        return;
      }
    }

    connection->stop();

    boost::asio::post(
        m_coreContext, [this, conn=connection]()
        {
          onReplicaRemoved(conn);
        });
  }
  else
  {
    if(remove)
    {
      if(auto it = std::find(m_connections.begin(), m_connections.end(), connection); it != m_connections.end())
      {
        m_connections.erase(it);
      }
      else
      {
        return;
      }
    }

    connection->stop();

    if(getSimModeNetwork() == SimMode::Replica)
    {
      if(m_connections.empty())
      {
        broadcastAddresses.clear();
        doSendReplicaDiscover();
        doReceiveReplicaDiscover();

        boost::asio::post(
            m_coreContext, [this]()
            {
              onConnectionRemoved(invalidIndex, {});
              onReplicaRemoved({});
            });
      }
    }
    else
    {
      boost::asio::post(
          m_coreContext, [this, connId=connection->connectionId(), subscribedChannels=connection->subscribedChannels]()
          {
            onConnectionRemoved(connId, subscribedChannels);
          });
    }
  }
}

void Simulator::onConnectionRemoved(const size_t connId, const std::vector<uint16_t> &subscribedChannels)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  const bool isReplica = getSimModeUnlocked() == SimMode::Replica;
  if(isReplica && !m_connections.empty())
    return; // We still have another connection to master
  // TODO: does not seem to be called when master connection gets closed???

  // Turn off encoded tracks
  if(isReplica || connId == invalidIndex)
  {
    for(size_t idx = 0; idx < m_stateData.sensors.size(); idx++)
    {
      m_stateData.sensors[idx].encoding = SensorState::Encoding::CodeAbsent;
    }

    // Also power off world and reset train speed factor
    m_stateData.powerOn = false;
    m_stateData.trainSpeedFactor = 1.0f;

    // Now remove all trains
    {
      auto it = m_stateData.trains.begin();
      while(it != m_stateData.trains.end())
      {
        Train &train = it->second;
        destroyTrain(train, true);
        it = m_stateData.trains.erase(it);
      }
    }

    {
      while(!m_stateData.vehicles.empty())
      {
        removeVehicle(&m_stateData.vehicles.begin()->second);
      }
    }
  }
  else
  {
    // TODO: multiple connection could subscribe to same channel...
    // But only one must be allowed to set encoding

    // FIXME: subscribedChannels should only be accessed by network thread
    for(uint16_t channel : subscribedChannels)
    {
      auto channelMap = staticData.sensorChannelMap.find(channel);
      if(channelMap == staticData.sensorChannelMap.end())
        continue;

      for(auto sensor : channelMap->second)
      {
        if(staticData.sensors[sensor.second].channel != channel)
          continue;

        // TODO: update vehicles when they will listen to encoding
        m_stateData.sensors[sensor.second].encoding = SensorState::Encoding::CodeAbsent;
      }
    }
  }

  // Turn off owned signals
  for(auto &it : m_stateData.mainSignals)
  {
    MainSignal *s = &it.second;
    if(isReplica || s->ownerConnectionId != connId)
      continue;

    s->ownerConnectionId = invalidIndex;
    s->maxCurrentSpeed = MainSignal::SpecialSpeeds::INVALID;
    s->maxNextSpeed = MainSignal::SpecialSpeeds::INVALID;

    for(MainSignal::Light& l : s->lights)
    {
      l.color = MainSignal::Light::Color::Red;
      l.state = MainSignal::State::Off;
    }

    s->setArrowLightOn(false);
    s->setAdvanceSignalState(MainSignal::State::Off);
    s->directionIndicatorText = ' ';
    s->rappelState = MainSignal::RappelState::Off;
  }

  for(auto &it : m_stateData.auxSignals)
  {
    AuxSignal *s = &it.second;
    if(isReplica || s->ownerConnectionId != connId)
      continue;

    s->ownerConnectionId = invalidIndex;
    s->mLights = 0; // Turn lights off
    s->mPosition = 0.0f; // Go back to stop position
  }

  // Turn off owned spawns
  for(auto &it : m_stateData.spawns)
  {
    SpawnState &spawnState = it.second;
    if(isReplica || spawnState.ownerConnectionId != connId)
      continue;

    spawnState.ownerConnectionId = invalidIndex;
    spawnState.state = SpawnState::State::Inactive;
  }
}

void Simulator::onReplicaRemoved(const std::shared_ptr<SimulatorConnection>& connection)
{
  // No-op for now
  (void)connection;
}

void Simulator::accept()
{
  assert(isNetworkThread());

  m_acceptor.async_accept(
      [this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket)
      {
        if(!ec)
        {
          lastConnectionId++;
          if(lastConnectionId == invalidIndex)
            lastConnectionId = 0;

          m_connections.emplace_back(std::make_shared<SimulatorConnection>(
                                         shared_from_this(), std::move(socket),
                                         lastConnectionId))->start();
          accept();
        }
      });
}

constexpr char RequestMessage[] = {'s', 'i', 'm', '?'};
constexpr char ResponseMessage[] = {'s', 'i', 'm', '!'};
constexpr char ReplicaRequestMessage[] = {'r', 'e', 'p', '?'};

void Simulator::doReceive()
{
  assert(isNetworkThread());

  m_socketUDP.async_receive_from(
      boost::asio::buffer(m_udpBuffer),
      m_remoteEndpoint,
      [this](const boost::system::error_code& ec, std::size_t bytesReceived)
      {
        if(!ec)
        {
          const char *recvMsg = reinterpret_cast<char*>(m_udpBuffer.data());

          bool isRequest = false;

          if(getSimModeNetwork() != SimMode::Replica)
            isRequest = (bytesReceived >= sizeof(RequestMessage) &&
                         std::memcmp(recvMsg, &RequestMessage, sizeof(RequestMessage)) == 0);

          if(!isRequest && getSimModeNetwork() == SimMode::Master)
            isRequest = (bytesReceived >= sizeof(ReplicaRequestMessage) &&
                         std::memcmp(recvMsg, &ReplicaRequestMessage, sizeof(ReplicaRequestMessage)) == 0);

          if(isRequest)
          {
            if(!m_serverLocalHostOnly || m_remoteEndpoint.address().is_loopback())
            {
              // Send in big endian format
              uint16_t response[3] = {0, 0, htobe16(serverPort())};

              std::memcpy(&response, &ResponseMessage, sizeof(ResponseMessage));
              m_socketUDP.async_send_to(boost::asio::buffer(response, sizeof(response)), m_remoteEndpoint,
                                        [this](const boost::system::error_code& ec2, std::size_t bytesTransferred)
                                        {
                                          assert(!ec2 && bytesTransferred == 6);
                                          if(!ec2 && bytesTransferred == 6)
                                            doReceive();
                                        });
              return;
            }
          }
          doReceive();
        }
      });
}

void Simulator::doSendReplicaDiscover()
{
  assert(isNetworkThread());

  if(getSimModeNetwork() != SimMode::Replica)
    return;

  if(!m_connections.empty())
    return;

  m_replicaDiscoverSendTimer.expires_after(replicaDiscoverSendRate);
  m_replicaDiscoverSendTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          doSendReplicaDiscover();
        }
      });

  if(broadcastAddresses.empty() && trainTypeInterface)
    trainTypeInterface->getBroadcastAddresses(broadcastAddresses);

  for(const auto &address : broadcastAddresses)
  {
    boost::asio::ip::udp::endpoint udpServer(address, defaultPort);
    m_socketUDP.async_send_to(boost::asio::buffer(ReplicaRequestMessage, sizeof(ReplicaRequestMessage)),
                              udpServer,
                              [](const boost::system::error_code& /*ec2*/, std::size_t /*bytesTransferred*/)
                              {
                                // no-op
                              });
  }
}

void Simulator::doReceiveReplicaDiscover()
{
  assert(isNetworkThread());

  if(getSimModeNetwork() != SimMode::Replica)
    return;

  if(!m_connections.empty())
    return;

  m_socketUDP.async_receive_from(
      boost::asio::buffer(m_udpBuffer),
      m_remoteEndpoint,
      [this](const boost::system::error_code& ec, std::size_t bytesReceived)
      {
        if(!ec)
        {
          const char *recvMsg = reinterpret_cast<char*>(m_udpBuffer.data());

          bool isRequest = (bytesReceived >= (sizeof(ResponseMessage) + 2) &&
                            std::memcmp(recvMsg, &ResponseMessage, sizeof(ResponseMessage)) == 0);

          if(isRequest && m_connections.empty())
          {
            if(!m_serverLocalHostOnly || m_remoteEndpoint.address().is_loopback())
            {
              const uint16_t *response = reinterpret_cast< const uint16_t*>(recvMsg);
              const uint16_t remotePort = be16toh(response[2]);

              boost::asio::ip::tcp::endpoint masterServer;
              masterServer.port(remotePort);
              masterServer.address(m_remoteEndpoint.address());

              boost::system::error_code sockEc;
              boost::asio::ip::tcp::socket s(m_coreContext);
              s.connect(masterServer, sockEc);
              if(!sockEc)
              {
                m_connections.emplace_back(std::make_shared<SimulatorConnection>(
                                               shared_from_this(), std::move(s),
                                               lastConnectionId))->start();

                // Tell master we are a replica
                m_connections.back()->send(SimulatorProtocol::ReplicaGreeting());
              }
            }
          }

          if(m_connections.empty())
            doReceiveReplicaDiscover();
        }
      });
}

void Simulator::tick()
{
  assert(isCoreThread());

  m_tickTimer.expires_after(tickRate);
  m_tickTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          tick();
        }
      });

  onTick();

  {
    std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
    const auto start = std::chrono::high_resolution_clock::now();
    updateTrainPositions();
    updateSensors();
    updateSpawns();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    m_stateData.tickActive = duration.count() / 1e6f;
    m_stateData.tickLoad = static_cast<float>(100 * duration.count()) / static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(tickRate).count());
  }

  // Sync network state (deep copy)
  boost::asio::post(m_networkContext, [this, s=m_stateData]()
                    {
                      m_networkStateData = s;
                    });
}

void Simulator::handShakeSend()
{
  assert(isNetworkThread());

  m_handShakeSendTimer.expires_after(handShakeRateSend);
  m_handShakeSendTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          handShakeSend();
        }
      });

  const auto simMode = getSimModeNetwork();

  auto it = m_connections.begin();
  while(it != m_connections.end())
  {
    if(simMode != SimMode::Replica)
      (*it)->send(SimulatorProtocol::HandShake(false));
    it++;
  }

  it = m_repliacas.begin();
  while(it != m_repliacas.end())
  {
    (*it)->send(SimulatorProtocol::HandShake(false));
    it++;
  }
}

void Simulator::handShakeRecv()
{
  assert(isNetworkThread());

  m_handShakeRecvTimer.expires_after(handShakeRateRecv);
  m_handShakeRecvTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          handShakeRecv();
        }
      });

  auto it = m_connections.begin();
  while(it != m_connections.end())
  {
    if(!(*it)->handShakeResponseReceived())
    {
      std::shared_ptr<SimulatorConnection> conn = *it;
      it = m_connections.erase(it);

      std::cout << "Hit HANDSHAKE " << conn->remoteEndpoint() << std::endl;
      removeConnection(conn, false, false);
      continue;
    }

    (*it)->setHandShakeResponseReceived(false);
    it++;
  }

  it = m_repliacas.begin();
  while(it != m_repliacas.end())
  {
    if(!(*it)->handShakeResponseReceived())
    {
      std::shared_ptr<SimulatorConnection> conn = *it;
      it = m_repliacas.erase(it);

      removeConnection(conn, true, false);
      continue;
    }

    (*it)->setHandShakeResponseReceived(false);
    it++;
  }
}

void Simulator::syncSensorState()
{
  assert(isNetworkThread());

  if(getSimModeNetwork() == SimMode::Replica)
    return;

  m_syncSensorStateTimer.expires_after(syncSensorRate);
  m_syncSensorStateTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          syncSensorState();
        }
      });

  for(const std::shared_ptr<SimulatorConnection>& connection : m_connections)
  {
    if(connection->subscribedChannels.empty())
      continue;

    const size_t currentChannel = connection->subscribedChannels[connection->m_lastSyncedChannelIdx];
    const auto& channelMap = staticData.sensorChannelMap.at(currentChannel);

    size_t batchIdx = connection->m_nextSyncBatchIdx;
    bool sensorFound = false;

    uint16_t stateBuf = 0;
    std::map<size_t, size_t>::const_iterator it;

    while(true)
    {
      const size_t firstAddress = batchIdx * 16 + 1;
      const size_t lastAddress = (batchIdx + 1) * 16;

      it = channelMap.lower_bound(firstAddress);
      stateBuf = 0;

      while(it != channelMap.end())
      {
        if(it->first > lastAddress)
          break;

        const auto& sensor = staticData.sensors[it->second];

        // Axle counter sensor send diffs so there is no point in querying them
        if(sensor.type == Sensor::Type::AxleCounter)
        {
          it++;
          continue;
        }

        const auto& sensorState = m_networkStateData.sensors[it->second];
        if(sensorState.value)
          stateBuf |= 1 << (sensor.address - firstAddress);

        sensorFound = true;
        it++;
      }

      if(sensorFound || it == channelMap.end())
        break;

      // Go to next batch
      batchIdx = (it->first - 1) / 16;
    }

    if(sensorFound)
      connection->send(SimulatorProtocol::SensorBatchState(currentChannel, batchIdx, m_networkStateData.powerOn ? stateBuf : 0));

    if(it == channelMap.end())
    {
      // We reached channel end, go to start of next channel
      connection->m_nextSyncBatchIdx = 0;
      connection->m_lastSyncedChannelIdx++;

      if(connection->m_lastSyncedChannelIdx >= connection->subscribedChannels.size())
        connection->m_lastSyncedChannelIdx = 0; // Restart from first channel
    }
    else
    {
      connection->m_nextSyncBatchIdx = batchIdx + 1;
    }
  }

  mLastSpawnSyncIdx++;
  if(mLastSpawnSyncIdx >= m_networkStateData.spawns.size())
    mLastSpawnSyncIdx = 0;

  if(mLastSpawnSyncIdx < m_networkStateData.spawns.size())
  {
    auto it = staticData.spawns.begin();
    std::advance(it, mLastSpawnSyncIdx);

    const Spawn &spawn = it->second;
    SpawnState &spawnState = m_networkStateData.spawns.at(spawn.address);

    for(const auto& connection : m_connections)
    {
      if(connection->connectionId() != spawnState.ownerConnectionId)
        continue;

      uint8_t state = SimulatorProtocol::SpawnStateChange::Reset;
      if(spawnState.state == SpawnState::State::Ready)
        state = SimulatorProtocol::SpawnStateChange::Ready;
      else if(spawnState.state == SpawnState::State::WaitingReset)
        state = SimulatorProtocol::SpawnStateChange::WaitingReset;

      connection->send(SimulatorProtocol::SpawnStateChange(spawn.address, state));
      break;
    }
  }
}

void Simulator::syncReplicaState()
{
  assert(isNetworkThread());

  m_syncReplicaStateTimer.expires_after(syncReplicaRate);
  m_syncReplicaStateTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          syncReplicaState();
        }
      });

  if(staticData.sensorChannelMap.empty())
    return;

  Train *trainToSync = nullptr;
  {
    auto it = m_networkStateData.trains.find(mLastTrainSyncId);
    if(it == m_networkStateData.trains.end())
      it = m_networkStateData.trains.begin();

    if(it != m_networkStateData.trains.end())
    {
      trainToSync = &it->second;
      it++;

      if(it != m_networkStateData.trains.end())
        mLastTrainSyncId = it->first;
      else
        mLastTrainSyncId = invalidTrainId;
    }
  }

  if(trainToSync)
    sendTrainStateRefreshReplica(trainToSync);

  for(const std::shared_ptr<SimulatorConnection>& connection : m_repliacas)
  {
    size_t firstIdx = connection->m_nextSyncBatchIdx * 16;

    if(firstIdx >= staticData.sensors.size())
    {
      // Start from zero again
      connection->m_nextSyncBatchIdx = 0;
      firstIdx = 0;
    }

    const size_t idxCount = std::min(firstIdx + 16, staticData.sensors.size());

    uint16_t stateBuf = 0;
    for(size_t idx = firstIdx; idx < idxCount; idx++)
    {
      const auto& sensor = staticData.sensors[idx];

      // Axle counter sensor send diffs so there is no point in querying them
      if(sensor.type == Sensor::Type::AxleCounter)
      {
        continue;
      }

      const auto& sensorState = m_networkStateData.sensors[idx];
      if(sensorState.value)
        bitSet(stateBuf, idx - firstIdx);
    }

    connection->send(SimulatorProtocol::SensorBatchState(0, connection->m_nextSyncBatchIdx, stateBuf));
    connection->m_nextSyncBatchIdx++;

    if(mLastTurnoutSyncIdx >= staticData.turnouts.size())
      mLastTurnoutSyncIdx = 0;

    const size_t maxIdxToSync = std::min(mLastTurnoutSyncIdx + 11, staticData.turnouts.size());
    for(; mLastTurnoutSyncIdx < maxIdxToSync; mLastTurnoutSyncIdx++)
    {
      const auto &turnout = staticData.turnouts.at(mLastTurnoutSyncIdx);
      const auto &turnoutState = m_networkStateData.turnouts.at(mLastTurnoutSyncIdx);

      uint8_t val = 0;
      if(turnoutState.state == TurnoutState::State::Thrown)
        val = 2;
      else if(turnoutState.state == TurnoutState::State::Closed)
        val = 1;

      auto msg = SimulatorProtocol::AccessorySetState(turnout.channel, turnout.addresses[0], val);
      send(msg); // NOTE: also update clients
      sendReplicas(msg);
    }
  }
}

void Simulator::syncReplicaTrainFull()
{
  assert(isNetworkThread());

  // Less often we send full train state

  m_syncReplicaTrainFullStateTimer.expires_after(syncReplicaFullRate);
  m_syncReplicaTrainFullStateTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          syncReplicaTrainFull();
        }
      });

  // Refresh train speed factor periodically
  sendReplicas(SimulatorProtocol::TrainSpeedFactorChanged(m_networkStateData.trainSpeedFactor));

  if(staticData.sensorChannelMap.empty())
    return;

  Train *trainToSync = nullptr;
  {
    auto it = m_networkStateData.trains.find(mLastTrainSyncFullId);
    if(it == m_networkStateData.trains.end())
      it = m_networkStateData.trains.begin();

    if(it != m_networkStateData.trains.end())
    {
      trainToSync = &it->second;
      it++;

      if(it != m_networkStateData.trains.end())
        mLastTrainSyncFullId = it->first;
      else
        mLastTrainSyncFullId = invalidTrainId;
    }
  }

  if(trainToSync)
    sendTrainFullRefreshReplica(SimulatorProtocol::OpCode::TrainChangedVehicles,
                                trainToSync, true);
}

void Simulator::garbageCollectSegmentState()
{
  assert(isCoreThread());

  m_garbageCollectTimer.expires_after(garbageCollectRate);
  m_garbageCollectTimer.async_wait(
      [this](std::error_code ec)
      {
        if(!ec)
        {
          garbageCollectSegmentState();
        }
      });

  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  // We do it here to free some memory and speed up searches
  // do not do it at every vehicle change as it slows ticks too much
  auto it = m_stateData.segmentVehicles.begin();
  while(it != m_stateData.segmentVehicles.end())
  {
    if(it->second.vehicleIDs.empty())
      it = m_stateData.segmentVehicles.erase(it);
    else
      it++;
  }
}

void Simulator::updateTurnoutMovement()
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  for(size_t turnoutIdx = 0; turnoutIdx < m_stateData.turnouts.size(); turnoutIdx++)
  {
    TurnoutState &turnoutState = m_stateData.turnouts.at(turnoutIdx);
    if(turnoutState.currentPosition == turnoutState.requestedPosition)
      continue;

    const Turnout &turnout = staticData.turnouts.at(turnoutIdx);

    if(turnoutState.currentPosition < turnoutState.requestedPosition)
    {
      if((TurnoutState::ThrownPosition - turnout.step) < turnoutState.currentPosition)
        turnoutState.currentPosition = TurnoutState::ThrownPosition;
      else
        turnoutState.currentPosition += turnout.step;

      if(turnoutState.currentPosition > turnoutState.requestedPosition)
        turnoutState.currentPosition = turnoutState.requestedPosition;
    }
    else
    {
      if(turnout.step > turnoutState.currentPosition)
        turnoutState.currentPosition = TurnoutState::ClosedPosition;
      else
        turnoutState.currentPosition -= turnout.step;

      if(turnoutState.currentPosition < turnoutState.requestedPosition)
        turnoutState.currentPosition = turnoutState.requestedPosition;
    }

    turnoutState.currentPosition = std::clamp(turnoutState.currentPosition, TurnoutState::ClosedPosition, TurnoutState::ThrownPosition);

    if(turnoutState.currentPosition == turnoutState.requestedPosition)
    {
      // We reached target position
      setTurnoutIsMoving(false);
    }

    TurnoutState::State newState = TurnoutState::State::Unknown;
    if(turnoutState.currentPosition == TurnoutState::ClosedPosition)
      newState = TurnoutState::State::Closed;
    else if(turnoutState.currentPosition == TurnoutState::ThrownPosition)
      newState = TurnoutState::State::Thrown;

    doSetTurnoutState(newState, turnoutState, turnoutIdx);
  }

  if(mTurnoutsCurrentlyMoving > 0)
  {
    m_turnoutMoveTimer.expires_after(turnoutMovementRate);
    m_turnoutMoveTimer.async_wait(
        [this](std::error_code ec)
        {
          if(!ec)
          {
            updateTurnoutMovement();
          }
        });
  }
}

void Simulator::setTurnoutIsMoving(bool val)
{
  const bool wasActive = mTurnoutsCurrentlyMoving > 0;
  if(val)
  {
    mTurnoutsCurrentlyMoving++;
    if(!wasActive)
    {
      updateTurnoutMovement();
    }
  }
  else
  {
    assert(mTurnoutsCurrentlyMoving > 0);
    mTurnoutsCurrentlyMoving--;
  }
}

void Simulator::updateTrainPositions()
{
  if(staticData.trackSegments.empty()) [[unlikely]]
  {
    return;
  }

  constexpr float AccelerationUnitConversion = 1.0 / (1000.0 * 1000.0) * tickRate.count() * tickRate.count(); // m/s^2 to m/tick^2
  const float AccelerationConversionFactor = m_stateData.trainSpeedFactor * AccelerationUnitConversion;

  // Reduce a bit higher trainSpeedFactor so train does not depart instantly
  const float TickWaitFactor = 1.0 + (m_stateData.trainSpeedFactor - 1.0) / 3.0;

  const bool isReplica = getSimModeUnlocked() == SimMode::Replica;

  for(auto it = m_stateData.trains.begin(); it != m_stateData.trains.end(); )
  {
    Train& train = it->second;
    auto& trainState = train.state;

    if(!isReplica && train.state.isOnStationStop && train.state.mode != TrainState::Mode::Manual)
    {
      bool stoppedAndCannotContinue = train.state.speed == 0.0;
      if(stoppedAndCannotContinue)
        stoppedAndCannotContinue = (train.state.nextSignal.signal && train.state.nextSignal.signal->isStop());

      // Train cannot proceed forward, lets see if we can reverse
      if(stoppedAndCannotContinue &&
          train.state.prevSignal.signal && !train.state.prevSignal.signal->isStop())
      {
        // Prev signal detected and set to proceed, reverse train
        setTrainDirection(train, !train.state.reverse);
      }
    }

    bool wasStopped = false;
    float totalTravelled = 0.0f;

    // If train is stopped at signal check if signal goes to "Proceed"
    if(!isReplica && train.state.speed == 0.0f)
      checkNextSignal(train);

    // Update only if moving or first time to place vehicles on tracks
    if(train.state.isOnStationStop || train.state.speed != 0.0f ||
        (!train.vehicles.empty() && !m_stateData.getVehicleById(train.vehicles.front().vehicleId)->state.front.position.isFinite()))
    {
      if(!isReplica)
      {
        updateTrainNextSignal(train, true);
        if(train.state.isOnStationStop)
          updateTrainNextSignal(train, false);

        checkNextSignal(train);
      }

      auto updateHelper = [this](Simulator::Train::VehicleItem& item, float speed,
                                 bool reverse, bool isFirst, bool isLast,
                                 Train &train_, bool &trainRemoved, float &outRemaining) -> bool
      {
        if(reverse)
          speed = -speed;

        Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);
        if(item.reversed == reverse)
          return updateVehiclePosition(vehicle, true, speed, isFirst, false, train_, trainRemoved, outRemaining) &&
                 updateVehiclePosition(vehicle, false, speed, false, isLast, train_, trainRemoved, outRemaining);
        else
          return updateVehiclePosition(vehicle, false, speed, isFirst, false, train_, trainRemoved, outRemaining) &&
                 updateVehiclePosition(vehicle, true, speed, false, isLast, train_, trainRemoved, outRemaining);
      };

      const float speed = m_stateData.powerOn ? trainState.speed * m_stateData.trainSpeedFactor : 0.0f;

      bool trainRemoved = false;
      float outRemaining = 0.0f;

      do
      {
        float tickPosDelta = speed;
        if(outRemaining > 0.0f && speed != 0.0f)
        {
          // This is second round, travel just remaining distance to stop
          if(speed > 0.0f)
            tickPosDelta = outRemaining;
          else
            tickPosDelta = outRemaining;
        }

        bool isFirst = true;

        if(!train.vehicles.empty())
        {
          const size_t lastVehicleIdx = train.vehicles.size() - 1;
          for(size_t i = 0; i <= lastVehicleIdx; i++)
          {
            auto& vehicleItem = train.vehicles[trainState.reverse ? (lastVehicleIdx - i) : i];

            if(!updateHelper(vehicleItem, tickPosDelta, trainState.reverse, i == 0, i == lastVehicleIdx, train, trainRemoved, outRemaining))
            {
              if(trainRemoved)
                break;

              setTrainSpeed(train, 0.0f, true);
              if(isFirst)
                wasStopped = true;

              break;
            }
          }
        }

        if(!wasStopped)
          totalTravelled += tickPosDelta;
      }
      while(!trainRemoved && outRemaining > 0.0f && speed != 0.0f);

      if(trainRemoved)
      {
        const quint32 trainId = it->second.trainId;
        destroyTrain(it->second, true);
        it = m_stateData.trains.erase(it);
        onTrainAddedRemoved(false, trainId, invalidTrainId);

        if(getSimModeUnlocked() == SimMode::Master)
        {
          boost::asio::post(
              m_networkContext, [this, trainId]()
              {
                sendReplicas(SimulatorProtocol::TrainRemoved(trainId, false));
              });
        }
        continue;
      }
    }

    if(!wasStopped)
    {
      if(train.state.nextSignal.signal)
        train.state.nextSignal.distance -= totalTravelled;

      if(train.state.prevSignal.signal)
        train.state.prevSignal.distance += totalTravelled;

      if(train.state.waitToStart_ms > 0.0)
      {
        train.state.waitToStart_ms -= TickWaitFactor * tickRate.count();

        // Prevent going negative, also ignore wait if in Manual mode
        if(train.state.waitToStart_ms < 0.0 || train.state.mode == TrainState::Mode::Manual)
          train.state.waitToStart_ms = 0.0;
      }

      if(train.state.waitToStart_ms == 0.0)
      {
        // Accelerate
        if(train.state.targetSpeed > train.state.speed)
        {
          const float newSpeed = std::clamp(train.state.speed + TrainState::AccelerationRate_ms2 * AccelerationConversionFactor,
                                            0.0f, train.state.targetSpeed);

          // std::cout << "TRAIN ACCELERATE: " << newSpeed / SpeedKmHtoTick << " km/h    FROM: "
          //           << train.state.speed / SpeedKmHtoTick << " km/h" << std::endl;

          train.state.speed = newSpeed;
          train.state.speedOrDirectionChanged = true;
        }
        else if(train.state.targetSpeed < train.state.speed)
        {
          const float newSpeed = std::clamp(train.state.speed - TrainState::DecelerationRate_ms2 * AccelerationConversionFactor,
                                            train.state.targetSpeed, train.speedMax);
          // std::cout << "TRAIN BRAKE: " << newSpeed / SpeedKmHtoTick << " km/h    FROM: "
          //           << train.state.speed / SpeedKmHtoTick << " km/h" << std::endl;

          train.state.speed = newSpeed;
          train.state.speedOrDirectionChanged = true;
        }
      }
    }

    if(!isReplica && train.address != invalidAddress && trainState.speedOrDirectionChanged)
    {
      boost::asio::post(
          m_networkContext, [this, train=train]()
          {
            send(SimulatorProtocol::LocomotiveSpeedDirection(
                train.address,
                train.protocol,
                static_cast<uint8_t>(std::clamp<float>(std::round(std::numeric_limits<uint8_t>::max() * (train.state.speed / train.speedMax)),
                                                       std::numeric_limits<uint8_t>::min(),
                                                       std::numeric_limits<uint8_t>::max())),
                train.state.reverse ? Direction::Reverse : Direction::Forward,
                false));
          });

      trainState.speedOrDirectionChanged = false;

      if(getSimModeUnlocked() == SimMode::Master)
        sendTrainStateRefreshReplica(&train);
    }

    // Go to next train
    it++;
  }
}

inline bool isTurnout(const Simulator::TrackSegment& segment)
{
  switch (segment.type)
  {
  case Simulator::TrackSegment::Type::Straight:
  case Simulator::TrackSegment::Type::Curve:
    return false;
  case Simulator::TrackSegment::Type::Turnout:
  case Simulator::TrackSegment::Type::TurnoutCurved:
  case Simulator::TrackSegment::Type::Turnout3Way:
    return true;
  default:
    assert(false);
  }

  return false;
}

inline bool isTurnoutUnknownState(const Simulator::TrackSegment& segment,
                                  const Simulator::StateData& stateData)
{
  switch (segment.type)
  {
  case Simulator::TrackSegment::Type::Straight:
  case Simulator::TrackSegment::Type::Curve:
    return false;
  case Simulator::TrackSegment::Type::Turnout:
  case Simulator::TrackSegment::Type::TurnoutCurved:
  case Simulator::TrackSegment::Type::Turnout3Way:
  {
    return stateData.turnouts.at(segment.turnout.index).state
           == Simulator::TurnoutState::State::Unknown;
  }
  default:
    assert(false);
    break;
  }

  return false;
}



void Simulator::calculateFacePosition(Simulator::VehicleState::Face &face)
{
  if(face.segmentIndex == invalidIndex)
    return;

  const auto& segment = staticData.trackSegments[face.segmentIndex];
  size_t curveIndex = Simulator::invalidIndex;

  if(isStraight(segment))
  {
    face.position.x = segment.points[0].x + face.distance * std::cos(segment.rotation);
    face.position.y = segment.points[0].y + face.distance * std::sin(segment.rotation);
  }
  else if(isCurve(segment, curveIndex))
  {
    assert(curveIndex != Simulator::invalidIndex);
    const auto& curve = segment.curves[curveIndex];

    float angle = segment.rotation + (face.distance / curve.length) * curve.angle;
    if(curve.angle < 0)
    {
      angle += pi;
    }

    face.position.x = curve.center.x + curve.radius * std::sin(angle);
    face.position.y = curve.center.y - curve.radius * std::cos(angle);
  }
  else
  {
    assert(false);
  }
}

bool Simulator::updateVehiclePosition(Vehicle *vehicle, bool frontFace,
                                      const float speed, bool isFirst_, bool isLast_,
                                      Train &train_, bool &trainRemoved, float &outRemaining)
{
  using Object = Simulator::TrackSegment::Object;

  const bool isReplica = getSimModeUnlocked() == SimMode::Replica;

  VehicleState::Face& face = frontFace ? vehicle->state.front : vehicle->state.rear;

  outRemaining = 0.0f;

  auto objHelper = [this](const TrackSegment::Object& obj,
                          const float facePos, const float targetPos,
                          Train &train, bool dirFwd_, bool isFirst, bool isLast, bool &stop,
                          bool &trainRemoved_) -> bool
  {
    if(dirFwd_)
    {
      if(obj.position < facePos)
        return true;

      if(obj.position > targetPos)
        return false;
    }
    else
    {
      if(obj.position > facePos)
        return true;

      if(obj.position < targetPos)
        return false;
    }

    if(dirFwd_ && obj.allowedDirection == Object::AllowedDirections::Backwards)
      return true; // Only forward or both
    else if(!dirFwd_ && obj.allowedDirection == Object::AllowedDirections::Forward)
      return true; // Only backwards or both

    if(obj.type == Object::Type::PositionSensor)
    {
      SensorState& sensor = m_stateData.sensors[obj.sensorIndex];
      switch(staticData.sensors.at(obj.sensorIndex).type)
      {
      case Sensor::Type::PositionSensor:
      {
        // Trigger sensor and start delay count
        sensor.occupied = 1;
        sensor.curTime = sensor.maxTime;
        break;
      }
      case Sensor::Type::AxleCounter:
      {
        if(dirFwd_ == obj.dirForward)
          sensor.axleCount++;
        else
          sensor.axleCount--;
        sensor.curTime = sensor.axleCount > 0 ? sensor.maxTime : -sensor.maxTime;
        break;
      }
      default:
        break;
      }
    }
    else if (isFirst && obj.type == Object::Type::MainSignal && train.state.mode != TrainState::Mode::Manual)
    {
      auto it = m_stateData.mainSignals.find(obj.signalName);
      if(it == m_stateData.mainSignals.end())
        return true;

      // Reset station stop state for each passed main signal
      train.state.isOnStationStop = false;

      clearTrainSignalCache(m_stateData, train, false);

      MainSignal *signal = &it->second;
      if(signal->isStop())
      {
        // Stop immediately
        // TODO: if SPAD set to manual mode
        stop = true;
        return false;
      }

      // SemiAutomatic train should not depart if already stopped
      if(train.state.mode == TrainState::Mode::SemiAutomatic && train.state.speed == 0.0)
        return true;

      // Shift train speeds
      train.state.otherPreviousSignalSpeed = train.state.previousSignalSpeed;
      train.state.previousSignalSpeed = train.state.passedSignalSpeed;

      // Use previous signal next speed to calculate current signal speed
      train.state.passedSignalSpeed = signal->getCurrentSpeed(train.state.passedSignalNextSpeed);
      train.state.passedSignalNextSpeed = signal->maxNextSpeed;

      if(train.state.betweenSignalsCount < std::numeric_limits<uint8_t>::max())
      {
        // Train HEAD passed a signal
        train.state.betweenSignalsCount++;
      }

      // Apply new signal speed on next tick
      checkNextSignal(train);
      train.state.nextSignal.dirty = true;
    }
    else if (isLast && obj.type == Object::Type::MainSignal && train.state.mode != TrainState::Mode::Manual)
    {
      if(train.state.betweenSignalsCount > 0)
      {
        // Train TAIL passed a signal
        train.state.betweenSignalsCount--;
      }

      if(train.state.betweenSignalsCount == 0)
        train.state.previousSignalSpeed = MainSignal::SpecialSpeeds::NO_LIMIT;

      if(train.state.betweenSignalsCount <= 1)
        train.state.otherPreviousSignalSpeed = MainSignal::SpecialSpeeds::NO_LIMIT;
    }
    else if (isFirst && obj.type == Object::Type::ReverseDirection && train.state.mode != TrainState::Mode::Manual)
    {
      // Apply direction change on next tick
      setTrainDirection(train, !train.state.reverse);
      stop = true;
      return false;
    }
    else if (isFirst && obj.type == Object::Type::RemoveTrain)
    {
      trainRemoved_ = true;
      stop = true;
      return false;
    }
    else if(train.state.mode != TrainState::Mode::Manual && obj.type == Object::Type::StationStopPoint)
    {
      if(isFirst)
      {
        // When train head enters station stop, set flag
        train.state.isOnStationStop = true;
      }

      // At each train wagon passing recalculate previous signal
      train.state.prevSignal.dirty = true;
    }
    else if(isLast && train.state.mode == TrainState::Mode::Automatic && obj.type == Object::Type::SpeedLimitRelease)
    {
      // When entire train has passed speed release point, increase speed
      if(train.state.betweenSignalsCount == 0)
      {
        train.state.previousSignalSpeed = MainSignal::SpecialSpeeds::NO_LIMIT;

        if(train.state.passedSignalSpeed != MainSignal::SpecialSpeeds::OVERRIDE_BRAKE)
          train.state.passedSignalSpeed = MainSignal::SpecialSpeeds::NO_LIMIT; // Keep OVERRIDE_BRAKE
      }

      if(train.state.betweenSignalsCount <= 1)
        train.state.otherPreviousSignalSpeed = MainSignal::SpecialSpeeds::NO_LIMIT;

      // Apply new signal speed on next tick
      checkNextSignal(train);
    }

    return true;
  };

  float distance = face.distance + (face.segmentDirectionInverted ? -speed : speed);

  // Move to next segment when reaching the end:
  for(;;)
  {
    const size_t faceSegmentIndexBefore = face.segmentIndex;
    const auto& segment = staticData.trackSegments[face.segmentIndex];
    const auto segmentLength = getSegmentLength(segment, m_stateData);

    bool stop = false;
    const bool dirFwd = face.segmentDirectionInverted == (speed < 0);
    if(dirFwd && !isReplica)
    {
      for(const TrackSegment::Object& obj : segment.objects)
      {
        if(objHelper(obj, face.distance, distance,
                      train_, true, isFirst_, isLast_, stop, trainRemoved))
          continue;

        if(stop)
        {
          outRemaining = std::abs(face.distance - obj.position) - ADJ_MARGIN;
        }
        break;
      }
    }
    else if(!dirFwd && !isReplica)
    {
      for(const TrackSegment::Object& obj : segment.objects | std::views::reverse)
      {
        if(objHelper(obj, face.distance, distance,
                      train_, false, isFirst_, isLast_, stop, trainRemoved))
          continue;

        if(stop)
        {
          outRemaining = std::abs(face.distance - obj.position);
        }
        break;
      }
    }

    if(!isReplica && isFirst_ && vehicle->activeTrainId != invalidTrainId)
    {
      // We are train head, check other trains for collision

      auto scanSegment = [&](quint32 activeTrainId, size_t faceSegmentIndex,
                             bool goForward, float startDistance, float endDistance) -> bool
      {
        auto it = m_stateData.segmentVehicles.find(faceSegmentIndex);
        if(it == m_stateData.segmentVehicles.end())
          return false;

        const SegmentVehicles &segVec = it->second;
        bool otherTrainExists = false;

        for(quint32 otherId : segVec.vehicleIDs)
        {
          Vehicle *otherVehicle = m_stateData.getVehicleById(otherId);
          if(!otherVehicle || otherVehicle->activeTrainId == activeTrainId)
            continue; // Same train, skip

          otherTrainExists = true;

          auto faceHelper = [&](const VehicleState::Face &otherFace)
          {
            if(goForward && otherFace.distance > startDistance && (otherFace.distance - staticData.trainCouplingLength) < endDistance)
            {
              float remDist = std::max(startDistance, otherFace.distance - staticData.trainCouplingLength) - startDistance - ADJ_MARGIN;
              if(!stop || remDist < outRemaining)
                outRemaining = remDist;
              stop = true;
            }
            else if(!goForward && otherFace.distance < startDistance && (otherFace.distance + staticData.trainCouplingLength) > endDistance)
            {
              float remDist = startDistance - std::min(startDistance, otherFace.distance + staticData.trainCouplingLength) - ADJ_MARGIN;
              if(!stop || remDist < outRemaining)
                outRemaining = remDist;
              stop = true;
            }
          };

          if(otherVehicle->state.front.segmentIndex == faceSegmentIndex)
            faceHelper(otherVehicle->state.front);

          if(otherVehicle->state.rear.segmentIndex == faceSegmentIndex)
            faceHelper(otherVehicle->state.rear);
        }

        return otherTrainExists;
      };

      if(!train_.state.skipCollisionDetection)
      {
        if(!scanSegment(vehicle->activeTrainId, face.segmentIndex, dirFwd, face.distance, distance))
          train_.state.skipCollisionDetection = true; // We are the only train in segment, skip further checks
      }

      auto scanNextHelper = [&](bool goForward, size_t otherSegmentIdx, float searchDist, float startDistance, float endDistance)
      {
        while(searchDist > 0.0)
        {
          const auto nextSegmentIndex = getNextSegmentIndex(staticData.trackSegments[otherSegmentIdx], goForward, m_stateData);
          if(nextSegmentIndex == invalidIndex)
            break;

          const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];
          goForward = nextSegment.nextSegmentIndex[0] == otherSegmentIdx;

          const float segLength = getSegmentLength(nextSegment, m_stateData);
          scanSegment(vehicle->activeTrainId, nextSegmentIndex,
                      goForward, goForward ? -startDistance : (segLength + startDistance), goForward ? -endDistance : (segLength + endDistance));

          searchDist -= segLength;
          startDistance += segLength;
          endDistance += segLength;
          otherSegmentIdx = nextSegmentIndex;
        }
      };

      // When near end of segment, scan adjacent segment too!
      if(dirFwd && (distance + staticData.trainCouplingLength) > segmentLength)
      {
        scanNextHelper(dirFwd, faceSegmentIndexBefore,
                       (distance + staticData.trainCouplingLength) - segmentLength,
                       segmentLength - face.distance, segmentLength - distance);
      }
      else if(!dirFwd && (distance - staticData.trainCouplingLength) < 0)
      {
        scanNextHelper(dirFwd, faceSegmentIndexBefore,
                       -(distance - staticData.trainCouplingLength),
                       face.distance, distance);
      }
    }

    if(stop)
      return false;

    // If train is right on the end of segment (distance == segmentLenght) do not update
    // unlsess we are effectively moving. Because if speed is 0, segmentDirectionInverted is
    // not calculated correctly and because in case of 2 segment touching by their side 1,
    // it will infinitely jump vehicle face from one to another.
    if(distance > segmentLength || (distance == segmentLength && speed != 0))
    {
      if(isFirst_)
        train_.state.skipCollisionDetection = false; // Check new segment

      const auto nextSegmentIndex = getNextSegmentIndex(segment, true, m_stateData);
      if(nextSegmentIndex == invalidIndex)
      {
        outRemaining = segmentLength - face.distance - ADJ_MARGIN;
        if(outRemaining > ADJ_MARGIN && outRemaining <= std::abs(speed))
        {
          // Travel exactly to segment end
          distance = segmentLength - ADJ_MARGIN;
          break;
        }
        return false; // no next segment
      }

      // If no signal was found rescan on segment change
      if(!train_.state.nextSignal.signal)
        train_.state.nextSignal.dirty = true;

      const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];

      if(isTurnout(nextSegment))
      {
        // Check turnout
        if(nextSegment.nextSegmentIndex[0] != faceSegmentIndexBefore)
        {
          // We take turnout from straight/curve side, check its state
          if(!isReplica && getNextSegmentIndex(nextSegment, true, m_stateData) != faceSegmentIndexBefore)
          {
            // Turnout is not in good state, train will hit switch blades and derail
            // We simulate by stopping train and setting turnout to unknown position
            requestTurnoutState(nextSegmentIndex, TurnoutState::State::Unknown);
          }
        }

        if(m_stateData.turnouts.at(nextSegment.turnout.index).state
            == TurnoutState::State::Unknown)
        {
          // Turnout is in unknown position, stop train
          outRemaining = segmentLength - face.distance - ADJ_MARGIN;
          if(outRemaining > ADJ_MARGIN && outRemaining <= std::abs(speed))
          {
            // Travel exactly to segment end
            distance = segmentLength - ADJ_MARGIN;
            break;
          }
          return false;
        }
      }

      if(!isReplica && segment.sensor.index != invalidIndex)
      {
        auto& sensor = m_stateData.sensors[segment.sensor.index];
        assert(sensor.occupied != 0);
        sensor.occupied--;
      }

      maybeAddVehicleSegment(vehicle, nextSegmentIndex);
      face.segmentIndex = nextSegmentIndex;
      maybeRemoveVehicleSegment(vehicle, faceSegmentIndexBefore);

      if(nextSegment.nextSegmentIndex[0] == faceSegmentIndexBefore)
      {
        distance -= segmentLength;
        face.distance = 0.0f;
        face.segmentDirectionInverted = (speed < 0);
      }
      else
      {
        face.distance = getSegmentLength(nextSegment, m_stateData);
        distance = (face.distance + segmentLength) - distance;
        face.segmentDirectionInverted = (speed > 0);
      }

      if(!isReplica && nextSegment.sensor.index != invalidIndex)
      {
        m_stateData.sensors[nextSegment.sensor.index].occupied++;
      }
    }
    else if(distance < 0)
    {
      if(isFirst_)
        train_.state.skipCollisionDetection = false; // Check new segment

      const auto nextSegmentIndex = getNextSegmentIndex(segment, false, m_stateData);
      if(nextSegmentIndex == invalidIndex)
      {
        outRemaining = face.distance;
        if(outRemaining > ADJ_MARGIN && outRemaining <= std::abs(speed))
        {
          // Travel exactly to segment start
          distance = 0;
          break;
        }
        return false; // no next segment
      }

      // If no signal was found rescan on segment change
      if(!train_.state.nextSignal.signal)
        train_.state.nextSignal.dirty = true;

      const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];

      if(isTurnout(nextSegment))
      {
        // Check turnout
        if(nextSegment.nextSegmentIndex[0] != faceSegmentIndexBefore)
        {
          // We take turnout from straight/curve side, check its state
          if(!isReplica && getNextSegmentIndex(nextSegment, true, m_stateData) != faceSegmentIndexBefore)
          {
            // Turnout is not in good state, train will hit switch blades and derail
            // We simulate by stopping train and setting turnout to unknown position
            requestTurnoutState(nextSegmentIndex, TurnoutState::State::Unknown);
          }
        }

        if(m_stateData.turnouts.at(nextSegment.turnout.index).state
            == TurnoutState::State::Unknown)
        {
          // Turnout is in unknown position, stop train
          outRemaining = face.distance;
          if(outRemaining > ADJ_MARGIN && outRemaining <= std::abs(speed))
          {
            // Travel exactly to segment start
            distance = 0;
            break;
          }
          return false;
        }
      }

      if(!isReplica && segment.sensor.index != invalidIndex)
      {
        auto& sensor = m_stateData.sensors[segment.sensor.index];
        assert(sensor.occupied != 0);
        sensor.occupied--;
      }

      maybeAddVehicleSegment(vehicle, nextSegmentIndex);
      face.segmentIndex = nextSegmentIndex;
      maybeRemoveVehicleSegment(vehicle, faceSegmentIndexBefore);

      if(nextSegment.nextSegmentIndex[0] == faceSegmentIndexBefore)
      {
        distance = -distance;
        face.distance = 0.0f;
        face.segmentDirectionInverted = (speed < 0);
      }
      else
      {
        face.distance = getSegmentLength(nextSegment, m_stateData);
        distance += face.distance;
        face.segmentDirectionInverted = (speed > 0);
      }

      if(!isReplica && nextSegment.sensor.index != invalidIndex)
      {
        m_stateData.sensors[nextSegment.sensor.index].occupied++;
      }
    }
    else
    {
      break; // we're within section boundries
    }
  }

  // Calculate position:
  face.distance = distance;
  calculateFacePosition(face);

  return true;
}

void Simulator::updateReplicaTracking(Vehicle *vehicle, const VehicleState &oldState)
{
  maybeRemoveVehicleSegment(vehicle, oldState.front.segmentIndex);
  if(oldState.rear.segmentIndex != oldState.front.segmentIndex)
    maybeRemoveVehicleSegment(vehicle, oldState.rear.segmentIndex);

  maybeAddVehicleSegment(vehicle, vehicle->state.front.segmentIndex);
  if(vehicle->state.rear.segmentIndex != vehicle->state.front.segmentIndex)
    maybeAddVehicleSegment(vehicle, vehicle->state.rear.segmentIndex);

  calculateFacePosition(vehicle->state.front);
  calculateFacePosition(vehicle->state.rear);
}

void Simulator::maybeAddVehicleSegment(Vehicle *vehicle, size_t segmentIdx)
{
  if(segmentIdx == invalidIndex)
    return;

  auto it = m_stateData.segmentVehicles.find(segmentIdx);
  if(it == m_stateData.segmentVehicles.end())
    it = m_stateData.segmentVehicles.insert({segmentIdx, {}}).first;

  SegmentVehicles &seg = it->second;
  if(seg.vehicleIDs.contains(vehicle->vehicleId))
    return;

  for(quint32 otherId : seg.vehicleIDs)
  {
    Vehicle *other = m_stateData.getVehicleById(otherId);
    if(!other || other->activeTrainId == invalidTrainId || other->activeTrainId == vehicle->activeTrainId)
      continue;

    // Notify other trains we are in same segment so they need to check again
    Train &otherTrain = m_stateData.trains.at(other->activeTrainId);
    otherTrain.state.skipCollisionDetection = false;
  }

  seg.vehicleIDs.insert(vehicle->vehicleId);
}

void Simulator::maybeRemoveVehicleSegment(Vehicle *vehicle, size_t segmentIdx)
{
  if(segmentIdx == invalidIndex || vehicle->state.front.segmentIndex == segmentIdx || vehicle->state.rear.segmentIndex == segmentIdx)
    return; // Half vehicle still in segment, do not remove

  auto it = m_stateData.segmentVehicles.find(segmentIdx);
  if(it == m_stateData.segmentVehicles.end())
    return;

  SegmentVehicles &seg = it->second;
  auto it2 = seg.vehicleIDs.find(vehicle->vehicleId);
  if(it2 != seg.vehicleIDs.end())
    seg.vehicleIDs.erase(it2);
}

void Simulator::updateSensors()
{
  const size_t count = staticData.sensors.size();
  for(size_t i = 0; i < count; ++i)
  {
    const auto& sensor = staticData.sensors[i];
    auto& sensorState = m_stateData.sensors[i];
    int32_t axleCount = 0;
    bool sensorValue = sensorState.value;

    switch(sensor.type)
    {
    case Sensor::Type::TrackCircuit:
    {
      sensorValue = m_stateData.powerOn && (sensorState.occupied != 0);
      break;
    }
    case Sensor::Type::PositionSensor:
    {
      sensorValue = m_stateData.powerOn && (sensorState.occupied != 0);
      if(sensorValue && sensorState.curTime > 0)
      {
        // Decrease delay count
        sensorState.curTime--;

        if(sensorState.curTime == 0)
        {
          // When delay reaches zero go back to idle
          sensorState.occupied = 0;
          sensorValue = false;
        }
      }
      break;
    }
    case Sensor::Type::AxleCounter:
    {
      // Decrease delay count for visual effect
      if(sensorState.curTime > 0)
        sensorState.curTime--;
      else if(sensorState.curTime < 0)
        sensorState.curTime++;

      // Reset counter and send diff
      axleCount = sensorState.axleCount;
      sensorState.axleCount = 0;
      break;
    }
    default:
      assert(false);
    }

    if(sensorState.value != sensorValue || axleCount != 0)
    {
      if(getSimModeUnlocked() != SimMode::Replica)
      {
        sensorState.value = sensorValue;

        // TODO: maybe update networkStateData to avoid sending old state in syncSensorState() until next network state sync
        boost::asio::post(
            m_networkContext, [this, i, sensor=sensor, sensorState=sensorState, axleCount]()
            {
              // Cherry pick new state
              m_networkStateData.sensors[i] = sensorState;

              for(const auto& connection : m_connections)
              {
                // Only send to subscribed clients or clients with no subscriptions
                if(!connection->subscribedChannels.empty() &&
                    !vec_contains(connection->subscribedChannels, sensor.channel))
                  continue;
                connection->send(SimulatorProtocol::SensorChanged(sensor.channel, sensor.address, axleCount, sensorState.value));
              }

              if(getSimModeNetwork() == SimMode::Master)
              {
                for(const auto& connection : m_repliacas)
                {
                  connection->send(SimulatorProtocol::SensorChanged(sensor.channel, sensor.address, axleCount, sensorState.value));
                }
              }
            });
      }
    }
  }
}

void Simulator::updateSpawns()
{
  auto it = staticData.spawns.begin();
  auto it2 = m_stateData.spawns.begin();

  for(; it != staticData.spawns.end(); it++, it2++)
  {
    const Spawn &spawn = it->second;
    SpawnState &spawnState = it2->second;

    if(spawnState.waitingResetCount > 0 && spawnState.waitingResetCount != SpawnState::MAX_WAITING_RESET)
    {
      spawnState.waitingResetCount--;
      if(spawnState.waitingResetCount == 0 && spawnState.ownerConnectionId != invalidIndex)
      {
        spawnState.state = SpawnState::State::Ready;

        boost::asio::post(
            m_networkContext, [this, spawnAddress=spawn.address, spawnState=spawnState]
            {
              // Cherry pick new state
              m_networkStateData.spawns.at(spawnAddress) = spawnState;

              sendTo(spawnState.ownerConnectionId,
                     SimulatorProtocol::SpawnStateChange(spawnAddress, SimulatorProtocol::SpawnStateChange::Ready));
            });
      }
    }
  }
}

void Simulator::doSpawnTrain(uint16_t spawnAddress)
{
  auto it = staticData.spawns.find(spawnAddress);
  if(it == staticData.spawns.end())
    return;

  const Spawn &spawn = it->second;
  SpawnState &spawnState = m_stateData.spawns.at(spawnAddress);

  if(spawnState.state != SpawnState::State::Ready)
    return;

  // Spawn a train
  const std::string trainName = std::string("spawn") + std::to_string(spawn.address) + std::string("_");

  std::vector<Simulator::Train::VehicleItemFull> vehicles;
  size_t trainTypeIdx = trainTypeInterface->getRandomTrainType(spawn.allowList, spawn.blackList);

  if(trainTypeIdx != invalidIndex)
  {
    // Create composition
    bool reversible = false;
    size_t locoReversible = 0;
    vehicles = trainTypeInterface->createTrainOfType(trainTypeIdx,
                                                     reversible, locoReversible);
    if(!vehicles.empty())
    {
      std::uniform_int_distribution<size_t> invertLoco(0, 3);
      if(locoReversible != 0)
      {
        size_t shouldInvert = invertLoco(rng);
        if(locoReversible == 1 && shouldInvert <= 1)
          shouldInvert = 0;

        if(shouldInvert >= 2)
          vehicles.front().reversed = !vehicles.front().reversed;
      }

      std::uniform_int_distribution<size_t> invertTrain(0, 10);
      bool shouldInvert = reversible && invertTrain(rng) > 5;
      if(!spawn.forward)
        shouldInvert = !shouldInvert;

      if(shouldInvert)
      {
        std::reverse(vehicles.begin(), vehicles.end());
        for(auto &vehicle : vehicles)
          vehicle.reversed = !vehicle.reversed;
      }
    }
  }

  if(vehicles.empty())
  {
    // Create basic train
    std::uniform_int_distribution<size_t> dist(1, spawn.maxWagons);
    size_t numWagons = dist(rng);

    const auto& segment = staticData.trackSegments[spawn.segmentIndex];
    const float segmentLength = getSegmentLength(segment, m_stateData);
    const float avgLength = spawn.wagonLength + staticData.trainCouplingLength;
    const size_t maxWagons = size_t(std::floor(segmentLength / avgLength));

    numWagons = std::min(numWagons, maxWagons);

    for(size_t i = 0; i < numWagons; i++)
    {
      Simulator::Train::VehicleItemFull item;
      item.vehicle.length = spawn.wagonLength;
      item.vehicle.color = Color::Aqua;
      item.reversed = false;
      vehicles.push_back(item);
    }
  }

  TrainPlacement placement = TrainPlacement::PlaceEnd;
  if(!spawn.forward)
    placement = TrainPlacement::PlaceStart;

  quint32 outTrainId = invalidTrainId;
  if(addTrain(trainName, DecoderProtocol::DCCLong, 3,
               vehicles, spawn.segmentIndex, outTrainId, spawn.posInSegment, placement))
  {
    // DONE!
    Train &train = m_stateData.trains.at(outTrainId);
    setTrainDirection(train, !spawn.forward);
    setTrainMode(train, TrainState::Mode::Automatic);

    // Give some default speed
    train.state.speed = std::clamp(spawn.defaultSpeedKmH * SpeedKmHtoTick, 0.0f, train.speedMax);
    train.state.targetSpeed = std::clamp(train.speedMax * 0.8f, train.state.speed, train.speedMax);

    // Wait reset
    spawnState.state = SpawnState::State::WaitingReset;
    spawnState.waitingResetCount = SpawnState::MAX_WAITING_RESET;
  }
}

bool Simulator::isStraight(const TrackSegment& segment)
{
  using Type = TrackSegment::Type;
  using State = TurnoutState::State;

  switch(segment.type)
  {
  case Type::Straight:
    return true;

  case Type::Curve:
  case Type::TurnoutCurved:
    return false;

  case Type::Turnout:
  case Type::Turnout3Way:
    return m_stateData.turnouts[segment.turnout.index].state == State::Closed;

  case Type::SingleSlipTurnout:
  case Type::DoubleSlipTurnout:
    const auto state = m_stateData.turnouts[segment.turnout.index].state;
    switch (state)
    {
    case State::Closed:
    case State::ClosedLeft:
    case State::ClosedRight:
      return true;
    default:
      return false;
    }
  }
  assert(false);
  return false;
}

bool Simulator::isCurve(const TrackSegment& segment, size_t& curveIndex)
{
  using Type = TrackSegment::Type;
  using State = TurnoutState::State;

  switch(segment.type)
  {
  case Type::Straight:
    return false;

  case Type::Curve:
    curveIndex = 0;
    return true;

  case Type::Turnout:
    if(m_stateData.turnouts[segment.turnout.index].state == State::Thrown)
    {
      curveIndex = 0;
      return true;
    }
    return false;

  case Type::TurnoutCurved:
    curveIndex = m_stateData.turnouts[segment.turnout.index].state == State::Closed ? 0 : 1;
    return true;

  case Type::Turnout3Way:
    switch(m_stateData.turnouts[segment.turnout.index].state)
    {
    case State::Closed:
      return false;

    case State::ThrownLeft:
      curveIndex = 0;
      return true;

    case State::ThrownRight:
      curveIndex = 1;
      return true;

    default:
      break;
    }
    break;

  case Type::SingleSlipTurnout:
  case Type::DoubleSlipTurnout:
    const auto state = m_stateData.turnouts[segment.turnout.index].state;
    switch (state)
    {
    case State::Thrown:
    case State::ThrownLeft:
    case State::ThrownRight:
      curveIndex = 1; // TODO
      return true;
    default:
      return false;
    }
    break;
  }
  assert(false);
  return false;
}

void Simulator::loadTrackObjects(const nlohmann::json &track, StaticData &data, StateData &stateData,
                                 TrackSegment &segment, TrainTypeInterface *iface)
{
  // Objects:
  if(auto trackObjects = track.find("objects"); trackObjects != track.end() && trackObjects->is_array())
  {
    segment.objects.reserve(trackObjects->size());

    float totLen = 0;
    if(segment.type == TrackSegment::Type::Straight)
      totLen = segment.straight.length;
    else if(segment.type == TrackSegment::Type::Curve)
      totLen = segment.curves[0].length;

    if(totLen > 0)
    {
      for(const auto& item : *trackObjects)
      {
        if(!item.is_object())
        {
          continue;
        }

        TrackSegment::Object trackObj;
        trackObj.position = item.value("pos", std::numeric_limits<float>::quiet_NaN());
        if(!std::isfinite(trackObj.position))
          continue;

        if(trackObj.position < 0)
          trackObj.position = totLen + trackObj.position;

        if(trackObj.position < 0 || trackObj.position >= totLen)
          continue;

        if(segment.type == TrackSegment::Type::Straight)
        {
          trackObj.rotation = segment.rotation;
          trackObj.pos.x = segment.points[0].x + trackObj.position * std::cos(segment.rotation);
          trackObj.pos.y = segment.points[0].y + trackObj.position * std::sin(segment.rotation);
        }
        else if(segment.type == Simulator::TrackSegment::Type::Curve)
        {
          const auto& curve = segment.curves[0];

          float angle = segment.rotation + (trackObj.position / curve.length) * curve.angle;

          // Rotation must not be adjusted by 180 degrees
          trackObj.rotation = angle;

          if(curve.angle < 0)
          {
            angle += pi;
          }

          trackObj.pos.x = curve.center.x + curve.radius * std::sin(angle);
          trackObj.pos.y = curve.center.y - curve.radius * std::cos(angle);
        }

        while(trackObj.rotation > 2 * pi)
          trackObj.rotation -= 2 * pi;

        while(trackObj.rotation < 0)
          trackObj.rotation += 2 * pi;

        trackObj.dirForward = item.value("dir", true);

        const std::string_view type = item.value<std::string_view>("type", {});
        if(type == "sensor" || type == "axle_counter")
        {
          trackObj.type = TrackSegment::Object::Type::PositionSensor;
          trackObj.allowedDirection = TrackSegment::Object::AllowedDirections::Both;
          trackObj.lateralDiff = 2.0;

          if(const uint16_t sensorAddress = item.value("sensor_address", invalidAddress); sensorAddress != invalidAddress)
          {
            const uint16_t sensorChannel = item.value("sensor_channel", defaultChannel);

            auto &channelMap = data.sensorChannelMap[sensorChannel];
            auto it = channelMap.find(sensorAddress);
            if(it == channelMap.end())
            {
              // new sensor
              trackObj.sensorIndex = data.sensors.size();
              data.sensors.emplace_back(Sensor{sensorChannel, sensorAddress, Sensor::Type::PositionSensor});
              channelMap.insert({sensorAddress, trackObj.sensorIndex});
              if(type == "axle_counter")
                data.sensors.at(trackObj.sensorIndex).type = Sensor::Type::AxleCounter;
              stateData.sensors.emplace_back(SensorState{{0}, 0, 0, SensorState::Encoding::CodeAbsent, false});
            }
            else
              trackObj.sensorIndex = it->second;

            switch(data.sensors.at(trackObj.sensorIndex).type)
            {
            case Sensor::Type::PositionSensor:
            {
              // Make it stay triggered a bit
              stateData.sensors[trackObj.sensorIndex].maxTime = 90; // 3 seconds
              stateData.sensors[trackObj.sensorIndex].occupied = 0;
              break;
            }
            case Sensor::Type::AxleCounter:
            {
              // This is just for visual effect
              stateData.sensors[trackObj.sensorIndex].maxTime = 30; // 1 second
              stateData.sensors[trackObj.sensorIndex].axleCount = 0;
              break;
            }
            default:
              assert(false);
            }

          }

          if(trackObj.sensorIndex == invalidIndex) // invalid sensor
            continue;
        }
        else if(type == "main_signal")
        {
          trackObj.type = TrackSegment::Object::Type::MainSignal;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = -1.7; // Default on left side

          const std::string_view signalName = item.value<std::string_view>("name", {});
          const uint16_t signalAddress = item.value("address", invalidAddress);
          const uint16_t signalChannel = item.value("channel", defaultChannel);
          if(signalName.empty() || signalAddress == invalidAddress)
            continue;

          MainSignal *mainSignal = nullptr;
          auto signIt = stateData.mainSignals.find(signalName);
          if(signIt == stateData.mainSignals.end())
          {
            MainSignal mainSignal_;
            mainSignal_.name = signalName;
            mainSignal_.address = signalAddress;
            mainSignal_.channel = signalChannel;
            mainSignal_.lights.push_back({});
            auto res = stateData.mainSignals.insert({mainSignal_.name, mainSignal_});
            mainSignal = &res.first->second;
          }
          else
            mainSignal = &signIt->second;

          trackObj.signalName = mainSignal->name;

          size_t nLights = item.value("n_lights", std::max(size_t(1), mainSignal->lights.size()));
          nLights = std::clamp(nLights, size_t(1), size_t(3));
          mainSignal->lights.resize(nLights);

          mainSignal->square = item.value("square", false);
          if(mainSignal->square)
            mainSignal->hasSquareArrowLight = item.value("arrow_light", false);

          const size_t fixedLimit = item.value("fixed_limit", size_t(0));
          switch (fixedLimit)
          {
          case 30:
            mainSignal->fixedLimit = MainSignal::FixedLimit::Limit30;
            break;
          case 60:
            mainSignal->fixedLimit = MainSignal::FixedLimit::Limit60;
            break;
          default:
            mainSignal->fixedLimit = MainSignal::FixedLimit::NoLimit;
            break;
          }

          mainSignal->hasAdvanceSignal = item.value("advance_light", false);
          mainSignal->hasRappel = item.value("rappel", false);
          mainSignal->hasDirectionIndicator = item.value("direction_ind", false);
          mainSignal->isPureDistantSignal = item.value("pure_distant", false);
          mainSignal->isRipSignal = item.value("rip_signal", false);
          mainSignal->zoomLateralDiff = item.value("zoom_lat", true);

          if(mainSignal->hasRappel)
            mainSignal->fixedLimit = MainSignal::FixedLimit::NoLimit;

          if(mainSignal->hasAdvanceSignal || mainSignal->lights.size() != 1)
            mainSignal->isRipSignal = false;

          if(mainSignal->hasRappel || mainSignal->hasDirectionIndicator || mainSignal->isRipSignal ||
              mainSignal->hasAdvanceSignal || mainSignal->fixedLimit != MainSignal::FixedLimit::NoLimit ||
              mainSignal->lights.size() > 2)
            mainSignal->isPureDistantSignal = false;
        }
        else if(type == "aux_signal")
        {
          trackObj.type = TrackSegment::Object::Type::AuxSignal;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = -1.7; // Default on left side

          const std::string_view signalName = item.value<std::string_view>("name", {});
          const uint16_t signalAddress = item.value("address", invalidAddress);
          const uint16_t signalChannel = item.value("channel", defaultChannel);
          if(signalName.empty() || signalAddress == invalidAddress)
            continue;

          AuxSignal::SubType subType = AuxSignal::SubType::LightDwarfSignal;
          const std::string_view subTypeName = item.value<std::string_view>("sub_type", {});
          if(subTypeName == "light_dwarf_signal")
            subType = AuxSignal::SubType::LightDwarfSignal;
          else if(subTypeName == "rot_dwarf_signal")
            subType = Simulator::AuxSignal::SubType::RotatingDwarfSignal;
          else if(subTypeName == "high_departure_ind")
            subType = Simulator::AuxSignal::SubType::HighDepartureIndicator;
          else
            continue;

          AuxSignal *auxSignal = nullptr;
          auto signIt = stateData.auxSignals.find(signalName);
          if(signIt == stateData.auxSignals.end())
          {
            AuxSignal auxSignal_;
            auxSignal_.name = signalName;
            auxSignal_.address = signalAddress;
            auxSignal_.channel = signalChannel;
            auto res = stateData.auxSignals.insert({auxSignal_.name, auxSignal_});
            auxSignal = &res.first->second;
          }
          else
            auxSignal = &signIt->second;

          trackObj.signalName = auxSignal->name;

          auxSignal->subType = subType;
          auxSignal->zoomLateralDiff = item.value("zoom_lat", true);
        }
        else if(type == "reverse_dir")
        {
          trackObj.type = TrackSegment::Object::Type::ReverseDirection;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = 0; // Default on center
        }
        else if(type == "remove_train")
        {
          trackObj.type = TrackSegment::Object::Type::RemoveTrain;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = 0; // Default on center
        }
        else if(type == "spawn_train")
        {
          trackObj.type = TrackSegment::Object::Type::SpawnTrain;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = 0; // Default on center

          const size_t spawnAddress = item.value("address", invalidAddress);
          if(spawnAddress == invalidAddress || stateData.spawns.contains(spawnAddress))
            continue;

          Spawn spawn;
          spawn.address = spawnAddress;
          spawn.segmentIndex = data.trackSegments.size(); // TODO: HACK still has to be inserted
          spawn.maxWagons = item.value("max_wagons", spawn.maxWagons);
          spawn.wagonLength = item.value("wagons_length", spawn.wagonLength);
          spawn.defaultSpeedKmH = item.value("speed", spawn.defaultSpeedKmH);

          if(auto allowList = item.find("allow_list"); allowList != item.end() && allowList->is_array())
            spawn.allowList = iface->convertTypeList(*allowList);

          if(auto blackList = item.find("black_list"); blackList != item.end() && blackList->is_array())
            spawn.blackList = iface->convertTypeList(*blackList);

          stateData.spawns.insert({spawn.address, SpawnState{}});
          data.spawns.insert({spawn.address, std::move(spawn)});

          trackObj.sensorIndex = spawn.address;
        }
        else if(type == "station_stop_point")
        {
          trackObj.type = TrackSegment::Object::Type::StationStopPoint;
          trackObj.allowedDirection = TrackSegment::Object::AllowedDirections::Both;
          trackObj.lateralDiff = 0; // Default on center
        }
        else if(type == "speed_limit_release")
        {
          trackObj.type = TrackSegment::Object::Type::SpeedLimitRelease;
          trackObj.allowedDirection = trackObj.dirForward ?
                                          TrackSegment::Object::AllowedDirections::Forward :
                                          TrackSegment::Object::AllowedDirections::Backwards;
          trackObj.lateralDiff = 0; // Default on center
        }
        else
        {
          // Unknown object type
          continue;
        }

        const std::string_view allowedDir = item.value<std::string_view>("allowed_dir", {});
        if(allowedDir == "both")
          trackObj.allowedDirection = TrackSegment::Object::AllowedDirections::Both;
        if(allowedDir == "forward")
          trackObj.allowedDirection = TrackSegment::Object::AllowedDirections::Forward;
        if(allowedDir == "backwards")
          trackObj.allowedDirection = TrackSegment::Object::AllowedDirections::Backwards;

        trackObj.lateralDiff = item.value("lat_delta", trackObj.lateralDiff);

        if(trackObj.type == TrackSegment::Object::Type::SpawnTrain)
        {
          // Post init
          Spawn &spawn = data.spawns.at(trackObj.sensorIndex);
          spawn.forward = trackObj.allowedDirection == TrackSegment::Object::AllowedDirections::Forward;
          spawn.posInSegment = trackObj.position;
        }

        segment.objects.push_back(trackObj);
      }

      std::stable_sort(segment.objects.begin(),
                       segment.objects.end(),
                       [](const TrackSegment::Object& a, const TrackSegment::Object& b) -> bool
                       {
                         return a.position < b.position;
                       });
    }
  }
}

void Simulator::loadTrackplan(const nlohmann::json& world, StaticData &data, StateData& stateData, TrainTypeInterface *iface)
{
  if(auto trackPlan = world.find("trackplan"); trackPlan != world.end() && trackPlan->is_array())
  {
    data.trackSegments.reserve(trackPlan->size());

    size_t fromPointIndex = invalidIndex;
    size_t fromSegmentIndex = invalidIndex;
    Point curPoint{0.0f, 0.0f};
    float curRotation = 0;

    for(const auto& obj : *trackPlan)
    {
      if(!obj.is_object())
      {
        continue;
      }

      size_t startPointIndex = 0;
      size_t nextPointIndex = 1;
      TrackSegment segment;

      if(auto id = obj.value<std::string_view>("id", {}); !id.empty())
      {
        segment.m_id = id;
        data.trackSegmentId.emplace(std::move(id), data.trackSegments.size());
      }

      const auto type = obj.value<std::string_view>("type", {});
      if(type == "straight")
      {
        segment.type = TrackSegment::Type::Straight;
      }
      else if(type == "curve")
      {
        segment.type = TrackSegment::Type::Curve;
      }
      else if(type == "turnout")
      {
        segment.type = TrackSegment::Type::Turnout;
      }
      else if(type == "turnout_curved")
      {
        segment.type = TrackSegment::Type::TurnoutCurved;
      }
      else if(type == "turnout_3way")
      {
        segment.type = TrackSegment::Type::Turnout3Way;
      }
      else if(type == "turnout_single_slip")
      {
        segment.type = TrackSegment::Type::SingleSlipTurnout;
      }
      else if(type == "turnout_double_slip")
      {
        segment.type = TrackSegment::Type::DoubleSlipTurnout;
      }
      else
      {
        throw std::runtime_error("unknown track element type");
      }

      if(segment.type == TrackSegment::Type::Straight || segment.type == TrackSegment::Type::Turnout || segment.type == TrackSegment::Type::Turnout3Way)
      {
        segment.straight.length = obj.value("length", segment.straight.length);
      }
      if(segment.type == TrackSegment::Type::Curve ||
          segment.type == TrackSegment::Type::Turnout ||
          segment.type == TrackSegment::Type::TurnoutCurved ||
          segment.type == TrackSegment::Type::SingleSlipTurnout ||
          segment.type == TrackSegment::Type::DoubleSlipTurnout)
      {
        segment.curves[0].radius = obj.value("radius", 100.0f);
        segment.curves[0].angle = deg2rad(obj.value("angle", 45.0f));
      }
      if(segment.type == TrackSegment::Type::TurnoutCurved)
      {
        segment.curves[1].radius = obj.value("radius_2", 100.0f);
        segment.curves[1].angle = deg2rad(obj.value("angle_2", 45.0f));
      }
      if(segment.type == TrackSegment::Type::Turnout3Way)
      {
        const float angleMin = 0.0f;
        const float angleMax = 90.0f;
        const float radius = std::max(1.0f, obj.value("radius", 100.0f));
        const float angle = std::clamp(obj.value("angle", 45.0f), angleMin, angleMax);

        segment.curves[0].radius = std::max(1.0f,obj.value("radius_left", radius));
        segment.curves[0].angle = deg2rad(-std::clamp(obj.value("angle_left", angle), angleMin, angleMax));

        segment.curves[1].radius = std::max(1.0f,obj.value("radius_right", radius));
        segment.curves[1].angle = deg2rad(std::clamp(obj.value("angle_right", angle), angleMin, angleMax));
      }
      if(segment.type == TrackSegment::Type::Turnout ||
          segment.type == TrackSegment::Type::TurnoutCurved ||
          segment.type == TrackSegment::Type::Turnout3Way ||
          segment.type == TrackSegment::Type::SingleSlipTurnout ||
          segment.type == TrackSegment::Type::DoubleSlipTurnout)
      {
        Turnout turnout;
        turnout.channel = obj.value("channel", defaultChannel);

        if(segment.type == TrackSegment::Type::Turnout3Way ||
            segment.type == TrackSegment::Type::SingleSlipTurnout ||
            segment.type == TrackSegment::Type::DoubleSlipTurnout)
        {
          const auto address = obj.value("address", invalidAddress);
          turnout.addresses[0] = obj.value("address_left", address);
          turnout.addresses[1] = obj.value("address_right", (address != invalidAddress) ? static_cast<uint16_t>(address + 1) : invalidAddress);
        }
        else
        {
          turnout.addresses[0] = obj.value("address", invalidAddress);
        }

        const size_t movementTimeMillis = obj.value("move_ms", DefaultTurnoutMovement_ms);
        turnout.step = std::floor(0.5 * 1000.0 * 255.0 / float(movementTimeMillis));

        segment.turnout.index = stateData.turnouts.size(); // Current turnout index
        data.turnouts.push_back(turnout);

        auto& turnoutState = stateData.turnouts.emplace_back(TurnoutState{});
        if(segment.type == TrackSegment::Type::Turnout3Way ||
            segment.type == TrackSegment::Type::SingleSlipTurnout ||
            segment.type == TrackSegment::Type::DoubleSlipTurnout)
        {
          turnoutState.coils = 0x0A; // both closed
        }
      }

      if(getPointCount(segment.type) > 2)
      {
        if(const size_t n = obj.value("side", invalidIndex); n < getPointCount(segment.type))
        {
          startPointIndex = n;
        }
      }

      if(const auto fromId = obj.value<std::string>("from_id", {}); !fromId.empty())
      {
        const size_t fromPoint = obj.value("from_point", invalidIndex);

        if(auto it = data.trackSegmentId.find(fromId); it != data.trackSegmentId.end())
        {
          auto& fromSegment = data.trackSegments[it->second];
          const auto pointCount = getPointCount(fromSegment.type);
          bool unconnectedPointFound = false;
          for(size_t i = 0; i < pointCount; ++i)
          {
            if((fromSegment.nextSegmentIndex[i] == invalidIndex && fromPoint == invalidIndex) || fromPoint == i)
            {
              if(fromSegment.nextSegmentIndex[i] != invalidIndex)
              {
                throw std::runtime_error("point already connected");
              }
              curPoint = fromSegment.points[i];
              curRotation = getPointRotation(fromSegment, i);
              fromPointIndex = i;
              fromSegmentIndex = it->second;
              unconnectedPointFound = true;
              break;
            }
          }

          if(!unconnectedPointFound)
          {
            throw std::runtime_error("track element is already fully connected");
          }
        }
        else
        {
          throw std::runtime_error("from_id contains unknown id");
        }
      }
      else if(const auto fromSide = obj.value("from_side", invalidIndex);
               fromSide != invalidIndex &&
               fromSegmentIndex != invalidIndex &&
               getPointCount(data.trackSegments[fromSegmentIndex].type) > 2 &&
               fromSide < getPointCount(data.trackSegments[fromSegmentIndex].type) &&
               data.trackSegments[fromSegmentIndex].nextSegmentIndex[fromSide] == invalidIndex)
      {
        const auto& fromSegment = data.trackSegments[fromSegmentIndex];
        curPoint = fromSegment.points[fromSide];
        curRotation = getPointRotation(fromSegment, fromSide);
        fromPointIndex = fromSide;
      }
      else
      {
        if(obj.contains("x"))
        {
          curPoint.x = obj.value("x", 0.0f);
          fromSegmentIndex = invalidIndex;
        }
        if(obj.contains("y"))
        {
          curPoint.y = obj.value("y", 0.0f);
          fromSegmentIndex = invalidIndex;
        }
        if(obj.contains("rotation"))
        {
          curRotation = deg2rad(obj.value("rotation", 0.0f));
          fromSegmentIndex = invalidIndex;
        }
        updateView(data.view, curPoint);
      }

      if((segment.type == TrackSegment::Type::Turnout ||
           segment.type == TrackSegment::Type::Turnout3Way ||
           segment.type == TrackSegment::Type::SingleSlipTurnout ||
           segment.type == TrackSegment::Type::DoubleSlipTurnout) &&
          startPointIndex == 1)
      {
        segment.rotation = curRotation + pi;
        if(segment.rotation >= 2 * pi)
        {
          segment.rotation -= 2 * pi;
        }
        else if(segment.rotation < 0)
        {
          segment.rotation += 2 * pi;
        }

        segment.points[0].x = curPoint.x + segment.straight.length * std::cos(curRotation);
        segment.points[0].y = curPoint.y + segment.straight.length * std::sin(curRotation);
        nextPointIndex = 0;
      }
      else if((segment.type == TrackSegment::Type::Turnout && startPointIndex == 2) ||
               (segment.type == TrackSegment::Type::TurnoutCurved && (startPointIndex == 1 || startPointIndex == 2)) ||
               (segment.type == TrackSegment::Type::Turnout3Way && (startPointIndex == 2 || startPointIndex == 3)) ||
               (segment.type == TrackSegment::Type::SingleSlipTurnout && startPointIndex == 2) ||
               (segment.type == TrackSegment::Type::DoubleSlipTurnout && startPointIndex == 2))
      {
        const size_t curveIndex = ((segment.type == TrackSegment::Type::TurnoutCurved && startPointIndex == 2) || (segment.type == TrackSegment::Type::Turnout3Way && startPointIndex == 3)) ? 1 : 0;
        auto& curve = segment.curves[curveIndex];

        const float curAngle = (curve.angle > 0) ? (curRotation + pi) : curRotation;

        // Calc circle center:
        curve.center.x = curPoint.x - curve.radius * std::sin(curAngle);
        curve.center.y = curPoint.y + curve.radius * std::cos(curAngle);

        // Calc origin:
        segment.points[0].x = curve.center.x - curve.radius * std::sin(curAngle - curve.angle + pi);
        segment.points[0].y = curve.center.y + curve.radius * std::cos(curAngle - curve.angle + pi);

        curRotation -= curve.angle;

        segment.rotation = curRotation + pi;
        if(segment.rotation >= 2 * pi)
        {
          segment.rotation -= 2 * pi;
        }
        else if(segment.rotation < 0)
        {
          segment.rotation += 2 * pi;
        }

        nextPointIndex = 0;
      }
      else
      {
        segment.points[0] = curPoint;
        segment.rotation = curRotation;
        if(segment.rotation >= 2 * pi)
        {
          segment.rotation -= 2 * pi;
        }
        else if(segment.rotation < 0)
        {
          segment.rotation += 2 * pi;
        }

        if(segment.type == TrackSegment::Type::Curve || segment.type == TrackSegment::Type::TurnoutCurved)
        {
          curRotation += segment.curves[0].angle;
        }
      }

      // Keep value in range [0, 2 * pi)
      if(curRotation >= 2 * pi)
      {
        curRotation -= 2 * pi;
      }
      else if(curRotation < 0)
      {
        curRotation += 2 * pi;
      }

      assert(segment.points[0].isFinite()); // origin must be known now

      // Calculate center/length of curves:
      for(size_t i = 0; i < getCurveCount(segment.type); ++i)
      {
        auto& curve = segment.curves[i];
        if(!curve.center.isFinite()) // skip if already known
        {
          const float startAngle = (curve.angle < 0) ? (segment.rotation + pi) : segment.rotation;
          curve.center.x = segment.points[0].x - curve.radius * std::sin(startAngle);
          curve.center.y = segment.points[0].y + curve.radius * std::cos(startAngle);
        }
        curve.length = std::abs(curve.radius * curve.angle);
        assert(curve.center.isFinite());
      }

      // Calculate points if not known:
      switch(segment.type)
      {
      case TrackSegment::Type::Straight:
        if(!segment.points[1].isFinite())
        {
          segment.points[1] = straightEnd(segment);
          updateView(data.view, segment.points[1]);
        }
        break;

      case TrackSegment::Type::Curve:
        if(!segment.points[1].isFinite())
        {
          segment.points[1] = curveEnd(segment, 0);
          updateView(data.view, segment.curves[0], segment.rotation);
        }
        break;

      case TrackSegment::Type::Turnout:
        if(!segment.points[1].isFinite())
        {
          segment.points[1] = straightEnd(segment);
          updateView(data.view, segment.points[1]);
        }
        if(!segment.points[2].isFinite())
        {
          segment.points[2] = curveEnd(segment, 0);
          updateView(data.view, segment.curves[0], segment.rotation);
        }
        break;

      case TrackSegment::Type::TurnoutCurved:
        if(!segment.points[1].isFinite())
        {
          segment.points[1] = curveEnd(segment, 0);
          updateView(data.view, segment.curves[0], segment.rotation);
        }
        if(!segment.points[2].isFinite())
        {
          segment.points[2] = curveEnd(segment, 1);
          updateView(data.view, segment.curves[1], segment.rotation);
        }
        break;

      case TrackSegment::Type::Turnout3Way:
        if(!segment.points[1].isFinite())
        {
          segment.points[1] = straightEnd(segment);
          updateView(data.view, segment.points[1]);
        }
        if(!segment.points[2].isFinite())
        {
          segment.points[2] = curveEnd(segment, 0);
          updateView(data.view, segment.curves[0], segment.rotation);
        }
        if(!segment.points[3].isFinite())
        {
          segment.points[3] = curveEnd(segment, 1);
          updateView(data.view, segment.curves[1], segment.rotation);
        }
        break;

      default:
        assert(false);
        break;
      }
#ifndef NDEBUG
      for(size_t i = 1; i < getPointCount(segment.type); ++i)
      {
        assert(segment.points[i].isFinite()); // all points must be known now
      }
#endif

      // Set current point:
      assert(nextPointIndex < getPointCount(segment.type));
      curPoint = segment.points[nextPointIndex];

      // Sensors:
      if(const uint16_t sensorAddress = obj.value("sensor_address", invalidAddress); sensorAddress != invalidAddress)
      {
        const uint16_t sensorChannel = obj.value("sensor_channel", defaultChannel);

        auto &channelMap = data.sensorChannelMap[sensorChannel];
        auto it = channelMap.find(sensorAddress);
        if(it == channelMap.end())
        {
          // new sensor
          segment.sensor.index = data.sensors.size();
          data.sensors.emplace_back(Sensor{sensorChannel, sensorAddress, Sensor::Type::TrackCircuit});
          channelMap.insert({sensorAddress, segment.sensor.index});
          stateData.sensors.emplace_back(SensorState{{0}, 0, 0, SensorState::Encoding::CodeAbsent, false});
        }
        else
          segment.sensor.index = it->second;
      }

      loadTrackObjects(obj, data, stateData, segment, iface);

      if(fromSegmentIndex != invalidIndex)
      {
        assert(startPointIndex < getPointCount(segment.type));
        segment.nextSegmentIndex[startPointIndex] = fromSegmentIndex;

        assert(fromPointIndex < getPointCount(data.trackSegments[fromSegmentIndex].type));
        data.trackSegments[fromSegmentIndex].nextSegmentIndex[fromPointIndex] = data.trackSegments.size();
      }

      data.trackSegments.emplace_back(std::move(segment));

      switch(startPointIndex)
      {
      case 0:
        fromPointIndex = 1;
        if(segment.type == TrackSegment::Type::Turnout || segment.type == TrackSegment::Type::TurnoutCurved)
        {
          if(segment.nextSegmentIndex[1] != invalidIndex)
          {
            fromPointIndex = 2;
          }
        }
        if(segment.type == TrackSegment::Type::Turnout3Way)
        {
          if(segment.nextSegmentIndex[1] != invalidIndex)
          {
            fromPointIndex = 2;
          }
          if(segment.nextSegmentIndex[2] != invalidIndex)
          {
            fromPointIndex = 3;
          }
        }
        break;

      case 1:
      case 2:
      case 3:
        fromPointIndex = 0;
        break;

      default:
        assert(false);
        break;
      }

      fromSegmentIndex = data.trackSegments.size() - 1;
    }

    // Connect open ends:
    {
      auto findSegment = [&data](size_t start, Point point, size_t& index)
      {
        const size_t count = data.trackSegments.size();
        for(size_t i = start + 1; i < count; ++i)
        {
          auto& segment = data.trackSegments[i];
          const auto pointCount = getPointCount(segment.type);
          for(size_t j = 0; j < pointCount; ++j)
          {
            if(segment.nextSegmentIndex[j] == invalidIndex && pointsClose(point, segment.points[j]))
            {
              segment.nextSegmentIndex[j] = start;
              index = i;
              return;
            }
          }
        }
      };

      const size_t count = data.trackSegments.size() - 1;
      for(size_t i = 0; i < count; ++i)
      {
        auto& segment = data.trackSegments[i];
        const auto pointCount = getPointCount(segment.type);
        for(size_t j = 0; j < pointCount; ++j)
        {
          if(segment.nextSegmentIndex[j] == invalidIndex)
          {
            findSegment(i, segment.points[j], segment.nextSegmentIndex[j]);
          }
        }
      }
    }
  }
}

Simulator::StaticData Simulator::load(const nlohmann::json& world, StateData& stateData, TrainTypeInterface *iface)
{
  StaticData data;

  const float scaleA = world.value("scale_num", 1.0);
  const float scaleB = world.value("scale_den", 1.0);
  if(scaleB > 0.0001)
    data.worldScale = scaleA / scaleB;

  data.trainWidth = world.value("train_width", data.trainWidth);
  data.trainCouplingLength = world.value("train_coupling_length", data.trainCouplingLength);
  data.defaultSignalScale = world.value("default_signal_scale", data.defaultSignalScale);
  data.defaultCenterPos.x = world.value("default_center_x", data.defaultCenterPos.x);
  data.defaultCenterPos.y = world.value("default_center_y", data.defaultCenterPos.y);
  data.defaultViewScale = world.value("default_zoom", data.defaultViewScale);

  loadTrackplan(world, data, stateData, iface);

  if(auto vehicles = world.find("vehicles"); vehicles != world.end() && vehicles->is_array())
  {
    for(const auto& object : *vehicles)
    {
      if(!object.is_object())
      {
        continue;
      }

      Vehicle vehicle;
      vehicle.vehicleId = stateData.mLastVehicleId++;
      vehicle.length = object.value("length", 20.0f);
      vehicle.color = stringToEnum<Color>(object.value<std::string_view>("color", {})).value_or(Color::Red);
      stateData.vehicles.insert({vehicle.vehicleId, vehicle});
    }
  }

  if(auto misc = world.find("misc"); misc != world.end() && misc->is_array())
  {
    for(const auto& object : *misc)
    {
      if(!object.is_object())
      {
        continue;
      }

      Misc item;

      item.origin.x = object.value("x", std::numeric_limits<float>::quiet_NaN());
      item.origin.y = object.value("y", std::numeric_limits<float>::quiet_NaN());
      item.color = stringToEnum<Color>(object.value<std::string_view>("color", {})).value_or(Color::None);
      if(!item.origin.isFinite() || item.color == Color::None)
      {
        continue;
      }

      const auto type = object.value<std::string_view>("type", {});
      if(type == "rectangle")
      {
        item.type = Misc::Type::Rectangle;
        item.rotation = deg2rad(object.value("rotation", 0.0f));
        item.height = object.value("height", std::numeric_limits<float>::quiet_NaN());
        item.width = object.value("width", std::numeric_limits<float>::quiet_NaN());
        if(!std::isfinite(item.height) || !std::isfinite(item.width))
        {
          continue;
        }
      }
      else
      {
        continue;
      }

      updateView(data.view, item.origin);
      switch(item.type)
      {
      case Misc::Type::Rectangle:
      {
        const float cosRotation = std::cos(item.rotation);
        const float sinRotation = std::sin(item.rotation);
        updateView(data.view, {item.origin.x + item.width * cosRotation, item.origin.y + item.width * sinRotation}); // top right
        updateView(data.view, {item.origin.x - item.height * sinRotation, item.origin.y + item.height * cosRotation}); // bottom left
        updateView(data.view, {item.origin.x - item.height * sinRotation + item.width * cosRotation, item.origin.y + item.height * cosRotation + item.width * sinRotation}); // bottom right
        break;
      }
      }

      data.misc.emplace_back(std::move(item));
    }
  }

  if(auto images = world.find("images"); images != world.end() && images->is_array())
  {
    for(const auto& object : *images)
    {
      if(!object.is_object())
      {
        continue;
      }

      ImageRef item;

      item.origin.x = object.value("x", std::numeric_limits<float>::quiet_NaN());
      item.origin.y = object.value("y", std::numeric_limits<float>::quiet_NaN());
      item.fileName = object.value<std::string_view>("file", {});
      item.rotation = deg2rad(object.value("rotation", 0.0f));
      item.opacity = object.value("opacity", 1.0);

      const float pxCount = object.value("n_px", std::numeric_limits<float>::quiet_NaN());
      const float mtCount = object.value("n_mt", std::numeric_limits<float>::quiet_NaN());

      if(!item.origin.isFinite() || item.fileName.empty() || pxCount == 0)
      {
        continue;
      }

      item.ratio = mtCount / pxCount;

      updateView(data.view, item.origin);

      data.images.emplace_back(std::move(item));
    }
  }

  data.view.top -= data.trainWidth;
  data.view.left -= data.trainWidth;
  data.view.bottom += data.trainWidth;
  data.view.right += data.trainWidth;

  assert(data.sensors.size() == stateData.sensors.size());

  return data;
}

bool Simulator::addTrain(const std::string_view& name, DecoderProtocol proto, uint16_t addr,
                         const std::vector<Train::VehicleItemFull> &vehicles, size_t segmentIndex,
                         quint32 &trainIdOut, float startPos, TrainPlacement placement, bool reverse)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  if(getSimModeUnlocked() == SimMode::Replica)
    return false; // Only add trains if received command from master

  if(name.empty())
    return false;

  std::unique_ptr<Train> train(new Train);
  train->name = name;
  train->protocol = proto;
  train->address = addr;
  train->vehicles.reserve(vehicles.size());
  train->state.reverse = reverse;

  bool empty = true;
  for(const auto& item : vehicles)
  {
    train->length += item.vehicle.length + (empty ? 0.0f : staticData.trainCouplingLength);
    empty = false;
  }

  if(empty)
    return false;

  float segmentLength = 0.0f;

  {
    const auto& segment = staticData.trackSegments[segmentIndex];
    if(isTurnoutUnknownState(segment, m_stateData))
      return false;
    segmentLength = getSegmentLength(segment, m_stateData);

    if(startPos < 0.0f || startPos >= segmentLength)
      startPos = segmentLength / 2.0;
  }

  const float placingMargin = std::max(0.1, staticData.trainCouplingLength / 2.0);

  float extraLength[2] = {0.0, 0.0};
  for(int i = 0; i < 2; i++)
  {
    bool segmentInverted = (i == 1);
    size_t curSegmentIndex = segmentIndex;

    while(extraLength[i] < (train->length + placingMargin))
    {
      const auto& curSegment = staticData.trackSegments[curSegmentIndex];
      const size_t nextSegmentIndex = getNextSegmentIndex(curSegment, segmentInverted, m_stateData);

      if(nextSegmentIndex == invalidIndex)
      {
        break; // no next segment
      }

      const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];
      if(isTurnoutUnknownState(nextSegment, m_stateData))
        break;

      if(nextSegment.nextSegmentIndex[0] == curSegmentIndex)
      {
        // TODO: double slip switches
        segmentInverted = true;
      }
      else
      {
        segmentInverted = false;

        // Check turnout is in correct position
        bool invalidTurnout = false;

        switch (nextSegment.type)
        {
        case Simulator::TrackSegment::Type::Straight:
        case Simulator::TrackSegment::Type::Curve:
          break;
        case Simulator::TrackSegment::Type::Turnout:
        case Simulator::TrackSegment::Type::TurnoutCurved:
        {
          const auto state = m_stateData.turnouts.at(nextSegment.turnout.index).state;
          if(state == Simulator::TurnoutState::State::Closed && nextSegment.nextSegmentIndex[1] != curSegmentIndex)
            invalidTurnout = true;
          else if(state == Simulator::TurnoutState::State::Thrown && nextSegment.nextSegmentIndex[2] != curSegmentIndex)
            invalidTurnout = true;
          break;
        }
        case Simulator::TrackSegment::Type::Turnout3Way:
        {
          const auto state = m_stateData.turnouts.at(nextSegment.turnout.index).state;
          if(state == Simulator::TurnoutState::State::Closed && nextSegment.nextSegmentIndex[1] != curSegmentIndex)
            invalidTurnout = true;
          else if(state == Simulator::TurnoutState::State::ThrownLeft && nextSegment.nextSegmentIndex[2] != curSegmentIndex)
            invalidTurnout = true;
          else if(state == Simulator::TurnoutState::State::ThrownRight && nextSegment.nextSegmentIndex[3] != curSegmentIndex)
            invalidTurnout = true;
          break;
        }
        default:
          assert(false);
          break;
        }

        if(invalidTurnout)
          break;
      }

      extraLength[i] += getSegmentLength(nextSegment, m_stateData);
      curSegmentIndex = nextSegmentIndex;
    }
  }

  // Extra length is calculated from adjacent segment start
  // Convert it to distance from startPos
  extraLength[0] += startPos;
  extraLength[1] += segmentLength - startPos;

  // Now check if there are vehicles backwards
  float otherVehicleDistance = 0.0f;
  Vehicle *otherVehicle = getVehicleNear(segmentIndex, startPos, train->length + placingMargin,
                                         false, true, &otherVehicleDistance);
  if(otherVehicle)
  {
    otherVehicleDistance -= placingMargin;
    if(otherVehicleDistance < extraLength[0])
      extraLength[0] = otherVehicleDistance;
  }

  // And check if there are vehicles forward
  otherVehicle = getVehicleNear(segmentIndex, startPos, train->length + placingMargin,
                                true, false, &otherVehicleDistance);
  if(otherVehicle)
  {
    otherVehicleDistance -= placingMargin;
    if(otherVehicleDistance < extraLength[1])
      extraLength[1] = otherVehicleDistance;
  }

  const float totalAvaliableLength = extraLength[0] + extraLength[1];
  if(totalAvaliableLength < (train->length + 2 * placingMargin))
    return false;

  float trainStartPos = 0.0f;
  switch (placement)
  {
  case TrainPlacement::PlaceCenter:
    trainStartPos = startPos - train->length / 2.0 - placingMargin;
    break;
  case TrainPlacement::PlaceStart:
    trainStartPos = startPos - train->length - placingMargin;
    break;
  case TrainPlacement::PlaceEnd:
    trainStartPos = startPos + placingMargin;
    break;
  default:
    break;
  }

  if(trainStartPos < (startPos - extraLength[0]))
  {
    trainStartPos = startPos - extraLength[0] + placingMargin;
  }

  float trainEndPos = trainStartPos + train->length;
  if(trainEndPos >= (startPos + extraLength[1]))
  {
    trainEndPos = startPos + extraLength[1] - placingMargin;
    trainStartPos = trainEndPos - train->length;
  }

  // Find start segment
  bool segmentInverted = false;
  size_t curSegmentIndex = segmentIndex;
  float curPos = trainStartPos;
  float curSegmentLenght = segmentLength;

  while(curPos < 0)
  {
    const auto& curSegment = staticData.trackSegments[curSegmentIndex];
    const size_t nextSegmentIndex = getNextSegmentIndex(curSegment, segmentInverted, m_stateData);

    assert(nextSegmentIndex != invalidIndex);

    const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];
    segmentInverted = (nextSegment.nextSegmentIndex[0] == curSegmentIndex);

    curSegmentLenght = getSegmentLength(nextSegment, m_stateData);
    curPos += curSegmentLenght;
    curSegmentIndex = nextSegmentIndex;
  }

  auto advancePos = [&segmentInverted, &curSegmentIndex, &curPos, &curSegmentLenght, this](float delta)
  {
    curPos += delta;
    if(curPos < curSegmentLenght)
      return; // Still inside current segment

    while(curPos >= curSegmentLenght)
    {
      // Go next segment (opposite direction, so !segmentInverted)
      const auto& curSegment = staticData.trackSegments[curSegmentIndex];
      curPos -= curSegmentLenght;

      const size_t nextSegmentIndex = getNextSegmentIndex(curSegment, !segmentInverted, m_stateData);
      assert(nextSegmentIndex != invalidIndex);

      const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];
      segmentInverted = (nextSegment.nextSegmentIndex[0] != curSegmentIndex);

      curSegmentLenght = getSegmentLength(nextSegment, m_stateData);
      curSegmentIndex = nextSegmentIndex;
    }
  };

  // Generate Train ID
  train->trainId = m_stateData.mLastTrainId++;

  // Add wagons to train only when sure train will be added
  for(const Train::VehicleItemFull& item : vehicles)
  {
    Vehicle *vehicle = addVehicle(item.vehicle.length, item.vehicle.color, item.vehicle.typeIdx);

    if(!vehicle)
      continue;

    vehicle->activeTrainId = train->trainId;
    train->vehicles.push_back({vehicle->vehicleId, item.reversed});
  }

  // Place wagons in segments
  bool first = true;
  for(Train::VehicleItem& item : train->vehicles | std::views::reverse)
  {
    if(first)
      first = false;
    else
      advancePos(staticData.trainCouplingLength);

    float pos = curPos;
    if(segmentInverted)
      pos = curSegmentLenght - pos;

    Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);

    VehicleState& vehicleState = vehicle->state;
    vehicleState.rear.segmentIndex = curSegmentIndex;
    vehicleState.rear.distance = pos;
    vehicleState.rear.segmentDirectionInverted = segmentInverted;
    maybeAddVehicleSegment(vehicle, curSegmentIndex);

    const auto& rearSegment = staticData.trackSegments[curSegmentIndex];
    if(rearSegment.sensor.index != invalidIndex)
    {
      auto& sensor = m_stateData.sensors[rearSegment.sensor.index];
      sensor.occupied += 1;
    }

    advancePos(vehicle->length);
    pos = curPos;
    if(segmentInverted)
      pos = curSegmentLenght - pos;

    vehicleState.front.segmentIndex = curSegmentIndex;
    vehicleState.front.distance = pos;
    vehicleState.front.segmentDirectionInverted = segmentInverted;
    maybeAddVehicleSegment(vehicle, curSegmentIndex);

    const auto& frontSegment = staticData.trackSegments[curSegmentIndex];
    if(frontSegment.sensor.index != invalidIndex)
    {
      auto& sensor = m_stateData.sensors[frontSegment.sensor.index];
      sensor.occupied += 1;
    }

    if(item.reversed)
    {
      std::swap(vehicleState.front, vehicleState.rear);
    }
  }

  if(train->speedMax < 0.0001)
    train->speedMax = defaultSpeedTickRate * staticData.worldScale;

  if(trainTypeInterface)
  {
    trainTypeInterface->setupTrainSpeedPowered(this, train.get());

    if(!train->isPowered)
      train->state.mode = TrainState::Mode::Manual;
  }

  trainIdOut = train->trainId;
  auto pair = m_stateData.trains.insert({train->trainId, *train});

  onTrainAddedRemoved(true, trainIdOut, invalidTrainId);

  if(getSimModeUnlocked() == SimMode::Master)
  {
    sendTrainFullRefreshReplica(SimulatorProtocol::OpCode::TrainAdded,
                                &pair.first->second);
  }

  return true;
}

bool Simulator::requestAddTrain(const std::string &name, const std::vector<Train::VehicleItemFull> &vehicles,
                                size_t segmentIndex, quint32 token, float startPos,
                                TrainPlacement placement, bool invertDir)
{
  if(getSimMode() != SimMode::Replica)
    return false;

  boost::asio::post(
    m_networkContext, [this, name, vehicles, segmentIndex, token, startPos, placement, invertDir]()
    {
      QByteArray buf;
      buf.reserve(100);

      QDataStream stream(&buf, QIODeviceBase::WriteOnly);
      stream.setByteOrder(QDataStream::LittleEndian);
      stream << uint8_t(SimulatorProtocol::OpCode::TrainRequestAdd);
      stream << uint16_t(0); // Size placeholder
      serializeTrainAddRequest(stream, name, vehicles, segmentIndex, token, startPos, placement, invertDir);

      sendToMaster(buf);
    });
  return true;
}

bool Simulator::removeTrainUser(quint32 trainId)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  if(!m_stateData.trains.contains(trainId))
    return false;

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    boost::asio::post(
      m_networkContext, [this, trainId]()
      {
        send(SimulatorProtocol::TrainRemoved(trainId, true));
      });
    return true;
  }
  else
  {
    return removeTrain(trainId, true);
  }
}

bool Simulator::removeTrain(quint32 trainId, bool removeWagons, quint32 suggestedTrainId)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  auto it = m_stateData.trains.find(trainId);
  if(it == m_stateData.trains.end())
    return false;

  if(!m_stateData.trains.contains(suggestedTrainId))
    suggestedTrainId = invalidTrainId;

  Train &train = it->second;
  destroyTrain(train, removeWagons);
  it = m_stateData.trains.erase(it);
  onTrainAddedRemoved(false, trainId, suggestedTrainId);

  if(getSimModeUnlocked() == SimMode::Master)
  {
    // TODO: suggest new active train
    boost::asio::post(
      m_networkContext, [this, trainId, suggestedTrainId]()
      {
        sendReplicas(SimulatorProtocol::TrainRemoved(trainId, false, suggestedTrainId));
      });
  }

  return true;
}

Simulator::Vehicle *Simulator::addVehicle(float length, Color color, size_t typeIdx, quint32 suggestId)
{    
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);

  Vehicle vehicle;

  if(suggestId == invalidTrainId)
    vehicle.vehicleId = m_stateData.mLastVehicleId++;
  else
    vehicle.vehicleId = suggestId;

  vehicle.length = length;
  vehicle.color = color;
  vehicle.typeIdx = typeIdx;

  auto result = m_stateData.vehicles.insert({vehicle.vehicleId, vehicle});
  return &result.first->second;
}

bool Simulator::removeVehicle(Vehicle *vehicle)
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  auto it = m_stateData.vehicles.find(vehicle->vehicleId);
  if(it == m_stateData.vehicles.end())
    return false;

  if(&it->second != vehicle)
    return false;

  if(vehicle->activeTrainId != invalidTrainId)
    return false;

  // Free track sensors
  if(vehicle->state.front.segmentIndex != invalidIndex)
  {
    const size_t oldSegmentIndex = vehicle->state.front.segmentIndex;
    vehicle->state.front.segmentIndex = invalidIndex;

    const auto& segment = staticData.trackSegments[oldSegmentIndex];
    if(segment.sensor.index != invalidIndex)
    {
      auto& sensor = m_stateData.sensors[segment.sensor.index];
      sensor.occupied--;
    }

    maybeRemoveVehicleSegment(vehicle, oldSegmentIndex);
  }

  if(vehicle->state.rear.segmentIndex != invalidIndex)
  {
    const size_t oldSegmentIndex = vehicle->state.rear.segmentIndex;
    vehicle->state.rear.segmentIndex = invalidIndex;

    const auto& segment = staticData.trackSegments[oldSegmentIndex];
    if(segment.sensor.index != invalidIndex)
    {
      auto& sensor = m_stateData.sensors[segment.sensor.index];
      sensor.occupied--;
    }
    maybeRemoveVehicleSegment(vehicle, oldSegmentIndex);
  }

  m_stateData.vehicles.erase(it);
  return true;
}

void Simulator::sendTrainFullRefreshReplica(SimulatorProtocol::OpCode op,
                                            Train *train, bool networkThread, bool isFull)
{
  QByteArray buf;
  buf.reserve(100);

  QDataStream stream(&buf, QIODeviceBase::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);
  stream << uint8_t(op);
  stream << uint16_t(0); // Size placeholder
  serializeTrain(stream, *train, isFull, networkThread ? m_networkStateData : m_stateData);

  if(networkThread)
  {
    sendReplicas(buf);
  }
  else
  {
    std::vector<Vehicle> vehicles;
    vehicles.reserve(train->vehicles.size());
    for(const auto &item : train->vehicles)
    {
      const auto &vehicle = m_stateData.getVehicleById(item.vehicleId);
      vehicles.push_back(*vehicle);
    }

    boost::asio::post(
        m_networkContext, [this, buf=buf, trainCopy=*train, vehiclesCopy=std::move(vehicles)]()
        {
          // Cherry pick new state
          m_networkStateData.trains[trainCopy.trainId] = trainCopy;

          // We also need to cherry pick vehicles
          for(const auto &vehicle : vehiclesCopy)
          {
            m_networkStateData.vehicles[vehicle.vehicleId] = vehicle;
          }

          // TODO: avoid another buf copy
          QByteArray bufCopy = buf;
          sendReplicas(bufCopy);
        });
  }
}

void Simulator::updateTrainNextSignal(Train &train, bool next)
{
  Simulator::TrainState::SignalCache &cache = next ? train.state.nextSignal : train.state.prevSignal;

  if(!cache.dirty && train.state.mode != TrainState::Mode::Manual)
    return;

  // De-register
  clearTrainSignalCache(m_stateData, train, next);

  if(train.state.mode == TrainState::Mode::Manual)
    return;
  if(train.state.mode == TrainState::Mode::SemiAutomatic && train.state.speed == 0.0f)
    return;

  if(!train.state.isOnStationStop && !next)
    return; // Only look at previous signal if inside a station stop

  const bool trainReverse = (train.state.reverse == next);
  const Train::VehicleItem& firstItem = trainReverse ? train.vehicles.back() : train.vehicles.front();
  const Vehicle *firstVehicle = m_stateData.getVehicleById(firstItem.vehicleId);
  const VehicleState::Face& face = (firstItem.reversed == trainReverse) ? firstVehicle->state.front : firstVehicle->state.rear;

  float totalDistance = 0.0f;
  float signalPositionInSegment = 0.0f;

  auto helper = [&cache, &totalDistance, &signalPositionInSegment, train, this](bool dirFwd, size_t segmentId, float startPos) -> MainSignal *
  {
    const auto &curSegment = staticData.trackSegments[segmentId];

    switch (curSegment.type)
    {
    case TrackSegment::Type::Turnout:
    case TrackSegment::Type::TurnoutCurved:
    case TrackSegment::Type::Turnout3Way:
    {
      cache.turnouts.push_back(curSegment.turnout.index);
      m_stateData.turnouts[curSegment.turnout.index].watchingTrainIDs.push_back(train.trainId);
      break;
    }
    default:
      break;
    }

    if(dirFwd)
    {
      for(const TrackSegment::Object& trackObj : curSegment.objects)
      {
        if(trackObj.position < startPos)
          continue;

        if(trackObj.type != TrackSegment::Object::Type::MainSignal)
          continue;

        if(trackObj.allowedDirection == TrackSegment::Object::AllowedDirections::Backwards)
          continue;

        auto it = m_stateData.mainSignals.find(trackObj.signalName);
        if(it == m_stateData.mainSignals.end())
          continue;

        signalPositionInSegment = trackObj.position;
        totalDistance += trackObj.position - startPos;
        return &it->second;
      }

      totalDistance += getSegmentLength(curSegment, m_stateData) - startPos;
    }
    else
    {
      for(const TrackSegment::Object& trackObj : curSegment.objects | std::views::reverse)
      {
        if(trackObj.position > startPos)
          continue;

        if(trackObj.type != TrackSegment::Object::Type::MainSignal)
          continue;

        if(trackObj.allowedDirection == TrackSegment::Object::AllowedDirections::Forward)
          continue;

        auto it = m_stateData.mainSignals.find(trackObj.signalName);
        if(it == m_stateData.mainSignals.end())
          continue;

        signalPositionInSegment = trackObj.position;
        totalDistance += startPos - trackObj.position;
        return &it->second;
      }

      totalDistance += startPos;
    }

    return nullptr;
  };

  size_t segmentIndex = face.segmentIndex;
  bool dirFwd = face.segmentDirectionInverted == trainReverse;
  float startPos = face.distance;

  while(totalDistance < 1000)
  {
    if(MainSignal *s = helper(dirFwd, segmentIndex, startPos))
    {
      cache.signal = s;
      cache.segmentIdx = segmentIndex;
      cache.distance = totalDistance;
      cache.posInSegment = signalPositionInSegment;
      return;
    }

    const auto &curSegment = staticData.trackSegments[segmentIndex];

    const auto nextSegmentIndex = getNextSegmentIndex(curSegment, dirFwd, m_stateData);
    if(nextSegmentIndex == invalidIndex)
    {
      break; // no next segment
    }

    const auto& nextSegment = staticData.trackSegments[nextSegmentIndex];
    dirFwd = nextSegment.nextSegmentIndex[0] == segmentIndex;
    segmentIndex = nextSegmentIndex;

    startPos = 0.0;
    if(!dirFwd)
      startPos = getSegmentLength(nextSegment, m_stateData);
  }

  // No signal found
  clearTrainSignalCache(m_stateData, train, next);
}

void Simulator::destroyTrain(Train &train, bool removeWagons)
{
  clearTrainSignalCache(m_stateData, train, true);
  clearTrainSignalCache(m_stateData, train, false);

  for(const auto &item : train.vehicles)
  {
    Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);
    vehicle->activeTrainId = invalidTrainId;
    if(removeWagons)
      removeVehicle(vehicle);
  }
  train.vehicles.clear();
}

bool Simulator::checkNextSignal(Train &train)
{
  if(train.state.mode == TrainState::Mode::Manual)
    return true;
  if(train.state.mode == TrainState::Mode::SemiAutomatic && train.state.speed == 0.0f)
    return true;

  const Train::VehicleItem& firstItem = train.state.reverse ? train.vehicles.back() : train.vehicles.front();
  const Vehicle *firstVehicle = m_stateData.getVehicleById(firstItem.vehicleId);
  const VehicleState::Face& face = (firstItem.reversed == train.state.reverse) ? firstVehicle->state.front : firstVehicle->state.rear;

  if(!train.state.nextSignal.signal)
  {
    const float length = getSegmentLength(staticData.trackSegments[face.segmentIndex], m_stateData);
    if(length >= 100)
    {
      // Long segments do not find signal and also do not set dirty
      // Set dirty near to end to re-trigger signal scan
      const bool dirFwd = face.segmentDirectionInverted == train.state.reverse;
      if((dirFwd && (length - face.distance) < 600) || (!dirFwd && face.distance < 600))
      {
        train.state.nextSignal.dirty = true;
      }
    }
  }

  static const std::pair<float, float> SpeedCurve[] = {
    {30.0, 0.0},
    {30.2, 1.5},
    {32,   4.42},
    {35,   8.83},
    {80,  29.91}, // 46
    {120,  31.19}, // 50
    {140, 44.10}, // 100
    {150, 54.01},
    {200, 62.36},
    {300, 76.38},
    {400, 88.19},
    {500, 98.60},
    {600, 108},
    {700, 116.65},
    {800, 124.71},
    {900, 132.27},
    {1000, 139.43},
    {1200, 152.74}
  };

  static const size_t SpeedCurveSz = sizeof(SpeedCurve)/sizeof(SpeedCurve[0]);
  static const float BRAKE_DISTANCE = SpeedCurve[SpeedCurveSz - 1].first;
  static const float STOP_DISTANCE = SpeedCurve[0].first;
  static const float DISTANCE_30_KMH = SpeedCurve[5].first;

  uint16_t speedLimit = train.state.otherPreviousSignalSpeed;
  if(MainSignal::decodeRawSpeed(train.state.previousSignalSpeed) < MainSignal::decodeRawSpeed(speedLimit))
    speedLimit = train.state.previousSignalSpeed;
  if(MainSignal::decodeRawSpeed(train.state.passedSignalSpeed) < MainSignal::decodeRawSpeed(speedLimit))
    speedLimit = train.state.passedSignalSpeed;

  if(speedLimit == 0 ||
      speedLimit == MainSignal::SpecialSpeeds::INVALID)
    speedLimit = 30; // Maybe we SPAD? or maybe wrong values

  uint16_t nextSignalSpeed = train.state.passedSignalNextSpeed;
  if(train.state.nextSignal.signal)
    nextSignalSpeed = train.state.nextSignal.signal->getCurrentSpeed(train.state.passedSignalNextSpeed);

  const float speedLimitDecoded = MainSignal::decodeRawSpeed(speedLimit) * SpeedKmHtoTick;
  const float nextSignalSpeedDecoded = MainSignal::decodeRawSpeed(nextSignalSpeed) * SpeedKmHtoTick;

  // Ensure we are not above current speed limit
  if(train.state.speed > speedLimitDecoded)
  {
    std::cout << "TRAIN RESET SPEED FROM: " << train.state.speed / SpeedKmHtoTick << " TO: " << speedLimitDecoded / SpeedKmHtoTick
              << std::endl;

    setTrainSpeed(speedLimitDecoded, true);
  }

  if(nextSignalSpeed == MainSignal::SpecialSpeeds::OVERRIDE_BRAKE && train.state.targetSpeed > 100)
  {
    // If we have short brake distance, start braking from previous signal until 100 km/h is reached
    setTrainSpeed(train, 100 * SpeedKmHtoTick, false);
  }
  else if(speedLimit == MainSignal::SpecialSpeeds::NO_LIMIT)
  {
    // We have no imposed limit from previous signal, accelerate to next signal speed
    if(!train.state.nextSignal.signal || train.state.nextSignal.distance > BRAKE_DISTANCE)
      setTrainSpeed(train, train.speedMax, false); // If next signal is far away, accelerate to max speed
    else
      setTrainSpeed(train, nextSignalSpeedDecoded, false);
  }

  const float totalDistance = train.state.nextSignal.distance;
  if(train.state.nextSignal.signal && totalDistance <= BRAKE_DISTANCE)
  {
    float maxSpeed = train.speedMax;
    float targetSpeed = nextSignalSpeedDecoded;

    for(size_t i = 0; i < SpeedCurveSz; i++)
    {
      const auto& entry = SpeedCurve[i];

      if(totalDistance < entry.first)
      {
        // Decrease to set size
        maxSpeed = nextSignalSpeedDecoded + (entry.second * SpeedKmHtoTick);

        // Set target to next speed
        if(i > 0)
          targetSpeed = nextSignalSpeedDecoded + (SpeedCurve[i - 1].second * SpeedKmHtoTick);
        break;
      }
    }

    maxSpeed = std::min(maxSpeed, train.speedMax);
    targetSpeed = std::min(targetSpeed, train.state.targetSpeed);
    targetSpeed = std::min(targetSpeed, train.speedMax);

    if(train.state.speed > maxSpeed)
    {
      std::cout << "TRAIN DECELERATE HARD FROM: " << train.state.speed / SpeedKmHtoTick << " TO: " << maxSpeed / SpeedKmHtoTick
                << std::endl;

      setTrainSpeed(train, maxSpeed, true);
    }

    if(train.state.targetSpeed > targetSpeed)
      setTrainSpeed(train, targetSpeed, false);

    if(train.state.speed == 0.0 && totalDistance <= STOP_DISTANCE && train.state.nextSignal.signal->isStop())
      return false; // Stop at STOP_DISTANCE from a red signal
  }

  const bool underSignalSpeed = train.state.speed < nextSignalSpeedDecoded && train.state.speed < speedLimitDecoded;
  if(train.state.targetSpeed < (30 * SpeedKmHtoTick - ADJ_MARGIN) && (!train.state.nextSignal.signal || totalDistance > STOP_DISTANCE || underSignalSpeed))
  {
    // Get near at 30 km/h to next signal if stopped or currently at less than 30 km/h
    // If stopped at less than 30 meters from signal, remain stopped
    if(!train.state.nextSignal.signal || totalDistance >= DISTANCE_30_KMH || underSignalSpeed)
    {
      setTrainSpeed(train, 30 * SpeedKmHtoTick);
    }
    else
    {
      float targetSpeed = train.speedMax;

      for(size_t i = 0; i < SpeedCurveSz; i++)
      {
        const auto& entry = SpeedCurve[i];
        if(totalDistance < entry.first)
        {
          if(i > 1) // Avoid also first entry
            targetSpeed = nextSignalSpeedDecoded + (SpeedCurve[i - 1].second * SpeedKmHtoTick);
          else
            targetSpeed = nextSignalSpeedDecoded + (entry.second * SpeedKmHtoTick);
          break;
        }
      }

      targetSpeed = std::min(targetSpeed, train.speedMax);
      targetSpeed = std::min(targetSpeed, speedLimitDecoded);

      if(train.state.targetSpeed < targetSpeed)
        setTrainSpeed(train, targetSpeed);
    }
  }

  return true;
}

void Simulator::setTrainSpeedFactor(float val)
{
  val = std::clamp(val, 1.0f, 7.0f); // 100% to 700%

  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  if(val == m_stateData.trainSpeedFactor)
    return;

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    // Send to Master and DO NOT update our value
    boost::asio::post(
      m_networkContext, [this, factor=val]()
      {
        send(SimulatorProtocol::TrainSpeedFactorChanged(factor));
      });
    return;
  }

  m_stateData.trainSpeedFactor = val;

  if(getSimModeUnlocked() == SimMode::Master)
  {
    boost::asio::post(
      m_networkContext, [this, factor=m_stateData.trainSpeedFactor]()
      {
        // Cherry pick value
        m_networkStateData.trainSpeedFactor = factor;
        sendReplicas(SimulatorProtocol::TrainSpeedFactorChanged(factor));
      });
  }
}

void Simulator::loadTrains(const nlohmann::json &world)
{
  if(auto trains = world.find("trains"); trains != world.end() && trains->is_array())
  {
    for(const auto& object : *trains)
    {
      if(!object.is_object() || !object.contains("vehicles") || !object["vehicles"].is_array())
      {
        continue;
      }

      const std::string_view name = object.value<std::string_view>("name", {});
      if(name.empty())
        continue;

      size_t segmentIndex = invalidIndex;

      if(const auto trackId = object.value<std::string>("track_id", {}); !trackId.empty())
      {
        if(auto it = staticData.trackSegmentId.find(trackId); it != staticData.trackSegmentId.end())
        {
          segmentIndex = it->second;
        }
      }

      if(segmentIndex == invalidIndex)
      {
        segmentIndex = 0; // in case there is no free segment
        for(size_t i = 0; i < staticData.trackSegments.size(); ++i)
        {
          const auto& segment = staticData.trackSegments[i];
          if(segment.hasSensor() && m_stateData.sensors[segment.sensor.index].occupied == 0)
          {
            segmentIndex = i;
            break;
          }
        }
      }

      std::vector<Train::VehicleItem> vehicles;
      for(const auto& vehicleObj : object["vehicles"])
      {
        if(!vehicleObj.is_object())
        {
          continue;
        }

        Vehicle *vehicle = nullptr;
        // TODO: use vehicle ID? or declare vehicles inside train JSON object
        //if(const auto it = m_stateData.vehicles.find(vehicleName); it != m_stateData.vehicles.end())
        //  vehicle = it->second;

        if(!vehicle || vehicle->activeTrainId != invalidTrainId)
          continue;

        Train::VehicleItem item{vehicle->vehicleId, false};
        item.reversed = vehicleObj.value("reversed", item.reversed);

        vehicles.push_back(item);
      }

      // quint32 unused1 = invalidTrainId;
      // addTrain(name,
      //          stringToEnum<DecoderProtocol>(object.value<std::string_view>("protocol", {})).value_or(DecoderProtocol::None),
      //          object.value("address", 3),
      //          vehicles,
      //          segmentIndex,
      //          unused1);
    }
  }
}

Simulator::Vehicle *Simulator::getVehicleNearHelper(size_t segmentIdx, float pos, float maxDistance,
                                                    bool canGoForward, bool canGoBackwards, Vehicle *bestVehicle, float &bestDistance)
{
  auto it = m_stateData.segmentVehicles.find(segmentIdx);
  if(it == m_stateData.segmentVehicles.end())
    return nullptr;

  SegmentVehicles &seg = it->second;
  if(seg.vehicleIDs.empty())
    return nullptr;

  for(quint32 otherId : seg.vehicleIDs)
  {
    Vehicle *vehicle = m_stateData.getVehicleById(otherId);
    if(!vehicle)
      continue;

    const VehicleState::Face &front = vehicle->state.front;
    const VehicleState::Face &rear = vehicle->state.rear;

    if(front.segmentIndex == segmentIdx && rear.segmentIndex == segmentIdx)
    {
      // Both in segment
      float posA = front.distance;
      float posB = rear.distance;
      if(posA > posB)
        std::swap(posA, posB);

      if(posA <= pos && pos <= posB)
        return vehicle;

      if(!canGoBackwards && posA < pos && posB < pos)
        continue;

      if(!canGoForward && posA > pos && posB > pos)
        continue;

      const float distance = std::min(std::abs(posA - pos), std::abs(posB - pos));
      if(distance < maxDistance && (!bestVehicle || distance < bestDistance))
      {
        bestVehicle = vehicle;
        bestDistance = distance;
      }
      continue;
    }
    else if(front.segmentIndex == segmentIdx || rear.segmentIndex == segmentIdx)
    {
      const VehicleState::Face &segmentFace = front.segmentIndex == segmentIdx ? front : rear;
      const VehicleState::Face &otherFace = front.segmentIndex != segmentIdx ? front : rear;

      auto helper = [&](bool goForward, size_t nextSegIdx) -> bool
      {
        float remainingDistance = vehicle->length;
        bool found = false;
        bool goNextSegment = goForward;

        while(remainingDistance > 0.0)
        {
          const size_t segmentIdxBefore = nextSegIdx;
          nextSegIdx = getNextSegmentIndex(staticData.trackSegments.at(segmentIdxBefore), goNextSegment, m_stateData);
          if(nextSegIdx == invalidIndex)
            break;

          const TrackSegment &nextSegment = staticData.trackSegments.at(nextSegIdx);
          goNextSegment = nextSegment.nextSegmentIndex[0] == segmentIdxBefore;

          if(otherFace.segmentIndex == nextSegIdx)
          {
            found = true;

            if(goForward)
            {
              if(segmentFace.distance < pos && (segmentFace.distance + vehicle->length) > pos)
              {
                bestVehicle = vehicle;
                bestDistance = 0.0;
                return true;
              }

              float distance = std::min(std::abs(segmentFace.distance - pos), std::abs(segmentFace.distance + vehicle->length - pos));
              if(distance < maxDistance && (!bestVehicle || distance < bestDistance))
              {
                bestVehicle = vehicle;
                bestDistance = distance;
              }
            }
            else
            {
              if(segmentFace.distance > pos && (segmentFace.distance - vehicle->length) < pos)
              {
                bestVehicle = vehicle;
                bestDistance = 0.0;
                return true;
              }

              float distance = std::min(std::abs(segmentFace.distance - pos), std::abs(segmentFace.distance + vehicle->length - pos));
              if(distance < maxDistance && (!bestVehicle || distance < bestDistance))
              {
                bestVehicle = vehicle;
                bestDistance = distance;
              }
            }
            break;
          }

          remainingDistance -= getSegmentLength(nextSegment, m_stateData);
        }

        return found;
      };

      const float segmentLegth = getSegmentLength(staticData.trackSegments.at(segmentIdx), m_stateData);

      if(canGoForward && (segmentFace.distance + vehicle->length) > segmentLegth)
      {
        // Try going forward
        if(helper(true, segmentIdx))
        {
          if(bestVehicle && bestDistance == 0.0)
            return bestVehicle;
          continue;
        }
      }

      if(canGoBackwards && (segmentFace.distance - vehicle->length) < 0)
      {
        // Try going backwards
        if(helper(false, segmentIdx))
        {
          if(bestVehicle && bestDistance == 0.0)
            return bestVehicle;
          continue;
        }
      }
    }
  }

  return bestVehicle;
}

Simulator::Vehicle *Simulator::getVehicleNear(size_t segmentIdx,
                                              float pos, float maxDistance,
                                              bool canGoForward, bool canGoBackwards,
                                              float *outDistance)
{
  if(segmentIdx == invalidIndex)
    return nullptr;

  Vehicle *bestVehicle = nullptr;
  float bestDistance = 0.0;

  bestVehicle = getVehicleNearHelper(segmentIdx, pos, maxDistance, canGoForward, canGoBackwards, bestVehicle, bestDistance);
  if(bestVehicle && bestDistance == 0.0)
  {
    if(outDistance)
      *outDistance = bestDistance;
    return bestVehicle;
  }

  const TrackSegment &segment = staticData.trackSegments.at(segmentIdx);
  const float segLength = getSegmentLength(segment, m_stateData);

  auto helper = [&](bool goForward, size_t nextSegIdx, float originalPos, float remainingPos)
  {
    while(true)
    {
      const size_t segmentIdxBefore = nextSegIdx;
      const TrackSegment &segmentBefore = staticData.trackSegments.at(segmentIdxBefore);
      const float segmentBeforeLength = getSegmentLength(segmentBefore, m_stateData);

      if(goForward && remainingPos < segmentBeforeLength)
        break;
      else if(!goForward && remainingPos >= 0)
        break;

      nextSegIdx = getNextSegmentIndex(segmentBefore, goForward, m_stateData);
      if(nextSegIdx == invalidIndex)
        break;

      if(goForward)
      {
        remainingPos -= segmentBeforeLength;
        originalPos  -= segmentBeforeLength;
      }
      else
      {
        remainingPos = -remainingPos;
        originalPos  = -originalPos;
      }

      const TrackSegment &nextSegment = staticData.trackSegments.at(nextSegIdx);
      goForward = nextSegment.nextSegmentIndex[0] == segmentIdxBefore;

      const float nextSegmentLength = getSegmentLength(nextSegment, m_stateData);

      if(!goForward)
      {
        remainingPos = nextSegmentLength - remainingPos;
        originalPos  = nextSegmentLength - originalPos;
      }

      auto otherVehicle = getVehicleNearHelper(nextSegIdx, originalPos, maxDistance, goForward, !goForward, bestVehicle, bestDistance);
      if(otherVehicle)
      {
        bestVehicle = otherVehicle;
        if(bestDistance == 0.0)
          return;
      }
    }
  };

  if(canGoBackwards && (pos - maxDistance) < 0)
  {
    helper(false, segmentIdx, pos, pos - maxDistance);
    if(bestVehicle && bestDistance == 0.0)
    {
      if(outDistance)
        *outDistance = bestDistance;
      return bestVehicle;
    }
  }

  if(canGoForward && (pos + maxDistance) > segLength)
  {
    helper(true, segmentIdx, pos, pos + maxDistance);
    if(bestVehicle && bestDistance == 0.0)
    {
      if(outDistance)
        *outDistance = bestDistance;
      return bestVehicle;
    }
  }
  if(bestVehicle && outDistance)
    *outDistance = bestDistance;
  return bestVehicle;
}

bool Simulator::splitTrain(quint32 trainToSplitId, const size_t vehicleIdx, const bool fwd, quint32 &trainIdOut)
{
  auto trainPtr = m_stateData.getTrainById(trainToSplitId);
  if(!trainPtr)
    return false;

  Train &trainToSplit = *trainPtr;

  if(!trainToSplit.state.isManualAndStopped())
    return false; // Train must be Manual and stopped

  if(trainToSplit.vehicles.size() <= 1 || vehicleIdx >= trainToSplit.vehicles.size())
    return false; // Train must have more than 1 vehicle and valid vehicle selected

  if((vehicleIdx == 0 && fwd) || (vehicleIdx == (trainToSplit.vehicles.size() - 1) && !fwd))
    return false; // Cannot split before first or after last vehicle

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    const auto coupleOp = fwd ?
                              SimulatorProtocol::TrainRequestCoupleSplit::SplitFront :
                              SimulatorProtocol::TrainRequestCoupleSplit::SplitRear;
    boost::asio::post(
      m_networkContext, [this, trainId=trainToSplit.trainId, vehicleIdx, coupleOp]()
      {
        send(SimulatorProtocol::TrainRequestCoupleSplit(trainId, vehicleIdx, coupleOp));
      });
    return true;
  }

  std::unique_ptr<Train> remainingTrain(new Train);

  // Put a name
  remainingTrain->name = trainToSplit.name + '_';
  remainingTrain->protocol = trainToSplit.protocol;
  remainingTrain->address = trainToSplit.address; // TODO: unique
  remainingTrain->speedMax = trainToSplit.speedMax;
  remainingTrain->state.mode = TrainState::Mode::Manual;
  remainingTrain->state.reverse = trainToSplit.state.reverse;
  remainingTrain->trainId = m_stateData.mLastTrainId++;

  if(fwd)
  {
    remainingTrain->vehicles.reserve(vehicleIdx);

    for(size_t i = 0; i < vehicleIdx; i++)
    {
      remainingTrain->vehicles.push_back(trainToSplit.vehicles.at(i));

      Vehicle *vehicle = m_stateData.getVehicleById(remainingTrain->vehicles.back().vehicleId);
      vehicle->activeTrainId = remainingTrain->trainId;
    }

    trainToSplit.vehicles.erase(trainToSplit.vehicles.begin(), trainToSplit.vehicles.begin() + vehicleIdx);
  }
  else
  {
    remainingTrain->vehicles.reserve(trainToSplit.vehicles.size() - (vehicleIdx + 1));

    for(size_t i = vehicleIdx + 1; i < trainToSplit.vehicles.size(); i++)
    {
      remainingTrain->vehicles.push_back(trainToSplit.vehicles.at(i));

      Vehicle *vehicle = m_stateData.getVehicleById(remainingTrain->vehicles.back().vehicleId);
      vehicle->activeTrainId = remainingTrain->trainId;
    }

    trainToSplit.vehicles.erase(trainToSplit.vehicles.begin() + (vehicleIdx + 1), trainToSplit.vehicles.end());
  }

  trainToSplit.vehicles.shrink_to_fit();

  // Recalculate length
  bool first = true;
  trainToSplit.length = 0.0f;
  for(const auto& item : trainToSplit.vehicles)
  {
    const Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);
    trainToSplit.length += vehicle->length + (first ? 0.0f : staticData.trainCouplingLength);
    first = false;
  }

  if(trainTypeInterface)
  {
    trainTypeInterface->setupTrainSpeedPowered(this, &trainToSplit);
    trainTypeInterface->setupTrainSpeedPowered(this, remainingTrain.get());
  }

  trainIdOut = remainingTrain->trainId;
  auto pair = m_stateData.trains.insert({remainingTrain->trainId, *remainingTrain});

  onTrainAddedRemoved(true, trainIdOut, invalidTrainId);


  if(getSimModeUnlocked() == SimMode::Master)
  {
    sendTrainFullRefreshReplica(SimulatorProtocol::OpCode::TrainChangedVehicles,
                                &trainToSplit);

    sendTrainFullRefreshReplica(SimulatorProtocol::OpCode::TrainAdded,
                                &pair.first->second);
  }

  return true;
}

bool Simulator::coupleTrain(quint32 trainId, bool fwd, quint32& removedTrainIdOut)
{
  auto trainPtr = m_stateData.getTrainById(trainId);
  if(!trainPtr)
    return false;

  Train &train = *trainPtr;
  if(!train.state.isManualAndStopped() || train.vehicles.empty())
    return false; // Train must be Manual and stopped

  if(getSimModeUnlocked() == SimMode::Replica)
  {
    const auto coupleOp = fwd ?
                              SimulatorProtocol::TrainRequestCoupleSplit::CoupleFront :
                              SimulatorProtocol::TrainRequestCoupleSplit::CoupleRear;
    boost::asio::post(
      m_networkContext, [this, trainId=train.trainId, coupleOp]()
      {
        send(SimulatorProtocol::TrainRequestCoupleSplit(trainId, 0, coupleOp));
      });
    return true;
  }

  const Train::VehicleItem itemToCouple = fwd ? train.vehicles.front() : train.vehicles.back(); // Deep copy, vehicles could realloc on resize
  const Vehicle *vehicleToCouple = m_stateData.getVehicleById(itemToCouple.vehicleId);
  const VehicleState::Face &faceToCouple = (itemToCouple.reversed != fwd) ? vehicleToCouple->state.front : vehicleToCouple->state.rear;

  // Look for a vehicle at coupling distance from us
  // Use half coupling distance + small offset to not get ourselves as result
  const float halfCouplingLength = staticData.trainCouplingLength / 2.0f;

  float newDist = faceToCouple.distance;
  if(faceToCouple.segmentDirectionInverted != fwd)
    newDist += (halfCouplingLength + 0.01);
  else
    newDist += (-halfCouplingLength - 0.01);

  Simulator::Vehicle *otherVehicle = getVehicleNear(faceToCouple.segmentIndex, newDist, halfCouplingLength);
  if(!otherVehicle || otherVehicle->activeTrainId == train.trainId)
    return false;

  Train &trainToRemove = m_stateData.trains.at(otherVehicle->activeTrainId);
  if(!trainToRemove.state.isManualAndStopped())
    return false;

  bool goForward = faceToCouple.segmentDirectionInverted != fwd;
  std::vector<Train::VehicleItem> vehiclesToAdd;
  //if(trainToRemove)
  {
    std::vector<Train::VehicleItem> &sourceVehicles = trainToRemove.vehicles;
    size_t segmentIdx = faceToCouple.segmentIndex;
    bool found = false;
    bool flipList = false;

    for(int i = 0; i < 1000; i++)
    {
      const Train::VehicleItem &otherHead = sourceVehicles.front();
      const Train::VehicleItem &otherTail = sourceVehicles.back();

      const Vehicle *otherHeadVehicle = m_stateData.getVehicleById(otherHead.vehicleId);
      const Vehicle *otherTailVehicle = m_stateData.getVehicleById(otherTail.vehicleId);

      bool hasHead = false;
      bool hasTail = false;

      if(segmentIdx == otherHeadVehicle->state.front.segmentIndex
          || segmentIdx == otherHeadVehicle->state.rear.segmentIndex)
      {
        hasHead = true;
      }
      if(segmentIdx == otherTailVehicle->state.front.segmentIndex
          || segmentIdx == otherTailVehicle->state.rear.segmentIndex)
      {
        hasTail = true;
      }

      if(hasHead && hasTail)
      {
        // We must check which one is the nearest
        if(sourceVehicles.size() == 1)
        {
          // Single vehicle, check which face is nearest
          if(segmentIdx == otherHeadVehicle->state.front.segmentIndex
              && segmentIdx == otherHeadVehicle->state.rear.segmentIndex)
          {
            flipList = (otherHeadVehicle->state.rear.distance < otherHeadVehicle->state.front.distance);

            if(!goForward)
              flipList = !flipList;
          }
          else
          {
            // If we find tail first, flip the list
            flipList = (segmentIdx == otherHeadVehicle->state.rear.segmentIndex);
          }

          if(otherHead.reversed)
            flipList = !flipList;
        }
        else
        {
          // Multiple vehicles, check if head or tail is nearest
          const float frontDist = std::min(otherHeadVehicle->state.front.distance,
                                           otherHeadVehicle->state.rear.distance);
          const float backDist  = std::min(otherTailVehicle->state.front.distance,
                                          otherTailVehicle->state.rear.distance);
          flipList = (backDist < frontDist);

          if(!goForward)
            flipList = !flipList;
        }

        found = true;
        break;
      }
      else if(hasHead || hasTail)
      {
        // If we find tail first, flip the list
        flipList = hasTail;
        found = true;
        break;
      }

      const auto &segment = staticData.trackSegments.at(segmentIdx);
      size_t nextSegmentIdx = getNextSegmentIndex(segment, goForward, m_stateData);
      if(segmentIdx == invalidIndex)
        break;

      const auto &nextSegment = staticData.trackSegments.at(nextSegmentIdx);
      goForward = nextSegment.nextSegmentIndex[0] == segmentIdx;
      segmentIdx = nextSegmentIdx;
    }

    if(!found)
      return false;

    if(fwd)
      flipList = !flipList;

    vehiclesToAdd = std::move(sourceVehicles);
    for(Train::VehicleItem &item : vehiclesToAdd)
    {
      Vehicle *vehicle = m_stateData.getVehicleById(item.vehicleId);
      vehicle->activeTrainId = train.trainId;
      if(flipList)
      {
        // When flipping list, also flip each vehicle and flit its segment orientation
        item.reversed = !item.reversed;
        vehicle->state.front.segmentDirectionInverted = !vehicle->state.front.segmentDirectionInverted;
        vehicle->state.rear.segmentDirectionInverted = !vehicle->state.rear.segmentDirectionInverted;
      }
    }

    if(flipList)
      std::reverse(vehiclesToAdd.begin(), vehiclesToAdd.end());

    train.length += trainToRemove.length;
    trainToRemove.length = 0.0f;
  }
  // else
  // {
  //   // TODO: remove?
  //   size_t segmentIdx = faceToCouple.segmentIndex;
  //   bool found = false;
  //   bool flipList = false;

  //   for(int i = 0; i < 1000; i++)
  //   {
  //     // Single vehicle, check which face is nearest
  //     if(segmentIdx == otherVehicle->state.front.segmentIndex
  //         && segmentIdx == otherVehicle->state.rear.segmentIndex)
  //     {
  //       flipList = (otherVehicle->state.rear.distance < otherVehicle->state.front.distance);

  //       if(!goForward)
  //         flipList = !flipList;
  //     }
  //     else if(segmentIdx == otherVehicle->state.front.segmentIndex
  //             || segmentIdx == otherVehicle->state.rear.segmentIndex)
  //     {
  //       // If we find tail first, flip the list
  //       flipList = (segmentIdx == otherVehicle->state.rear.segmentIndex);
  //     }

  //     const auto &segment = staticData.trackSegments.at(segmentIdx);
  //     size_t nextSegmentIdx = getNextSegmentIndex(segment, goForward, m_stateData);
  //     if(segmentIdx == invalidIndex)
  //       break;

  //     const auto &nextSegment = staticData.trackSegments.at(nextSegmentIdx);
  //     goForward = nextSegment.nextSegmentIndex[0] == segmentIdx;
  //     segmentIdx = nextSegmentIdx;
  //   }

  //   if(!found)
  //     return false;

  //   if(fwd)
  //     flipList = !flipList;

  //   if(flipList)
  //   {
  //     otherVehicle->state.front.segmentDirectionInverted = !otherVehicle->state.front.segmentDirectionInverted;
  //     otherVehicle->state.rear.segmentDirectionInverted = !otherVehicle->state.rear.segmentDirectionInverted;
  //   }

  //   otherVehicle->activeTrainId = train.trainId;
  //   vehiclesToAdd.push_back({otherVehicle->vehicleId, flipList});
  //   train.length += otherVehicle->length;
  // }

  train.length += staticData.trainCouplingLength;

  if(fwd)
    train.vehicles.insert(train.vehicles.begin(), vehiclesToAdd.begin(), vehiclesToAdd.end());
  else
    train.vehicles.insert(train.vehicles.end(), vehiclesToAdd.begin(), vehiclesToAdd.end());

  if(getSimModeUnlocked() == SimMode::Master)
  {
    sendTrainFullRefreshReplica(SimulatorProtocol::OpCode::TrainChangedVehicles,
                                &train);
  }

  removedTrainIdOut = trainToRemove.trainId;
  removeTrain(trainToRemove.trainId, false, train.trainId);

  if(trainTypeInterface)
  {
    trainTypeInterface->setupTrainSpeedPowered(this, &train);
  }

  return true;
}

void Simulator::sendMasterFollowTrain(quint32 trainId)
{
  if(getSimMode() != SimMode::Replica)
    return;

  boost::asio::post(
    m_networkContext, [this, trainId]()
    {
      auto msg = SimulatorProtocol::SetMasterFollowTrain(trainId);
      send(msg);
    });
}

TrainTypeInterface::~TrainTypeInterface()
{

}

void Simulator::setSimMode(SimMode m)
{
  boost::asio::post(
      m_networkContext, [this, m]()
      {
        if(mNetworkSimMode == m)
          return;

        const SimMode oldMode = mNetworkSimMode;
        mNetworkSimMode = m;

        if(oldMode != SimMode::Master)
        {
          m_syncReplicaStateTimer.cancel();
          m_syncReplicaTrainFullStateTimer.cancel();

          while(!m_repliacas.empty())
          {
            auto connection =  m_repliacas.back();
            m_repliacas.pop_back();
            connection->stop();
          }
        }

        if(oldMode == SimMode::Replica)
        {
          m_replicaDiscoverSendTimer.cancel();
        }
        else
        {
          m_syncSensorStateTimer.cancel();
        }

        if(mNetworkSimMode == SimMode::Master)
        {
          syncReplicaState();
          syncReplicaTrainFull();
        }
        else if(mNetworkSimMode == SimMode::Replica)
        {
          m_stateData.powerOn = false; // Wait Master

          // Stop UDP discovery
          boost::system::error_code ec;
          m_socketUDP.close(ec);

          // Restart UDP to send discover message
          // TODO: make unified
          m_socketUDP.open(boost::asio::ip::udp::v4(), ec);
          if(ec)
            assert(false);

          m_socketUDP.set_option(boost::asio::socket_base::reuse_address(true), ec);
          if(ec)
            assert(false);

          m_socketUDP.set_option(boost::asio::socket_base::broadcast(true), ec);
          if(ec)
            assert(false);

          m_socketUDP.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), defaultPort), ec);
          if(ec)
            assert(false);

          broadcastAddresses.clear();
          doSendReplicaDiscover();
          doReceiveReplicaDiscover();
        }

        if(mNetworkSimMode != SimMode::Replica)
        {
          syncSensorState();
        }

        boost::asio::post(
            m_coreContext, [this, m=mNetworkSimMode]()
            {
              std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
              mCoreSimMode = m;
            });
      });
}

Simulator::SimMode Simulator::getSimMode() const
{
  std::lock_guard<std::recursive_mutex> lock(m_stateMutex);
  return mCoreSimMode;
}
