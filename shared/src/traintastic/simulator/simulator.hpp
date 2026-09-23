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

#ifndef TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_SIMULATOR_HPP
#define TRAINTASTIC_SHARED_TRAINTASTIC_SIMULATOR_SIMULATOR_HPP

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <thread>
#include <utility>
#include <vector>
#include <cstdint>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/signals2/signal.hpp>
#include <nlohmann/json.hpp>
#include <traintastic/enum/color.hpp>
#include <traintastic/enum/decoderprotocol.hpp>
#include <traintastic/utils/stringhash.hpp>
#include <traintastic/utils/stringequal.hpp>

#include <traintastic/simulator/protocol.hpp> // TODO: forward declare

#include <QtGlobal>

class QByteArray;

namespace SimulatorProtocol {
enum class OpCode : uint8_t;
struct Message;
}

class SimulatorConnection;
class TrainTypeInterface;

class Simulator : public std::enable_shared_from_this<Simulator>
{
public:
  static constexpr auto invalidIndex = std::numeric_limits<size_t>::max();
  static constexpr uint16_t defaultChannel = 0;
  static constexpr auto invalidAddress = std::numeric_limits<uint16_t>::max();
  static constexpr auto invalidTrainId = std::numeric_limits<quint32>::max();

  struct Point
  {
    float x = 0.0;
    float y = 0.0;

    bool isFinite() const
    {
      return std::isfinite(x) && std::isfinite(y);
    }
  };
  static constexpr Point invalidPoint{std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN()};

  struct TrackSegment
  {
    static constexpr size_t connectorCount = 4;

    enum class Type
    {
      Straight,
      Curve,
      Turnout,
      TurnoutCurved,
      Turnout3Way,
      SingleSlipTurnout,
      DoubleSlipTurnout,
    } type;

    std::string m_id;
    std::array<size_t, connectorCount> nextSegmentIndex{{invalidIndex, invalidIndex, invalidIndex, invalidIndex}};
    std::array<Point, connectorCount> points{{invalidPoint, invalidPoint, invalidPoint, invalidPoint}};
    float rotation;

    struct Straight
    {
      float length = 0.0f;
    } straight, straight2; // TODO: make array

    struct Curve
    {
      Point center = invalidPoint;
      float radius = 0.0f;
      float angle = 0.0f;
      float length = 0.0f;
    };
    std::array<Curve, 2> curves;

    struct Sensor
    {
      size_t index = invalidIndex;
      uint16_t channel = 0;
      uint16_t address = invalidAddress;
    } sensor;

    struct Turnout
    {
      size_t index = invalidIndex;
      uint16_t channel = 0;
      std::array<uint16_t, 2> addresses{{invalidAddress, invalidAddress}};
    } turnout;

    struct Object
    {
      enum class AllowedDirections
      {
        Both = 0,
        Forward = 1,
        Backwards = 2
      };

      enum class Type
      {
        PositionSensor = 0,
        MainSignal = 1,
        ReverseDirection = 2,
        RemoveTrain = 3,
        SpawnTrain = 4,
        AuxSignal = 5,
        StationStopPoint = 6
      };

      Point pos;
      float rotation = 0.0f;
      float position = 0.0f;
      float lateralDiff = 3.0f;

      size_t sensorIndex = invalidIndex;
      AllowedDirections allowedDirection = AllowedDirections::Both;
      bool dirForward = true;
      Type type = Type::PositionSensor;
      std::string_view signalName;
    };
    std::vector<Object> objects;

    Point origin() const
    {
      return points[0];
    }

    bool hasSensor() const
    {
      return sensor.index != invalidIndex;
    }
  };

  struct Sensor
  {
    uint16_t channel = defaultChannel;
    uint16_t address = invalidAddress;

    enum class Type
    {
      TrackCircuit = 0,
      PositionSensor,
      AxleCounter
    } type = Type::TrackCircuit;
  };

  struct Spawn
  {
    std::vector<size_t> allowList, blackList;

    uint16_t address = invalidAddress;
    size_t segmentIndex = invalidIndex;
    size_t maxWagons = 20;
    float wagonLength = 20.0f;
    float defaultSpeedKmH = 0.0f;
    float posInSegment = 0.0f;
    bool forward = true;
  };

  struct Misc
  {
    enum class Type
    {
      Rectangle,
    } type;

    Point origin;
    float rotation = 0.0f;
    float height = 0.0f;
    float width = 0.0f;
    Color color;
  };

  struct ImageRef
  {
    std::string fileName;
    Point origin;
    float rotation = 0.0f;
    float ratio = 0.0f;
    float opacity = 1.0f;
  };

  struct StaticData
  {
    struct View
    {
      float top = std::numeric_limits<float>::infinity();
      float left = std::numeric_limits<float>::infinity();
      float bottom = -std::numeric_limits<float>::infinity();
      float right = -std::numeric_limits<float>::infinity();

      float height() const
      {
        return bottom - top;
      }

      float width() const
      {
        return right - left;
      }
    } view;
    std::vector<TrackSegment> trackSegments;
    std::vector<Sensor> sensors;
    std::vector<Misc> misc;

    std::vector<ImageRef> images;
    std::unordered_map<std::string, size_t> trackSegmentId;

    // Channel, (Address, sensor index)
    std::unordered_map<size_t, std::map<size_t, size_t>> sensorChannelMap;

    std::unordered_map<size_t, Spawn> spawns;

    Simulator::Point defaultCenterPos = {0, 0};
    float defaultViewScale = 1.0f;
    float trainWidth = 10.0f;
    float trainCouplingLength = 4.0f;
    float worldScale = 1.0f;
    float defaultSignalScale = 1.0f;
  };

  struct SensorState
  {
    union {
      uint32_t occupied = 0;
      int32_t axleCount;
    };

    int8_t maxTime = 0;
    int8_t curTime = 0;

    enum Encoding : uint8_t
    {
      CodeAbsent = 0,
      Code75 = 1,
      Code120 = 2,
      Code180 = 3,
      Code270 = 4
    };
    Encoding encoding = Encoding::CodeAbsent;
    bool value = false;
  };

  struct Train;

  struct TurnoutState
  {
    enum class State : uint8_t
    {
      Unknown = 0,
      Closed = 1,
      Thrown = 2,
      ThrownLeft = 3,
      ThrownRight = 4,
      ClosedLeft = 5,
      ClosedRight = 6
    };

    State state = State::Closed;
    uint8_t coils = 0;

    std::vector<quint32> watchingTrainIDs;
  };

  struct MainSignal
  {
    // NOTE: keep in sync with protocol.hpp
    enum class State
    {
      Off = 0,
      On,
      BlikOn,
      BlinkReverseOn
    };

    enum class RappelState
    {
      Off = 0,
      OneLine_60,
      TwoLines_100
    };

    static constexpr uint8_t StateMask = uint8_t(State::BlinkReverseOn);
    static constexpr uint8_t ArrowLight = 0x04;

    uint16_t channel = defaultChannel;
    uint16_t address = invalidAddress;
    std::string name;
    size_t ownerConnectionId = invalidIndex;

    double maxSpeed = 0;

    uint8_t advanceSignalStateAndArrow = 0;
    // Signal blink phase
    uint8_t blinkStart = 0;

    char directionIndicatorText = ' ';

    bool square = false;
    bool hasSquareArrowLight = false;
    bool hasAdvanceSignal = false;
    bool hasDirectionIndicator = false;
    bool hasRappel = false;
    bool isPureDistantSignal = false;

    // Increase lateral diff with signal scale factor
    // This prevents 2 parallel signals to overlap each other
    // Set to false if signal is in-between 2 tracks as it would move it.
    bool zoomLateralDiff = true;

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

    inline uint8_t getSignalBlinkStart(bool startSignal) const
    {
      if(startSignal)
        return (blinkStart >> 4) & 0x0F;
      return blinkStart & 0x0F;
    }

    inline void setSignalBlinkStart(bool startSignal, uint8_t state)
    {
      if(startSignal)
        blinkStart = (advanceSignalStateAndArrow & 0x0F) | (state << 4);
      else
        blinkStart = (advanceSignalStateAndArrow & 0xF0) | state;
    }

    enum class FixedLimit
    {
      NoLimit = 0,
      Limit30,
      Limit60
    } fixedLimit = FixedLimit::NoLimit;

    RappelState rappelState = RappelState::Off;

    struct Light
    {
      enum class Color
      {
        Red = 0,
        Yellow,
        Green
      } color = Color::Red;

      State state = State::Off;
    };

    std::vector<Light> lights;
  };

  struct AuxSignal
  {
    enum class SubType
    {
      LightDwarfSignal = 0,
      RotatingDwarfSignal,
      HighDepartureIndicator
    };

    uint16_t channel = defaultChannel;
    uint16_t address = invalidAddress;
    std::string name;
    size_t ownerConnectionId = invalidIndex;

    uint8_t mLights = 0;
    uint8_t mPosition = 0;

    // Increase lateral diff with signal scale factor
    // This prevents 2 parallel signals to overlap each other
    // Set to false if signal is in-between 2 tracks as it would move it.
    bool zoomLateralDiff = true;
    SubType subType = SubType::LightDwarfSignal;

    inline void setLightOn(uint8_t n, bool on)
    {
      assert(n >= 0 && n < 8);
      if(on)
        mLights |= (1u << n);
      else
        mLights &= ~(1u << n);
    }

    inline bool isLightOn(uint8_t n) const
    {
      assert(n >= 0 && n < 8);
      return mLights & (1u << n);
    }
  };

  struct SpawnState
  {
    size_t ownerConnectionId = invalidIndex;

    static constexpr size_t START_WAITING_RESET = 60; // Ticks, 2 seconds
    static constexpr size_t MAX_WAITING_RESET = START_WAITING_RESET + 1;
    size_t waitingResetCount = 0;

    enum class State
    {
      Inactive = 0,
      Ready = 1,
      WaitingReset = 2
    };

    State state = State::Inactive;
  };

  struct TrainState
  {
    float speed = 0.0f;
    float targetSpeed = 0.0f;
    float waitToStart_ms = 0;
    bool reverse = false;
    bool speedOrDirectionChanged = false;
    bool skipCollisionDetection = false;

    // TODO: make adjustable
    static constexpr int SecondsWaitFromStandstill = 8;
    static constexpr float AccelerationRate_ms2 = 0.6;
    static constexpr float DecelerationRate_ms2 = 0.75;

    using Mode = SimulatorProtocol::TrainMode;

    Mode mode = Mode::SemiAutomatic;

    struct SignalCache
    {
      MainSignal *signal = nullptr;
      std::vector<size_t> turnouts;
      size_t segmentIdx = invalidIndex;
      float distance = 0.0f;
      float posInSegment = 0.0f;
      bool dirty = false;
    };

    SignalCache nextSignal, prevSignal;
    bool isOnStationStop = false;

    inline bool isManualAndStopped() const
    {
      if(mode != Mode::Manual)
        return false;

      return speed == 0.0f && targetSpeed == 0.0f;
    }
  };

  struct VehicleState
  {
    struct Face
    {
      Point position = invalidPoint;
      size_t segmentIndex = invalidIndex;
      bool segmentDirectionInverted = false;
      float distance = 0.0f;
    };

    Face front;
    Face rear;
  };

  struct Vehicle
  {
    VehicleState state;
    quint32 vehicleId = invalidTrainId;
    float length = 0.0f;
    size_t typeIdx = invalidIndex;
    quint32 activeTrainId = invalidTrainId;
    Color color = Color::Blue;
  };

  struct Train
  {
    quint32 trainId = invalidTrainId;
    std::string name;

    struct VehicleItem
    {
      quint32 vehicleId = invalidTrainId;
      bool reversed = false;
    };

    struct VehicleItemFull
    {
      Vehicle vehicle;
      bool reversed = false;
    };

    std::vector<VehicleItem> vehicles;
    float length = 0.0f;
    float speedMax = 0.0f;
    bool isPowered = false;
    DecoderProtocol protocol = DecoderProtocol::None;
    uint16_t address = invalidAddress;
    TrainState state;

    ~Train()
    {
    }

    inline bool isHeadOrTail(const Vehicle *vehicle) const
    {
      if(vehicles.empty())
        return false;
      return vehicle->vehicleId == vehicles.front().vehicleId || vehicle->vehicleId == vehicles.back().vehicleId;
    }

    inline VehicleItem getHead() const
    {
      return vehicles.empty() ? VehicleItem{} : vehicles.front();
    }

    inline VehicleItem getTail() const
    {
      return vehicles.empty() ? VehicleItem{} : vehicles.back();
    }
  };

  struct SegmentVehicles
  {
    std::unordered_set<quint32> vehicleIDs;
  };

  struct StateData
  {
    float tickActive = 0.0f;
    float tickLoad = 0.0f;
    float trainSpeedFactor = 1.0f;

    // TODO move to private member of Simulator
    quint32 mLastTrainId = 0;
    quint32 mLastVehicleId = 0;

    bool powerOn = false;

    uint8_t signalBlinkState = 0; // Goes from 0 to 7 TODO: remove

    std::vector<SensorState> sensors;
    std::vector<TurnoutState> turnouts;

    std::map<quint32, Train> trains; // Avoid rehashing
    std::map<quint32, Vehicle> vehicles;

    // TODO: split state
    std::unordered_map<std::string, MainSignal, StringHash, StringEqual> mainSignals;
    std::unordered_map<std::string, AuxSignal, StringHash, StringEqual> auxSignals;

    std::unordered_map<size_t, SpawnState> spawns;
    std::unordered_map<size_t, SegmentVehicles> segmentVehicles;

    inline const Simulator::Train *getTrainById(quint32 trainId) const
    {
      if(trainId == invalidTrainId)
        return nullptr;

      auto it = trains.find(trainId);
      if(it == trains.end())
        return nullptr;

      return &it->second;
    }

    inline Simulator::Train *getTrainById(quint32 trainId)
    {
      if(trainId == invalidTrainId)
        return nullptr;

      auto it = trains.find(trainId);
      if(it == trains.end())
        return nullptr;

      return &it->second;
    }

    inline Simulator::Vehicle *getVehicleById(quint32 vehicleId)
    {
      if(vehicleId == invalidTrainId)
        return nullptr;

      auto it = vehicles.find(vehicleId);
      if(it == vehicles.end())
        return nullptr;

      return &it->second;
    }

    inline const Simulator::Vehicle *getVehicleById(quint32 vehicleId) const
    {
      if(vehicleId == invalidTrainId)
        return nullptr;

      auto it = vehicles.find(vehicleId);
      if(it == vehicles.end())
        return nullptr;

      return &it->second;
    }
  };

  enum class SimMode
  {
    Standalone = 0,
    Master,
    Replica
  };

private:
  StateData m_stateData;

public:
  const StaticData staticData;
  boost::signals2::signal<void()> onTick;
  boost::signals2::signal<void(bool add, quint32 trainId, quint32 suggestedTrainId)> onTrainAddedRemoved;

  Simulator(const nlohmann::json& world, TrainTypeInterface *iface);
  ~Simulator();

  static void updateView(Simulator::StaticData::View& view, Simulator::Point point);
  static void updateView(Simulator::StaticData::View& view,
                         const Simulator::TrackSegment::Curve& curve, float startAngle);

  const StateData& stateData() const;

  void enableServer(bool localhostOnly = true, uint16_t port = 5741);

  uint16_t serverPort() const;

  void start(bool discoverable = false);
  void stop();

  void setPowerOn(bool powerOn);
  void togglePowerOn();

  static std::pair<Point, Point> getTrainExtents(const StateData &stateData_, const Train &train);

  void setTrainDirection(quint32 trainId, bool reverse)
  {
    if(trainId == invalidTrainId)
      return;
    Train *train = m_stateData.getTrainById(trainId);
    if(!train)
      return;

    setTrainDirection(*train, reverse);
  }

  void setTrainSpeed(quint32 trainId, float speed, bool immediate = false)
  {
    if(trainId == invalidTrainId)
      return;
    Train *train = m_stateData.getTrainById(trainId);
    if(!train)
      return;

    setTrainSpeed(*train, speed, immediate);
  }

  void applyTrainSpeedDelta(quint32 trainId, float delta)
  {
    if(trainId == invalidTrainId)
      return;
    Train *train = m_stateData.getTrainById(trainId);
    if(!train)
      return;

    applyTrainSpeedDelta(*train, delta);
  }

  void setTrainMode(quint32 trainId, TrainState::Mode mode)
  {
    if(trainId == invalidTrainId)
      return;
    Train *train = m_stateData.getTrainById(trainId);
    if(!train)
      return;

    setTrainMode(*train, mode);
  }

  void stopTrainAndSetManual(quint32 trainId)
  {
    if(trainId == invalidTrainId)
      return;
    Train *train = m_stateData.getTrainById(trainId);
    if(!train)
      return;

    stopTrainAndSetManual(*train);
  }

  void stopAllTrains();

  void setTrainSpeedFactor(float val);

  bool segmentOccupied(size_t segmentIdx) const;

  void setTurnoutState(size_t segmentIndex, TurnoutState::State state, bool sendChange = true);
  void toggleTurnoutState(size_t segmentIndex, bool setUnknown);

  void send(const SimulatorProtocol::Message& message);
  void receive(const SimulatorProtocol::Message& message, size_t fromConnId);
  void receiveVariableSize(const SimulatorProtocol::Message& message, size_t fromConnId);

  void setConnectionAsReplica(const std::shared_ptr<SimulatorConnection>& connection);
  void removeConnection(const std::shared_ptr<SimulatorConnection>& connection, bool replica, bool remove = true);

  inline std::recursive_mutex& stateMutex() { return m_stateMutex; }

  enum class TrainPlacement : uint8_t
  {
    PlaceCenter = 0,
    PlaceStart = 1,
    PlaceEnd = 2
  };

  bool addTrain(const std::string_view& name,
                DecoderProtocol proto, uint16_t addr,
                const std::vector<Train::VehicleItemFull> &vehicles, size_t segmentIndex,
                quint32 &trainIdOut, float startPos = -1.0, TrainPlacement placement = TrainPlacement::PlaceCenter);

  bool requestAddTrain(const std::string &name,
                       const std::vector<Train::VehicleItemFull> &vehicles, size_t segmentIndex,
                       quint32 token, float startPos = -1.0, TrainPlacement placement = TrainPlacement::PlaceCenter, bool invertDir = false);

  bool removeTrain(quint32 trainId, bool removeWagons, quint32 suggestedTrainId = invalidTrainId);
  bool removeTrainUser(quint32 trainId);

  Vehicle *addVehicle(float length, Color color,
                      size_t typeIdx = invalidIndex,
                      quint32 suggestId = invalidTrainId);
  bool removeVehicle(Vehicle *vehicle);

  void sendTrainFullRefreshReplica(SimulatorProtocol::OpCode op, Train *train);
  void sendTrainStateRefreshReplica(Train *train);

  Vehicle *getVehicleNear(size_t segmentIdx, float pos, float maxDistance,
                          bool canGoForward = true, bool canGoBackwards = true,
                          float *outDistance = nullptr);

  bool splitTrain(Train &trainToSplit, const size_t vehicleIdx, const bool fwd, quint32 &trainIdOut);
  bool coupleTrain(Train &train, bool fwd, quint32 &removedTrainIdOut);

  void liftRestrictions(bool val)
  {
    for(auto it : m_stateData.mainSignals)
    {
      MainSignal *s = &it.second;
      s->maxSpeed = val ? 150 : 0;
    }
  }

  void setSimMode(SimMode m);
  SimMode getSimMode() const;


  inline SimMode getSimModeUnlocked() const { return mSimMode; };

private:
  friend struct TrainState;
  constexpr static auto tickRate = std::chrono::milliseconds(1000 / 30);
  constexpr static auto handShakeRateSend = std::chrono::milliseconds(3000);
  constexpr static auto handShakeRateRecv = std::chrono::milliseconds(10000);
  constexpr static auto signalBlinkRate = std::chrono::milliseconds(275);
  constexpr static auto syncSensorRate = std::chrono::milliseconds(150);
  constexpr static auto syncReplicaRate = std::chrono::milliseconds(220);
  constexpr static auto syncReplicaFullRate = std::chrono::milliseconds(1500);
  constexpr static auto replicaDiscoverSendRate = std::chrono::milliseconds(1000);
  constexpr static auto garbageCollectRate = std::chrono::milliseconds(500);

  constexpr static float defaultSpeedKmH = 200;
  constexpr static float SpeedKmHtoTick = tickRate.count() / 3600.0;
  constexpr static float defaultSpeedTickRate = defaultSpeedKmH * SpeedKmHtoTick;

  boost::asio::io_context m_ioContext;
  boost::asio::steady_timer m_tickTimer;
  boost::asio::steady_timer m_handShakeSendTimer;
  boost::asio::steady_timer m_handShakeRecvTimer;
  boost::asio::ip::tcp::acceptor m_acceptor;
  boost::asio::ip::udp::socket m_socketUDP;
  std::array<char, 8> m_udpBuffer;
  boost::asio::ip::udp::endpoint m_remoteEndpoint;

  std::vector<boost::asio::ip::address> broadcastAddresses;

  boost::asio::steady_timer m_signalBlinkStateTimer;
  boost::asio::steady_timer m_syncSensorStateTimer;
  boost::asio::steady_timer m_syncReplicaStateTimer;
  boost::asio::steady_timer m_syncReplicaTrainFullStateTimer;
  boost::asio::steady_timer m_garbageCollectTimer;
  boost::asio::steady_timer m_replicaDiscoverSendTimer;

  std::thread m_thread;
  mutable std::recursive_mutex m_stateMutex;
  bool m_serverEnabled = false;
  bool m_serverLocalHostOnly = true;
  static constexpr uint16_t defaultPort = 5741; // UDP Discovery
  uint16_t m_serverPort = 5741;

  size_t lastConnectionId = 0;
  std::list<std::shared_ptr<SimulatorConnection>> m_connections;
  std::list<std::shared_ptr<SimulatorConnection>> m_repliacas;
  size_t mLastSpawnSyncIdx = 0;
  quint32 mLastTrainSyncId = invalidTrainId;
  quint32 mLastTrainSyncFullId = invalidTrainId;

  TrainTypeInterface *trainTypeInterface = nullptr;

  SimMode mSimMode = SimMode::Standalone;

  void accept();
  void doReceive();
  void doSendReplicaDiscover();
  void doReceiveReplicaDiscover();
  void onConnectionRemoved(const std::shared_ptr<SimulatorConnection> &connection);
  void onReplicaRemoved(const std::shared_ptr<SimulatorConnection> &connection);

  void tick();
  void handShakeSend();
  void handShakeRecv();
  void blinkSignals();
  void syncSensorState();
  void syncReplicaState();
  void syncReplicaTrainFull();
  void garbageCollectSegmentState();

  void updateTrainPositions();

  void calculateFacePosition(Simulator::VehicleState::Face &face);
  bool updateVehiclePosition(Vehicle *vehicle, bool frontFace,
                             const float speed, bool isFirst_,
                             Train &train_, bool &trainRemoved, float &outRemaining);

  void updateReplicaTracking(Vehicle *vehicle, const VehicleState &oldState);
  void maybeAddVehicleSegment(Vehicle *vehicle, size_t segmentIdx);
  void maybeRemoveVehicleSegment(Vehicle *vehicle, size_t segmentIdx);

  void updateSensors();
  void updateSpawns();
  void doSpawnTrain(uint16_t spawnAddress);

  bool isStraight(const TrackSegment& segment);
  bool isCurve(const TrackSegment& segment, size_t& curveIndex);

  static StaticData load(const nlohmann::json& world, StateData& stateData, TrainTypeInterface *iface);
  static void loadTrackplan(const nlohmann::json &world,
                            StaticData &data, StateData &stateData, TrainTypeInterface *iface);
  static void loadTrackObjects(const nlohmann::json &track,
                               StaticData &data, StateData &stateData,
                               TrackSegment &segment, TrainTypeInterface *iface);

  void loadTrains(const nlohmann::json& world);

  void updateTrainNextSignal(Train &train, bool next);
  bool checkNextSignal(Train &train);

  Simulator::Vehicle *getVehicleNearHelper(size_t segmentIdx, float pos, float maxDistance,
                                           bool canGoForward, bool canGoBackwards,
                                           Vehicle *bestVehicle, float &bestDistance);

  void destroyTrain(Train &train, bool removeWagons);
  void setTrainDirection(Train &train, bool reverse);
  void setTrainSpeed(Train &train, float speed, bool immediate = false);
  void applyTrainSpeedDelta(Train &train, float delta);
  void setTrainMode(Train &train, TrainState::Mode mode);
  void stopTrainAndSetManual(Train &train);

  void sendReplicas(const SimulatorProtocol::Message& message);
  void sendReplicas(QByteArray &buf);
  void sendToMaster(QByteArray &buf);
};


class TrainTypeInterface
{
public:
  virtual ~TrainTypeInterface();

  virtual std::vector<size_t> convertTypeList(const nlohmann::json &trains) = 0;

  virtual size_t getMaxTypeIdx() = 0;

  virtual size_t getRandomTrainType(const std::vector<size_t>& allowList,
                                    const std::vector<size_t>& blackList) = 0;

  virtual std::vector<Simulator::Train::VehicleItemFull> createTrainOfType(size_t typeIdx,
                                                                           bool &canInvert, size_t &invertLoco) = 0;

  virtual void setupTrainSpeedPowered(Simulator *simulator, Simulator::Train *train) = 0;
  
  virtual void fillVehicleProperties(Simulator::Vehicle *vehicle) = 0;

  virtual void onAddTrainReply(quint32 trainId, quint32 token, bool isError) = 0;

  virtual void getBroadcastAddresses(std::vector<boost::asio::ip::address> &broadcastAddresses) const = 0;
};


constexpr Simulator::Point operator+(const Simulator::Point lhs, const Simulator::Point rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

constexpr Simulator::Point operator-(const Simulator::Point lhs, const Simulator::Point rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

constexpr Simulator::Point operator*(const Simulator::Point factor, const float multiplier)
{
  return {factor.x * multiplier, factor.y * multiplier};
}

constexpr Simulator::Point operator/(const Simulator::Point divided, const float divisor)
{
  return {divided.x / divisor, divided.y / divisor};
}

namespace
{

constexpr auto pi = std::numbers::pi_v<float>;

constexpr float deg2rad(float degrees)
{
  return degrees * static_cast<float>(std::numbers::pi / 180);
}

}

#endif
