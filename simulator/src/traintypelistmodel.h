#ifndef TRAINTYPELISTMODEL_H
#define TRAINTYPELISTMODEL_H

#include "trainvehiclelistmodel.h"
#include <traintastic/simulator/simulator.hpp>

class TrainsModel;

class TrainTypeListModel : public QAbstractListModel, public TrainTypeInterface
{
  Q_OBJECT

public:
  static constexpr size_t invalidIndex = std::numeric_limits<size_t>::max();

  struct Train
  {
    QVector<TrainVehicleListModel::Vehicle> vehicles;
    QString name;
    size_t locomotiveReversible = 0; // 0 to 2
    bool reversible = false;
  };

  TrainTypeListModel(VehicleTypesModel *m, TrainsModel *trains, QObject *parent = nullptr);

  // Basic functionality:
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;

  QVariant data(const QModelIndex &idx, int role = Qt::DisplayRole) const override;

  inline Train getTrainAt(size_t idx) const
  {
    return mTrains.value(idx, {});
  }

  size_t getTrainIdxByName(const QString& name) const;

  void loadTrainTypes(const nlohmann::json &trains);

  // TrainTypeInterface
  std::vector<size_t> convertTypeList(const nlohmann::json &trains) override;

  size_t getMaxTypeIdx() override
  {
    return mTrains.isEmpty() ? 0 : mTrains.size() - 1;
  }

  size_t getRandomTrainType(const std::vector<size_t>& allowList,
                            const std::vector<size_t>& blackList) override;

  std::vector<Simulator::Train::VehicleItemFull> createTrainOfType(size_t typeIdx,
                                                                   bool &canInvert, size_t &invertLoco) override;

  void setupTrainSpeedPowered(Simulator *simulator, Simulator::Train *train) override;

  void fillVehicleProperties(Simulator::Vehicle *vehicle) override;

  void onAddTrainReply(quint32 trainId, quint32 token, bool isError) override;

  void getBroadcastAddresses(std::vector<boost::asio::ip::address> &broadcastAddresses) const override;

private:
  VehicleTypesModel *vehicleTypesModel = nullptr;
  TrainsModel *mTrainsModel = nullptr;
  QVector<Train> mTrains;
};

#endif // TRAINTYPELISTMODEL_H
