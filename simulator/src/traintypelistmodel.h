#ifndef TRAINTYPELISTMODEL_H
#define TRAINTYPELISTMODEL_H

#include "trainvehiclelistmodel.h"
#include <traintastic/simulator/simulator.hpp>

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

  TrainTypeListModel(VehicleTypesModel *m, QObject *parent = nullptr);

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

  std::vector<Simulator::Train::VehicleItem> createTrainOfType(size_t typeIdx,
                                                               Simulator *simulator,
                                                               bool &canInvert, size_t &invertLoco) override;

  void setupTrainSpeedPowered(Simulator::Train *train) override;

private:
  VehicleTypesModel *vehicleTypesModel = nullptr;
  QVector<Train> mTrains;
};

#endif // TRAINTYPELISTMODEL_H
