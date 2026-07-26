#ifndef TRAINTYPELISTMODEL_H
#define TRAINTYPELISTMODEL_H

#include "trainvehiclelistmodel.h"

class TrainTypeListModel : public QAbstractListModel
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

private:
  VehicleTypesModel *vehicleTypesModel = nullptr;
  QVector<Train> mTrains;
};

#endif // TRAINTYPELISTMODEL_H
