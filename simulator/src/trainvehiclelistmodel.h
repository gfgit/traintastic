#ifndef TRAINVEHICLELISTMODEL_H
#define TRAINVEHICLELISTMODEL_H

#include <QAbstractTableModel>

#include "vehicletypesmodel.h"

class TrainVehicleListModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  enum Column
  {
    VehicleName = 0,
    Reverse,
    NCols
  };

  struct Vehicle
  {
    size_t vehicleTypeIndex = VehicleTypesModel::invalidIndex;
    bool reversed = false;
  };

  TrainVehicleListModel(VehicleTypesModel *vehiclesModel, QObject *parent = nullptr);

  // Header:
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  // Basic functionality:
  int rowCount(const QModelIndex &p = QModelIndex()) const override;
  int columnCount(const QModelIndex &p = QModelIndex()) const override;

  QVariant data(const QModelIndex &idx, int role = Qt::DisplayRole) const override;

  // Editable:
  bool setData(const QModelIndex &idx, const QVariant &value,
               int role = Qt::EditRole) override;

  Qt::ItemFlags flags(const QModelIndex& idx) const override;

  void setVehicleAt(int row, size_t vehicleTypeIdx);

  void addVehicle(int row);
  void removeVehicle(int row);

  inline size_t getVehicleTypeAt(int row) const
  {
    return mVehicles.value(row).vehicleTypeIndex;
  }

  QVector<Vehicle> vehicles() const;

  void setVehicles(const QVector<Vehicle> &newVehicles);

private:
  QVector<Vehicle> mVehicles;

  VehicleTypesModel *mVehiclesModel = nullptr;
};

#endif // TRAINVEHICLELISTMODEL_H
