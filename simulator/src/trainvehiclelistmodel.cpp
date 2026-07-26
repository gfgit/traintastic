#include "trainvehiclelistmodel.h"

TrainVehicleListModel::TrainVehicleListModel(VehicleTypesModel *vehiclesModel, QObject *parent)
  : QAbstractTableModel(parent)
  , mVehiclesModel(vehiclesModel)
{
}

QVariant TrainVehicleListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if(orientation == Qt::Horizontal && role == Qt::DisplayRole)
  {
    switch (section)
    {
    case VehicleName:
      return tr("Vehicle");
    case Reverse:
      return tr("Reverse");
    default:
      break;
    }
  }

  return QAbstractTableModel::headerData(section, orientation, role);
}

int TrainVehicleListModel::rowCount(const QModelIndex &p) const
{
  return p.isValid() ? 0 : mVehicles.size();
}

int TrainVehicleListModel::columnCount(const QModelIndex &p) const
{
  return p.isValid() ? 0 : NCols;
}

QVariant TrainVehicleListModel::data(const QModelIndex &idx, int role) const
{
  if (!idx.isValid() || idx.row() >= mVehicles.size() || idx.column() >= NCols)
    return QVariant();

  const Vehicle &vehicle = mVehicles.at(idx.row());

  switch(idx.column())
  {
  case VehicleName:
  {
    if(role == Qt::DisplayRole)
    {
      if(vehicle.vehicleTypeIndex == VehicleTypesModel::invalidIndex)
        return QString();

      return mVehiclesModel->getTypeAt(vehicle.vehicleTypeIndex).name;
    }
    else if(role == Qt::ToolTipRole)
    {
      if(vehicle.vehicleTypeIndex == VehicleTypesModel::invalidIndex)
        return QString();

      return mVehiclesModel->data(mVehiclesModel->index(vehicle.vehicleTypeIndex, 0), Qt::ToolTipRole);
    }
    break;
  }
  case Reverse:
  {
    if(role == Qt::CheckStateRole)
      return vehicle.reversed ? Qt::Checked : Qt::Unchecked;
    break;
  }
  default:
    break;
  }

  return QVariant();
}

bool TrainVehicleListModel::setData(const QModelIndex &idx, const QVariant &value, int role)
{
  if (!idx.isValid() || idx.row() >= mVehicles.size() || idx.column() >= NCols)
    return false;

  Vehicle &vehicle = mVehicles[idx.row()];

  if(idx.column() == Reverse)
  {
    vehicle.reversed = value.value<Qt::CheckState>() == Qt::Checked;
    emit dataChanged(idx, idx, {role});
    return true;
  }

  return false;
}

Qt::ItemFlags TrainVehicleListModel::flags(const QModelIndex &idx) const
{
  if (!idx.isValid() || idx.row() >= mVehicles.size() || idx.column() >= NCols)
    return Qt::NoItemFlags;

  Qt::ItemFlags f = QAbstractItemModel::flags(idx) | Qt::ItemIsEditable;
  if(idx.column() == Reverse)
    f.setFlag(Qt::ItemIsUserCheckable);
  return f;
}

void TrainVehicleListModel::setVehicleAt(int row, size_t vehicleTypeIdx)
{
  if(row < 0 || row >= mVehicles.size())
    return;

  Vehicle &vehicle = mVehicles[row];
  vehicle.vehicleTypeIndex = vehicleTypeIdx;

  emit dataChanged(index(row, 0), index(row, 0));
}

void TrainVehicleListModel::addVehicle(int row)
{
  row = qBound(0, row, mVehicles.size()); // Allow row = size to append

  if(row < 0)
    row = 0;
  if(row >= mVehicles.size())
    row = mVehicles.size();

  beginInsertRows(QModelIndex(), row, row);
  mVehicles.insert(row, {});
  endInsertRows();
}

void TrainVehicleListModel::removeVehicle(int row)
{
  row = qBound(0, row, mVehicles.size() - 1);

  beginRemoveRows(QModelIndex(), row, row);
  mVehicles.removeAt(row);
  endRemoveRows();
}

void TrainVehicleListModel::setVehicles(const QVector<Vehicle> &newVehicles)
{
  beginResetModel();
  mVehicles = newVehicles;
  endResetModel();
}

QVector<TrainVehicleListModel::Vehicle> TrainVehicleListModel::vehicles() const
{
  return mVehicles;
}
