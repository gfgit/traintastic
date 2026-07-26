#include "traintypelistmodel.h"

TrainTypeListModel::TrainTypeListModel(VehicleTypesModel *m, QObject *parent)
  : QAbstractListModel(parent)
  , vehicleTypesModel(m)
{
}

int TrainTypeListModel::rowCount(const QModelIndex &p) const
{
  return p.isValid() ? 0 : mTrains.size();
}

QVariant TrainTypeListModel::data(const QModelIndex &idx, int role) const
{
  if (!idx.isValid() || idx.row() >= mTrains.size() || idx.column() > 0)
    return QVariant();

  const Train &train = mTrains.at(idx.row());

  if(role == Qt::DisplayRole)
  {
    return train.name;
  }
  else if (role == Qt::ToolTipRole)
  {
    return tr("Train: <b>%1</b><br>"
              "Num vehicles: %2<br>"
              "Reversible: %3")
        .arg(train.name).arg(train.vehicles.size())
        .arg(train.reversible ? tr("yes") : tr("no"));
  }

  return QVariant();
}

size_t TrainTypeListModel::getTrainIdxByName(const QString &name) const
{
  for(size_t i = 0; i < size_t(mTrains.size()); i++)
  {
    if(mTrains.at(i).name == name)
      return i;
  }
  return invalidIndex;
}

void TrainTypeListModel::loadTrainTypes(const nlohmann::json &trains)
{
  beginResetModel();

  mTrains.clear();

  if(!trains.is_array())
  {
    mTrains.squeeze();
    endResetModel();
    return;
  }

  mTrains.reserve(trains.size());

  for(const auto& train : trains)
  {
    if(!train.is_object())
    {
      continue;
    }

    Train item;
    item.name = QString::fromStdString(train.value<std::string>("name", {}));
    if(item.name.isEmpty() || getTrainIdxByName(item.name) != invalidIndex)
      continue;

    item.reversible = train.value<bool>("reversible", item.reversible);
    item.locomotiveReversible = train.value<size_t>("reversible", item.locomotiveReversible);
    if(item.locomotiveReversible > 2)
      item.locomotiveReversible = 2;

    if(auto vehicles = train.find("vehicles"); vehicles != train.end() && vehicles->is_array())
    {
      for(const auto& object : *vehicles)
      {
        if(!object.is_object())
        {
          continue;
        }

        TrainVehicleListModel::Vehicle vehicle;
        auto name = QString::fromStdString(object.value<std::string>("name", {}));
        if(name.isEmpty())
          continue;

        vehicle.vehicleTypeIndex = vehicleTypesModel->getTypeIdxByName(name);
        if(vehicle.vehicleTypeIndex == VehicleTypesModel::invalidIndex)
          continue;

        vehicle.reversed = object.value<bool>("reversed", vehicle.reversed);
        item.vehicles.append(vehicle);
      }
    }

    if(item.vehicles.isEmpty())
      continue;

    mTrains.append(item);
  }

  mTrains.squeeze();

  endResetModel();
}
