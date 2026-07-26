#include "traintypelistmodel.h"

#include <QRandomGenerator>

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

std::vector<size_t> TrainTypeListModel::convertTypeList(const nlohmann::json &trains)
{
  std::vector<size_t> result;
  if(!trains.is_array())
    return result;

  result.reserve(trains.size());

  for(const auto& train : trains)
  {
    if(!train.is_string())
      continue;

    auto name = QString::fromStdString(train.get<std::string>());
    if(name.isEmpty())
      continue;

    size_t typeIdx = getTrainIdxByName(name);
    if(typeIdx == invalidIndex)
      continue;

    result.push_back(typeIdx);
  }

  return result;
}

size_t TrainTypeListModel::getRandomTrainType(const std::vector<size_t> &allowList, const std::vector<size_t> &blackList)
{
  if(!allowList.empty())
  {
    return size_t(QRandomGenerator::global()->bounded(qint64(allowList.size())));
  }

  // All types - black listed types = list from which to chose
  size_t typeIdx = size_t(QRandomGenerator::global()->bounded(qint64(size_t(mTrains.size()) - blackList.size())));
  while(std::find(blackList.begin(), blackList.end(), typeIdx) != blackList.end())
    typeIdx++;

  if(typeIdx >= size_t(mTrains.size()))
    return invalidIndex;

  return typeIdx;
}

std::vector<Simulator::Train::VehicleItem> TrainTypeListModel::createTrainOfType(size_t typeIdx,
                                                                                 Simulator *simulator,
                                                                                 bool &canInvert, size_t &invertLoco)
{
  if(typeIdx == invalidIndex || typeIdx >= size_t(mTrains.size()))
    return {};

  std::vector<Simulator::Train::VehicleItem> result;

  const Train& train = mTrains.at(typeIdx);
  canInvert = train.reversible;
  invertLoco = train.locomotiveReversible;

  for(const auto& vehicle : train.vehicles)
  {
    Simulator::Train::VehicleItem item;
    item.reversed = vehicle.reversed;

    if(vehicle.vehicleTypeIndex == VehicleTypesModel::invalidIndex)
        continue;

    VehicleTypesModel::VehicleType vehicleType = vehicleTypesModel->getTypeAt(vehicle.vehicleTypeIndex);
    if(vehicleType.name.isEmpty())
        continue;

    item.vehicle = simulator->addVehicle(vehicleType.name.toStdString(),
                                         vehicleType.length,
                                         Color::Blue,
                                         vehicle.vehicleTypeIndex);

    result.push_back(item);
  }

  return result;
}

void TrainTypeListModel::setupTrainSpeedPowered(Simulator::Train *train)
{
  bool isPowered = false;
  bool maxSpeedSet = false;
  float maxSpeed = 150.0f;

  for(const Simulator::Train::VehicleItem &item : train->vehicles)
  {
    if(item.vehicle->typeIdx != VehicleTypesModel::invalidIndex)
    {
      const VehicleTypesModel::VehicleType vehicleType = vehicleTypesModel->getTypeAt(item.vehicle->typeIdx);
      if(!vehicleType.name.isEmpty())
      {
        if(vehicleType.isPowered)
          isPowered = true;

        if(!maxSpeedSet || vehicleType.maxSpeedKmh < maxSpeed)
          maxSpeed = vehicleType.maxSpeedKmh;
      }
    }
    else
      isPowered = true; // Custom vehicles are always powered for tests

    train->isPowered = isPowered;
    train->speedMax = maxSpeed;
  }
}
