#include "vehiclesmodel.h"

#include <QFont>
#include <QFileInfo>
#include <QDir>

VehiclesModel::VehiclesModel(QObject *parent)
  : QAbstractListModel(parent)
{
}

int VehiclesModel::rowCount(const QModelIndex &p) const
{
  return p.isValid() ? 0 : mVehicleTypes.size();
}

QVariant VehiclesModel::data(const QModelIndex &idx, int role) const
{
  if (!idx.isValid() || idx.row() >= mVehicleTypes.size() || idx.column() != 0)
    return QVariant();

  const VehicleType &vehicleType = mVehicleTypes.at(idx.row());

  switch(role)
  {
  case Qt::DisplayRole:
  {
    return vehicleType.name;
  }
  case Qt::DecorationRole:
  {
    return vehicleType.mPixmap;
  }
  case Qt::ToolTipRole:
  {
    return tr("Vehicle: <b>%1</b><br>"
              "Powered: %2<br>"
              "Length: %3 m<br>"
              "Max speed: %4 km/h")
        .arg(vehicleType.name, vehicleType.isPowered ? tr("yes") : tr("no"))
        .arg(vehicleType.length + LengthAdjust).arg(vehicleType.maxSpeedKmh);
  }
  case Qt::FontRole:
  {
    if(vehicleType.isPowered)
    {
      QFont f;
      f.setBold(true);
      return f;
    }
    break;
  }
  default:
    break;
  };

  return QVariant();
}

size_t VehiclesModel::getTypeIdxByName(const QString &name) const
{
  for(size_t i = 0; i < size_t(mVehicleTypes.size()); i++)
  {
    if(mVehicleTypes.at(i).name == name)
      return i;
  }
  return invalidIndex;
}

void VehiclesModel::loadVehicles(const QString &baseFile, const nlohmann::json &vehicles)
{
  beginResetModel();

  mVehicleTypes.clear();

  const QDir dir = QFileInfo(baseFile).absoluteDir();

  if(!vehicles.is_array() || !dir.exists())
  {
    mVehicleTypes.squeeze();
    endResetModel();
    return;
  }

  mVehicleTypes.reserve(vehicles.size());

  for(const auto& object : vehicles)
  {
    if(!object.is_object())
    {
      continue;
    }

    VehicleType item;
    item.name = QString::fromStdString(object.value<std::string>("name", {}));
    if(item.name.isEmpty() || getTypeIdxByName(item.name) != invalidIndex)
      continue;

    item.isPowered = object.value<bool>("powered", item.isPowered);
    item.maxSpeedKmh = object.value<float>("max_speed_kmh", item.maxSpeedKmh);

    const QString imageName = QString::fromStdString(object.value<std::string>("image", {}));
    if(imageName.isEmpty())
      continue;

    if(!item.mPixmap.load(dir.absoluteFilePath(imageName)))
      continue;

    if(item.mPixmap.height() < 10 || item.mPixmap.height() > 40)
      continue;

    item.length = float(item.mPixmap.width()) / 10.0 - LengthAdjust;

    mVehicleTypes.append(item);
  }

  mVehicleTypes.squeeze();

  endResetModel();
}
