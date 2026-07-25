#ifndef VEHICLESMODEL_H
#define VEHICLESMODEL_H

#include <QAbstractListModel>
#include <QPixmap>
#include <QVector>

#include <nlohmann/json.hpp>

class VehiclesModel : public QAbstractListModel
{
  Q_OBJECT

public:
  struct VehicleType
  {
    QString name;
    QPixmap mPixmap;
    float length = 0.0f;
    float maxSpeedKmh = 100.0f;
    bool isPowered = false;
  };

  static constexpr float LengthAdjust = 2.0;
  static constexpr size_t invalidIndex = std::numeric_limits<size_t>::max();

  explicit VehiclesModel(QObject *parent = nullptr);

  // Basic functionality:
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;

  QVariant data(const QModelIndex &idx, int role = Qt::DisplayRole) const override;

  inline VehicleType getTypeAt(size_t idx) const
  {
    return mVehicleTypes.value(idx, {});
  }

  size_t getTypeIdxByName(const QString& name) const;

  void loadVehicles(const QString& baseFile, const nlohmann::json &vehicles);

private:
  QVector<VehicleType> mVehicleTypes;
};

#endif // VEHICLESMODEL_H
