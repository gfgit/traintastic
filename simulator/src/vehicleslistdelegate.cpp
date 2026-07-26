#include "vehicleslistdelegate.h"

#include "trainvehiclelistmodel.h"
#include "vehicletypesmodel.h"

#include <QComboBox>

VehiclesListDelegate::VehiclesListDelegate(VehicleTypesModel *m, QObject *parent)
  : QStyledItemDelegate{parent}
  , mVehiclesModel(m)
{

}

QWidget *VehiclesListDelegate::createEditor(QWidget *parent,
                                            const QStyleOptionViewItem &option,
                                            const QModelIndex &index) const
{
  if(index.column() == TrainVehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = new QComboBox(parent);
    modelCombo->setModel(mVehiclesModel);
    return modelCombo;
  }

  return QStyledItemDelegate::createEditor(parent, option, index);
}

void VehiclesListDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  if(index.column() == TrainVehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = static_cast<QComboBox *>(editor);
    const TrainVehicleListModel *m = static_cast<const TrainVehicleListModel *>(index.model());

    size_t vehicleTypeIdx = m->getVehicleTypeAt(index.row());
    if(vehicleTypeIdx == VehicleTypesModel::invalidIndex)
      modelCombo->setCurrentIndex(-1);
    else
      modelCombo->setCurrentIndex(vehicleTypeIdx);
    return;
  }

  QStyledItemDelegate::setEditorData(editor, index);
}

void VehiclesListDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  if(index.column() == TrainVehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = static_cast<QComboBox *>(editor);
    TrainVehicleListModel *m = static_cast<TrainVehicleListModel *>(model);

    if(modelCombo->currentIndex() == -1)
      m->setVehicleAt(index.row(), VehicleTypesModel::invalidIndex);
    else
      m->setVehicleAt(index.row(), modelCombo->currentIndex());
    return;
  }

  QStyledItemDelegate::setModelData(editor, model, index);
}
