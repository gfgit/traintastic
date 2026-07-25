#include "vehicleslistdelegate.h"

#include "vehiclelistmodel.h"
#include "vehiclesmodel.h"

#include <QComboBox>

VehiclesListDelegate::VehiclesListDelegate(VehiclesModel *m, QObject *parent)
  : QStyledItemDelegate{parent}
  , mVehiclesModel(m)
{

}

QWidget *VehiclesListDelegate::createEditor(QWidget *parent,
                                            const QStyleOptionViewItem &option,
                                            const QModelIndex &index) const
{
  if(index.column() == VehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = new QComboBox(parent);
    modelCombo->setModel(mVehiclesModel);
    return modelCombo;
  }

  return QStyledItemDelegate::createEditor(parent, option, index);
}

void VehiclesListDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  if(index.column() == VehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = static_cast<QComboBox *>(editor);
    const VehicleListModel *m = static_cast<const VehicleListModel *>(index.model());

    size_t vehicleTypeIdx = m->getVehicleTypeAt(index.row());
    if(vehicleTypeIdx == VehiclesModel::invalidIndex)
      modelCombo->setCurrentIndex(-1);
    else
      modelCombo->setCurrentIndex(vehicleTypeIdx);
    return;
  }

  QStyledItemDelegate::setEditorData(editor, index);
}

void VehiclesListDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  if(index.column() == VehicleListModel::VehicleName)
  {
    QComboBox *modelCombo = static_cast<QComboBox *>(editor);
    VehicleListModel *m = static_cast<VehicleListModel *>(model);

    if(modelCombo->currentIndex() == -1)
      m->setVehicleAt(index.row(), VehiclesModel::invalidIndex);
    else
      m->setVehicleAt(index.row(), modelCombo->currentIndex());
    return;
  }

  QStyledItemDelegate::setModelData(editor, model, index);
}
