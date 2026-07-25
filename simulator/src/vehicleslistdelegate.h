#ifndef VEHICLESLISTDELEGATE_H
#define VEHICLESLISTDELEGATE_H

#include <QStyledItemDelegate>

class VehicleListModel;
class VehiclesModel;

class VehiclesListDelegate : public QStyledItemDelegate
{
  Q_OBJECT
public:
  VehiclesListDelegate(VehiclesModel *m, QObject *parent = nullptr);

  QWidget *createEditor(QWidget *parent,
                        const QStyleOptionViewItem &option,
                        const QModelIndex &index) const override;

  void setEditorData(QWidget *editor,
                     const QModelIndex &index) const override;

  void setModelData(QWidget *editor,
                    QAbstractItemModel *model,
                    const QModelIndex &index) const override;

private:
  VehiclesModel *mVehiclesModel = nullptr;
};

#endif // VEHICLESLISTDELEGATE_H
