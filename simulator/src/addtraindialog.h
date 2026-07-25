#ifndef ADDTRAINDIALOG_H
#define ADDTRAINDIALOG_H

#include <QDialog>
#include <traintastic/simulator/simulator.hpp>

class QLabel;
class QLineEdit;
class QDoubleSpinBox;
class QComboBox;
class QTableView;

class TrainsModel;
class VehiclesModel;
class VehicleListModel;

class QFormLayout;

class AddTrainDialog : public QDialog
{
    Q_OBJECT
public:
    AddTrainDialog(size_t segmentIndex, const float startPos, const QString &segName,
                   TrainsModel *trainsModel, VehiclesModel *vehiclesModel,
                   QWidget *parent = nullptr);

    enum Mode
    {
        PreMadeTrain = 0,
        CustomTrain = 1,
        CustomVehicle = 2
    };

private:
    void setMode(Mode mode);


    std::vector<Simulator::Train::VehicleItem> createTrain();

protected:
    void done(int result);

private:
    QFormLayout *mainLay;
    QLabel *mLabel;
    QLineEdit *mTrainEdit;

    QComboBox *modeCombo;

    QComboBox *trainCombo;

    QWidget *customTrainWidget;
    QTableView *customTrainView;

    QDoubleSpinBox *customLengthSpin;

    TrainsModel *mTrainsModel = nullptr;
    VehiclesModel *mVehiclesModel = nullptr;
    VehicleListModel *mVehiclesListModel = nullptr;
    size_t mSegmentIndex = 0;
    float mStartPos = 0.0f;
};

#endif // ADDTRAINDIALOG_H
