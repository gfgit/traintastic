#ifndef ADDTRAINDIALOG_H
#define ADDTRAINDIALOG_H

#include <QDialog>
#include <traintastic/simulator/simulator.hpp>

class QLabel;
class QLineEdit;
class QDoubleSpinBox;
class QComboBox;
class QTableView;
class QCheckBox;

class TrainsModel;
class VehicleTypesModel;
class TrainTypeListModel;
class TrainVehicleListModel;

class QFormLayout;

class AddTrainDialog : public QDialog
{
    Q_OBJECT
public:
    AddTrainDialog(size_t segmentIndex, const float startPos, const QString &segName,
                   TrainsModel *trainsModel,
                   VehicleTypesModel *vehiclesModel,
                   TrainTypeListModel *trainTypesModel,
                   bool segmentInverted,
                   QWidget *parent = nullptr);

    enum Mode
    {
        PreMadeTrain = 0,
        CustomTrain = 1,
        CustomVehicle = 2
    };

private:
    void setMode(Mode newMode);

    std::vector<Simulator::Train::VehicleItem> createTrain();

protected:
    void done(int result);

private:
    QFormLayout *mainLay;
    QLabel *mLabel;
    QLineEdit *mTrainEdit;

    QComboBox *modeCombo;
    QComboBox *placementCombo;

    QComboBox *trainCombo;

    QWidget *customTrainWidget;
    QTableView *customTrainView;

    QDoubleSpinBox *customLengthSpin;

    QCheckBox *invertTrainCB;

    TrainsModel *mTrainsModel = nullptr;
    VehicleTypesModel *mVehiclesModel = nullptr;
    TrainVehicleListModel *mVehiclesListModel = nullptr;
    TrainTypeListModel *mTrainTypesListModel = nullptr;

    Mode mode = Mode::CustomVehicle;
    size_t mSegmentIndex = 0;
    float mStartPos = 0.0f;
    bool mSegmentInverted = false;
};

#endif // ADDTRAINDIALOG_H
