#include "addtraindialog.h"

#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>
#include <QCheckBox>

#include <QFormLayout>
#include <QVBoxLayout>

#include <QDialogButtonBox>

#include <QMessageBox>

#include "trainsmodel.h"
#include "trainvehiclelistmodel.h"
#include "vehicleslistdelegate.h"
#include "traintypelistmodel.h"

AddTrainDialog::AddTrainDialog(size_t segmentIndex, const float startPos,
                               const QString& segName,
                               TrainsModel *trainsModel,
                               VehicleTypesModel *vehiclesModel,
                               TrainTypeListModel *trainTypesModel,
                               bool segmentInverted,
                               QWidget *parent)
    : QDialog{parent}
    , mTrainsModel(trainsModel)
    , mVehiclesModel(vehiclesModel)
    , mTrainTypesListModel(trainTypesModel)
    , mSegmentIndex(segmentIndex)
    , mStartPos(startPos)
    , mSegmentInverted(segmentInverted)
{
    mVehiclesListModel = new TrainVehicleListModel(mVehiclesModel, this);

    mainLay = new QFormLayout(this);

    mLabel = new QLabel;
    mainLay->addRow(mLabel);

    mTrainEdit = new QLineEdit;
    mainLay->addRow(tr("Train name:"), mTrainEdit);

    modeCombo = new QComboBox;
    modeCombo->addItem(tr("Pre-made Train"), PreMadeTrain);
    modeCombo->addItem(tr("Custom Train"), CustomTrain);
    modeCombo->addItem(tr("Custom Vehicle"), CustomVehicle);
    mainLay->addRow(tr("Mode:"), modeCombo);

    placementCombo = new QComboBox;
    placementCombo->addItem(tr("Center"), int(Simulator::TrainPlacement::PlaceCenter));
    placementCombo->addItem(tr("Start"),  int(Simulator::TrainPlacement::PlaceStart));
    placementCombo->addItem(tr("End"),    int(Simulator::TrainPlacement::PlaceEnd));
    mainLay->addRow(tr("Placement:"), placementCombo);

    invertTrainCB = new QCheckBox(tr("Invert Train"));
    mainLay->addRow(invertTrainCB);

    trainCombo = new QComboBox;
    trainCombo->setModel(mTrainTypesListModel);
    trainCombo->setCurrentIndex(0);
    mainLay->addRow(tr("Train:"), trainCombo);

    customTrainWidget = new QWidget;
    QVBoxLayout *customTrainLay = new QVBoxLayout(customTrainWidget);
    QHBoxLayout *trainButLay = new QHBoxLayout;

    QPushButton *addVehicleAfter = new QPushButton(tr("Add after"));
    QPushButton *addVehicleBefore = new QPushButton(tr("Add before"));
    QPushButton *removeVehicle = new QPushButton(tr("Remove"));

    trainButLay->addWidget(addVehicleAfter);
    trainButLay->addWidget(addVehicleBefore);
    trainButLay->addWidget(removeVehicle);

    customTrainLay->addLayout(trainButLay);

    customTrainView = new QTableView;
    customTrainView->setModel(mVehiclesListModel);
    customTrainView->setItemDelegateForColumn(0,
                                              new VehiclesListDelegate(mVehiclesModel, this));
    customTrainView->setColumnWidth(0, 350);
    customTrainView->setColumnWidth(1, 100);
    customTrainLay->addWidget(customTrainView);

    mainLay->addRow(customTrainWidget);

    customLengthSpin = new QDoubleSpinBox;
    customLengthSpin->setRange(1.0, 50.0);
    customLengthSpin->setValue(20.0);
    mainLay->addRow(tr("Custom length:"), customLengthSpin);

    QDialogButtonBox *butBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                                    Qt::Horizontal);
    connect(butBox, &QDialogButtonBox::accepted,
            this, &QDialog::accept);
    connect(butBox, &QDialogButtonBox::rejected,
            this, &QDialog::reject);
    mainLay->addRow(butBox);

    QString segName_ = segName;
    if(segName_.isEmpty())
        segName_ = tr("<i>(index %1)</i>").arg(segmentIndex);

    mLabel->setText(tr("Add Train on segment %1")
                    .arg(segName_));

    setMode(Mode::PreMadeTrain);

    connect(modeCombo, &QComboBox::activated, this, [this](int idx)
    {
        setMode(Mode(idx));
    });

    connect(addVehicleAfter, &QPushButton::clicked, this, [this]()
    {
        int row = customTrainView->currentIndex().row();
        if(row < 0)
            row = 0;

        mVehiclesListModel->addVehicle(row + 1);
    });

    connect(addVehicleBefore, &QPushButton::clicked, this, [this]()
    {
        int row = customTrainView->currentIndex().row();
        if(row < 0)
            row = 0;

        mVehiclesListModel->addVehicle(row);
    });

    connect(removeVehicle, &QPushButton::clicked, this, [this]()
    {
        int row = customTrainView->currentIndex().row();
        if(row < 0)
            row = 0;

        mVehiclesListModel->removeVehicle(row);
    });

    setMinimumSize(450, 500);
    setWindowTitle(tr("Add Train"));

    std::string baseName = "treno_";
    std::string resultName;
    size_t counter = 0;
    do
    {
        resultName = baseName + std::to_string(counter++);
    } while(trainsModel->simulator()->trainExists(resultName));

    mTrainEdit->setText(QString::fromStdString(resultName));
}

void AddTrainDialog::setMode(Mode newMode)
{
    if(mode == Mode::PreMadeTrain && newMode == Mode::CustomTrain)
    {
        if(trainCombo->currentIndex() >= 0)
        {
            // Set custom train to pre-made train
            auto train = mTrainTypesListModel->getTrainAt(trainCombo->currentIndex());
            if(!train.vehicles.isEmpty())
                mVehiclesListModel->setVehicles(train.vehicles);
        }
    }
    else if(mode == Mode::CustomVehicle && newMode == Mode::CustomTrain)
    {
        // Clear custom train
        mVehiclesListModel->setVehicles({});
    }

    mode = newMode;

    mainLay->setRowVisible(trainCombo, mode == Mode::PreMadeTrain);
    mainLay->setRowVisible(customTrainWidget, mode == Mode::CustomTrain);
    mainLay->setRowVisible(customLengthSpin, mode == Mode::CustomVehicle);

    modeCombo->setCurrentIndex(mode);
}

std::vector<Simulator::Train::VehicleItem> AddTrainDialog::createTrain()
{
    if(mode == Mode::CustomVehicle)
    {
        Simulator::Train::VehicleItem item;
        item.vehicle = mTrainsModel->simulator()->addVehicle(mTrainEdit->text().toStdString(),
                                                             customLengthSpin->value(),
                                                             Color::Blue);
        return {item};
    }

    QVector<TrainVehicleListModel::Vehicle> vehicles;

    if(mode == Mode::PreMadeTrain)
    {
        if(trainCombo->currentIndex() == -1)
            return {};

        vehicles = mTrainTypesListModel->getTrainAt(trainCombo->currentIndex()).vehicles;
    }
    else
    {
        vehicles = mVehiclesListModel->vehicles();
    }

    std::vector<Simulator::Train::VehicleItem> result;
    result.reserve(vehicles.size());

    for(const auto &vehicle : vehicles)
    {
        Simulator::Train::VehicleItem item;
        item.reversed = vehicle.reversed;

        if(vehicle.vehicleTypeIndex == VehicleTypesModel::invalidIndex)
            continue;

        VehicleTypesModel::VehicleType vehicleType = mVehiclesModel->getTypeAt(vehicle.vehicleTypeIndex);
        if(vehicleType.name.isEmpty())
            continue;

        item.vehicle = mTrainsModel->simulator()->addVehicle(vehicleType.name.toStdString(),
                                                             vehicleType.length,
                                                             Color::Blue,
                                                             vehicle.vehicleTypeIndex);
        result.push_back(item);
    }

    bool invert = mSegmentInverted;
    if(invertTrainCB->isChecked())
        invert = !invert;

    if(!invert)
    {
        // !invert, make first element go to the right
        std::reverse(result.begin(), result.end());
        for(auto &vehicle : result)
        {
            vehicle.reversed = !vehicle.reversed;
        }
    }

    return result;
}

void AddTrainDialog::done(int result)
{
    if(result == QDialog::Accepted)
    {
        std::vector<Simulator::Train::VehicleItem> vehicles = createTrain();
        if(vehicles.empty())
        {
            QMessageBox::warning(this, tr("Empty Train"), tr("Cannot add empty train"));
            return;
        }

        bool invert = mSegmentInverted;
        if(invertTrainCB->isChecked())
            invert = !invert;

        Simulator::TrainPlacement placement = Simulator::TrainPlacement(placementCombo->currentData().toInt());
        if(!mSegmentInverted)
        {
            // Swap placement
            if(placement == Simulator::TrainPlacement::PlaceStart)
                placement = Simulator::TrainPlacement::PlaceEnd;
            else if(placement == Simulator::TrainPlacement::PlaceEnd)
                placement = Simulator::TrainPlacement::PlaceStart;
        }

        std::lock_guard<std::recursive_mutex> lock(mTrainsModel->simulator()->stateMutex());

        QString errStr;
        if(!mTrainsModel->addTrain(mTrainEdit->text(),
                                   vehicles,
                                   mSegmentIndex, mStartPos,
                                   !invert, placement,
                                   &errStr))
        {
            for(const auto &item : vehicles)
            {
                mTrainsModel->simulator()->removeVehicle(item.vehicle);
            }
            vehicles.clear();

            QMessageBox::warning(this, tr("Cannot add Train"),
                                 errStr);
            return;
        }
    }

    QDialog::done(result);
}
