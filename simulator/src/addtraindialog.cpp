#include "addtraindialog.h"

#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>

#include <QFormLayout>
#include <QVBoxLayout>

#include <QDialogButtonBox>

#include <QMessageBox>

#include "trainsmodel.h"
#include "vehiclelistmodel.h"
#include "vehicleslistdelegate.h"

AddTrainDialog::AddTrainDialog(size_t segmentIndex, const float startPos,
                               const QString& segName,
                               TrainsModel *trainsModel, VehiclesModel *vehiclesModel,
                               QWidget *parent)
    : QDialog{parent}
    , mTrainsModel(trainsModel)
    , mVehiclesModel(vehiclesModel)
    , mSegmentIndex(segmentIndex)
    , mStartPos(startPos)
{
    mVehiclesListModel = new VehicleListModel(mVehiclesModel, this);

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

    trainCombo = new QComboBox;
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

    setMode(Mode::CustomTrain);

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

    setMinimumSize(450, 300);
    setWindowTitle(tr("Add Train"));
}

void AddTrainDialog::setMode(Mode mode)
{
    mainLay->setRowVisible(trainCombo, mode == Mode::PreMadeTrain);
    mainLay->setRowVisible(customTrainWidget, mode == Mode::CustomTrain);
    mainLay->setRowVisible(customLengthSpin, mode == Mode::CustomVehicle);

    modeCombo->setCurrentIndex(mode);
}

std::vector<Simulator::Train::VehicleItem> AddTrainDialog::createTrain()
{
    if(modeCombo->currentIndex() == Mode::CustomVehicle)
    {
        Simulator::Train::VehicleItem item;
        item.vehicle = mTrainsModel->simulator()->addVehicle(mTrainEdit->text().toStdString(),
                                                             customLengthSpin->value(),
                                                             Color::Blue);
        return {item};
    }

    QVector<VehicleListModel::Vehicle> vehicles = mVehiclesListModel->vehicles();

    if(modeCombo->currentIndex() == Mode::PreMadeTrain)
    {
        // TODO: fill list or also use it in table as default start
    }

    std::vector<Simulator::Train::VehicleItem> result;
    result.reserve(vehicles.size());

    for(const auto &vehicle : vehicles)
    {
        Simulator::Train::VehicleItem item;
        item.reversed = vehicle.reverse;

        if(vehicle.vehicleTypeIndex == VehiclesModel::invalidIndex)
            continue;

        VehiclesModel::VehicleType vehicleType = mVehiclesModel->getTypeAt(vehicle.vehicleTypeIndex);
        if(vehicleType.name.isEmpty())
            continue;

        item.vehicle = mTrainsModel->simulator()->addVehicle(vehicleType.name.toStdString(),
                                                             vehicleType.length,
                                                             Color::Blue,
                                                             vehicle.vehicleTypeIndex);
        result.push_back(item);
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

        QString errStr;
        if(!mTrainsModel->addTrain(mTrainEdit->text(),
                                   vehicles,
                                   mSegmentIndex, mStartPos,
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
