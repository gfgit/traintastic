#include "trainsmodel.h"

#include <traintastic/simulator/simulator.hpp>

#include <QTimerEvent>

#include <QRandomGenerator>

TrainsModel::TrainsModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

QVariant TrainsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        switch (section)
        {
        case Columns::Name:
            return tr("Name");
        case Columns::State:
            return tr("State");
        default:
            break;
        }
    }

    return QAbstractTableModel::headerData(section, orientation, role);
}

int TrainsModel::rowCount(const QModelIndex &p) const
{
    return p.isValid() ? 0 : mTrains.size();
}

int TrainsModel::columnCount(const QModelIndex &p) const
{
    return p.isValid() ? 0 : NCols;
}

QVariant TrainsModel::data(const QModelIndex &idx, int role) const
{
    if (!idx.isValid() || idx.column() >= NCols || idx.row() >= mTrains.size())
        return QVariant();

    auto it = mTrains.begin();
    std::advance(it, idx.row());
    const Train& train = *it;

    switch (role)
    {
    case Qt::DisplayRole:
    {
        switch (idx.column())
        {
        case Name:
            return train.name;
        case State:
        {
            if(train.speedKmH == 0)
                return tr("Stopped");
            return tr("%1 km/h").arg(train.speedKmH);
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }

    return QVariant();
}

bool TrainsModel::addTrain(const QString &name,
                           const std::vector<Simulator::Train::VehicleItemFull> &vehicles,
                           size_t segmentIdx, const float startPos, bool invert, Simulator::TrainPlacement placement, QString *errOut)
{
    if(!mSimulator)
        return false;

    std::lock_guard<std::recursive_mutex> lock(mSimulator->stateMutex());

    if(name.isEmpty())
    {
        *errOut = tr("Name is empty!");
        return false;
    }

    if(segmentIdx >= mSimulator->staticData.trackSegments.size())
    {
        *errOut = tr("Segment does not exist!");
        return false;
    }

    if(mSimulator->getSimModeUnlocked() == Simulator::SimMode::Replica)
    {
      // Generate a token
      do
      {
        mCurrentToken = QRandomGenerator::global()->generate();
      } while(mCurrentToken == 0); // Avoid null token as result

      mTokenExpire.start(1500, Qt::CoarseTimer, this);

      mSimulator->requestAddTrain(name.toStdString(), vehicles, segmentIdx, mCurrentToken, startPos, placement, invert);
      return true;
    }

    quint32 trainId = Simulator::invalidTrainId;
    if(!mSimulator->addTrain(name.toStdString(), DecoderProtocol::DCCShort, 3,
                             vehicles, segmentIdx, trainId, startPos, placement, invert))
    {
        *errOut = tr("Train is too long");
        return false;
    }

    beginResetModel();
    mTrains.insert(name, Train{name, 0.0});
    endResetModel();

    emit setCurrentTrain(trainId);

    return true;
}

bool TrainsModel::removeTrain(const QString &name)
{
    if(!mSimulator || !mTrains.contains(name))
        return false;

    // if(!mSimulator->removeTrain(name.toStdString(), true))
    //     return false;

    // beginResetModel();
    // mTrains.remove(name);
    // endResetModel();
    return true;
}

void TrainsModel::setSimulator(Simulator *sim)
{
    beginResetModel();
    mTrains.clear();

    mCurrentToken = 0;
    mTokenExpire.stop();

    mSimulator = sim;

    if(mSimulator)
    {
    }

    endResetModel();
}

Simulator *TrainsModel::simulator() const
{
    return mSimulator;
}

void TrainsModel::onAddTrainReply(quint32 trainId, quint32 token, bool isError)
{
  if(token != mCurrentToken && mCurrentToken != 0)
    return;

  mCurrentToken = 0;
  mTokenExpire.stop();

  if(trainId == Simulator::invalidTrainId)
    isError = true;

  {
    std::lock_guard<std::recursive_mutex> lock(mSimulator->stateMutex());
    if(!mSimulator->stateData().trains.contains(trainId))
    {
      isError = true;
      trainId = Simulator::invalidTrainId;
    }
  }

  if(!isError)
  {
    emit setCurrentTrain(trainId);
    return;
  }


}

void TrainsModel::timerEvent(QTimerEvent *ev)
{
  if(ev->timerId() == mTokenExpire.timerId())
  {
    mTokenExpire.stop();
    mCurrentToken = 0;
    return;
  }

  QAbstractTableModel::timerEvent(ev);
}
