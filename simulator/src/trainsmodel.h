#ifndef TRAINSMODEL_H
#define TRAINSMODEL_H

#include <QAbstractTableModel>

#include <QMap>
#include <QBasicTimer>

#include <traintastic/enum/color.hpp>
#include <traintastic/simulator/simulator.hpp>

class Simulator;

class TrainsModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum Columns
    {
        Name = 0,
        State,
        NCols
    };

    explicit TrainsModel(QObject *parent = nullptr);

    // Header:
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    // Basic functionality:
    int rowCount(const QModelIndex &p = QModelIndex()) const override;
    int columnCount(const QModelIndex &p = QModelIndex()) const override;

    QVariant data(const QModelIndex &idx, int role = Qt::DisplayRole) const override;

    bool addTrain(const QString& name,
                  const std::vector<Simulator::Train::VehicleItemFull> &vehicles,
                  size_t segmentIdx, const float startPos, bool invert,
                  Simulator::TrainPlacement placement,
                  QString *errOut);

    bool removeTrain(const QString& name);

    void setSimulator(Simulator *sim);

    Simulator *simulator() const;

    void onAddTrainReply(quint32 trainId, quint32 token, bool isError);

    void onSetMasterFollowTrain(quint32 trainId);

  protected:
    void timerEvent(QTimerEvent *ev) override;

signals:
    void setCurrentTrain(quint32 trainId);
    void setFollowTrain(quint32 trainId);
    void trainReplicaAddError(); // TODO: error string

private:
    Simulator *mSimulator = nullptr;

    struct Train
    {
        QString name;
        double speedKmH;
    };

    QMap<QString, Train> mTrains;

    QBasicTimer mTokenExpire;
    quint32 mCurrentToken = 0;
};

#endif // TRAINSMODEL_H
