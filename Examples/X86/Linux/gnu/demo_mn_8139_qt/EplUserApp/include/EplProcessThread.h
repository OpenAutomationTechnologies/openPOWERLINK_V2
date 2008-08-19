
#ifndef EPL_PROCESS_THREAD_H
#define EPL_PROCESS_THREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

extern "C" {
#include "global.h"
#include "Epl.h"
}


class QWidget;
class QString;

class EplProcessThread : public QThread
{
    Q_OBJECT

public:
    EplProcessThread();

    void run();
    void sigEplStatus(int iStatus_p);
    void sigNmtState(tEplNmtState State_p);
    void sigNodeAppeared(int iNodeId_p)
             { emit nodeAppeared(iNodeId_p); };
    void sigNodeDisappeared(int iNodeId_p)
             { emit nodeDisappeared(iNodeId_p); };
    void sigNodeStatus(int iNodeId_p, int iStatus_p)
             { emit nodeStatusChanged(iNodeId_p, iStatus_p); };

    tEplApiCbEvent getEventCbFunc();

    void waitForNmtStateOff();
    void reachedNmtStateOff();

signals:
    void eplStatusChanged(int iStatus_p);
    void nmtStateChanged(const QString &strState_p);
    void nodeAppeared(int iNodeId_p);
    void nodeDisappeared(int iNodeId_p);
    void allNodesRemoved();
    void nodeStatusChanged(int iNodeId_p, int iStatus_p);

private:
    QMutex         Mutex;
    QWaitCondition NmtStateOff;

    int iEplStatus;
};

#endif

