
#ifndef EPL_DATA_IN_OUT_THREAD_H
#define EPL_DATA_IN_OUT_THREAD_H

#include <QThread>

extern "C" {
#include "global.h"
#include "Epl.h"
}


class QWidget;
class QString;


class EplDataInOutThread : public QThread
{
    Q_OBJECT

public:
    EplDataInOutThread();

    void run();

signals:
    void processImageChanged(unsigned int iDataIn);

private:
    tEplApiProcessImage EplPiIn;
    tEplApiProcessImage EplPiOut;

    unsigned int        uiLedsOld;
};

#endif

