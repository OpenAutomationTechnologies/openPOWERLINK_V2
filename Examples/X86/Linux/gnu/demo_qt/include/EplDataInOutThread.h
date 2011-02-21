
#ifndef EPL_DATA_IN_OUT_THREAD_H
#define EPL_DATA_IN_OUT_THREAD_H

#include <QThread>

extern "C" {
#include "global.h"
#include "Epl.h"
}


class QWidget;
class QString;


typedef struct
{
    BYTE    m_bVarIn1;
    BYTE    m_abSelect[3];  // pushbuttons from CNs

} __attribute__((packed)) tAppProcessImageOut;

typedef struct
{
    BYTE    m_bVarOut1;
    BYTE    m_bLedsRow1;    // current state of the LEDs in row 1
    BYTE    m_bLedsRow2;    // current state of the LEDs in row 2

} __attribute__((packed)) tAppProcessImageIn;


class EplDataInOutThread : public QThread
{
    Q_OBJECT

public:
    EplDataInOutThread();

    void run();

    void acknowledge();

    tEplKernel SetupProcessImage();

signals:
    void processImageInChanged(unsigned int uiData_p);
    void processImageOutChanged(unsigned int uiData_p);

private:
//    tEplApiProcessImage m_EplPiIn;
//    tEplApiProcessImage m_EplPiOut;

    volatile unsigned int   m_uiAckCount;
    unsigned int            m_uiCount;
};

#endif

