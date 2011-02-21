
#ifndef EPL_API_H
#define EPL_API_H


#include "EplProcessThread.h"
#include "EplDataInOutThread.h"

extern "C" {
#include "global.h"
#include "Epl.h"
}


class MainWindow;


class QWidget;


class EplApi
{
public:
    EplApi(MainWindow *pMainWindow_p, unsigned int uiNodeId_p);
    ~EplApi();
    static unsigned int defaultNodeId();

private:
    tEplApiInitParam  EplApiInitParam;

    EplProcessThread  *pEplProcessThread;
    EplDataInOutThread  *pEplDataInOutThread;
};

#endif

