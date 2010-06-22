
#ifndef EPL_STATE_H
#define EPL_STATE_H

#include <QWidget>

#define LED_NUM        10
#define NODE_ID_MAX    255


class QToolButton;
class QPalette;
class QHBoxLayout;
class QLabel;
class Leds;

class EplState : public QWidget
{
    Q_OBJECT

public:
    EplState(QWidget *parent = 0);

public slots:
    void setLeds(unsigned int uiDataIn_p);
    void addNode(int iNodeId_p);
    void removeNode(int iNodeId_p);
    void removeAllNodes();
    void setNodeStatus(int iNodeId_p, int iStatus_p);
    void setEplStatusLed(int iStatus_p);
    void setNmtStateText(const QString &strState_p);

private:
    QPalette PalGreenButton;
    QPalette PalYellowButton;
    QPalette PalRedButton;

    QHBoxLayout  *pNodesLayout;

    QToolButton *pEplStatusLed;
    QLabel      *pNmtStateLabel;
    QHBoxLayout *pNmtStateLayout;

    QToolButton* apNodes[NODE_ID_MAX+1];

    Leds*       m_pLeds;

    QLabel      *pNmtSectionLabel;
    QLabel      *pNodesSectionLabel;

    int   iNodeWidth;
    int   iNodeHeight;

};

#endif
