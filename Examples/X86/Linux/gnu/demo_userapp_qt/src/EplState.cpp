
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QLabel>

#include "EplState.h"
#include "Leds.h"


#define LED_SIZE    30
#define NODE_WIDTH  (LED_SIZE * 3 / 2)
#define NODE_HEIGHT (LED_SIZE)


EplState::EplState(QWidget *parent)
    : QWidget(parent)
{
    int nIdx;

    QFont LabelFont;
    LabelFont.setBold(true);
    LabelFont.setPointSize(18);

    QPalette PalLeds;
    PalLeds.setColor(QPalette::Active, QPalette::Window, Qt::blue);
    PalLeds.setColor(QPalette::Active, QPalette::Button, Qt::blue);
    PalLeds.setColor(QPalette::Inactive, QPalette::Window, Qt::blue);
    PalLeds.setColor(QPalette::Inactive, QPalette::Button, Qt::blue);

    // ---------------------------------------------------------------------
    // Color init
    // ---------------------------------------------------------------------

    PalGreenButton.setColor(QPalette::Active, QPalette::Window, Qt::green);
    PalGreenButton.setColor(QPalette::Active, QPalette::Button, Qt::green);
    PalGreenButton.setColor(QPalette::Inactive, QPalette::Window, Qt::green);
    PalGreenButton.setColor(QPalette::Inactive, QPalette::Button, Qt::green);
    PalYellowButton.setColor(QPalette::Active, QPalette::Window, Qt::yellow);
    PalYellowButton.setColor(QPalette::Active, QPalette::Button, Qt::yellow);
    PalYellowButton.setColor(QPalette::Inactive, QPalette::Window, Qt::yellow);
    PalYellowButton.setColor(QPalette::Inactive, QPalette::Button, Qt::yellow);
    PalRedButton.setColor(QPalette::Active, QPalette::Window, Qt::red);
    PalRedButton.setColor(QPalette::Active, QPalette::Button, Qt::red);
    PalRedButton.setColor(QPalette::Inactive, QPalette::Window, Qt::red);
    PalRedButton.setColor(QPalette::Inactive, QPalette::Button, Qt::red);

    // ---------------------------------------------------------------------
    // Layout
    // ---------------------------------------------------------------------

    QVBoxLayout *pEplStateLayout = new QVBoxLayout;
    setLayout(pEplStateLayout);

    pEplStateLayout->addStretch(0);

    // process value LEDs at upper half
    QLabel* pDigiInLabel = new QLabel("Digital Inputs:");
    pDigiInLabel->setFont(LabelFont);
    pEplStateLayout->addWidget(pDigiInLabel);

    m_pLeds = new Leds(LED_NUM, LED_SIZE, PalLeds);
    pEplStateLayout->addWidget(m_pLeds);
/*
    QHBoxLayout *pLedsLayout = new QHBoxLayout;
    pEplStateLayout->addLayout(pLedsLayout);

    //pLedsLayout->addSpacing(10);
    for(nIdx=0; nIdx < LED_NUM; nIdx++)
    {
        apLeds[nIdx] = new QToolButton;
        apLeds[nIdx]->setPalette(PalRedButton);
        //pLedsLayout->addWidget(apLeds[nIdx], 0, Qt::AlignLeft);
        pLedsLayout->addWidget(apLeds[nIdx]);
    }
*/
    pEplStateLayout->addStretch(1);
    QFrame *pFrameUpper = new QFrame;
    pFrameUpper->setFrameStyle(QFrame::HLine);
    pEplStateLayout->addWidget(pFrameUpper);
//    pEplStateLayout->addSpacing(20);
    pEplStateLayout->addStretch(1);

    // MN State in the middle

    pNmtSectionLabel = new QLabel("NMT State:");
    pNmtSectionLabel->setFont(LabelFont);
    pEplStateLayout->addWidget(pNmtSectionLabel);
    pEplStateLayout->addSpacing(20);

    pNmtStateLayout = new QHBoxLayout;
    pEplStateLayout->addLayout(pNmtStateLayout);
    pEplStatusLed = new QToolButton;
    pEplStatusLed->setFixedSize(LED_SIZE, LED_SIZE);
    //pEplStatusLed->setVisible(false);
    pNmtStateLayout->addSpacing(10);
    pNmtStateLayout->addWidget(pEplStatusLed);

    pNmtStateLabel = new QLabel("Off");
    //pNmtStateLabel->setVisible(false);
//    QFont LabelFont(pNmtStateLabel->font());
//    LabelFont.setBold(true);
//    LabelFont.setPointSize(18);
    pNmtStateLabel->setFont(LabelFont);
    pNmtStateLayout->addSpacing(10);
    pNmtStateLayout->addWidget(pNmtStateLabel);

    pEplStateLayout->addStretch(0);
    QFrame *pFrameLower = new QFrame;
    pFrameLower->setFrameStyle(QFrame::HLine);
    pEplStateLayout->addStretch(1);
    pEplStateLayout->addWidget(pFrameLower);
    pEplStateLayout->addStretch(1);
//    pEplStateLayout->addSpacing(20);

    // CN states at lower half
    pNodesSectionLabel = new QLabel("Controlled Nodes (CNs):");
    pNodesSectionLabel->setFont(LabelFont);
    pEplStateLayout->addWidget(pNodesSectionLabel);
    pEplStateLayout->addSpacing(20);

    pNodesLayout = new QHBoxLayout;
    pEplStateLayout->addLayout(pNodesLayout);
    for(nIdx=0; nIdx <= NODE_ID_MAX; nIdx++)
    {
        apNodes[nIdx] = new QToolButton;
//        QFont LabelFont(apNodes[nIdx]->font());
//	LabelFont.setBold(true);
//	LabelFont.setPointSize(18);
        apNodes[nIdx]->setFont(LabelFont);
        apNodes[nIdx]->setText(QString::number(nIdx));
        pNodesLayout->addWidget(apNodes[nIdx]);
        apNodes[nIdx]->hide();
    }

    pEplStateLayout->addStretch(1);

}

void EplState::setLeds(unsigned int uiDataIn_p)
{
    m_pLeds->setLeds(uiDataIn_p);
/*
    int nIdx;

    for(nIdx=0; nIdx < LED_NUM; nIdx++)
    {
        if(uiDataIn_p & (1<<nIdx))
        {
            apLeds[nIdx]->setDisabled(false);
        }
        else
        {
            apLeds[nIdx]->setDisabled(true);
        }
    }
*/
}

void EplState::addNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        apNodes[iNodeId_p]->show();
        apNodes[iNodeId_p]->setFixedSize(NODE_WIDTH, NODE_HEIGHT);
        pNodesLayout->update();
    }

}

void EplState::removeNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        apNodes[iNodeId_p]->hide();
    }
}

void EplState::removeAllNodes()
{
int nIdx;

    // count() gives all widgets (hidden ones too)
    for(nIdx=0; nIdx < pNodesLayout->count(); nIdx++)
    {
        pNodesLayout->itemAt(nIdx)->widget()->hide();
    }
}

void EplState::setNodeStatus(int iNodeId_p, int iStatus_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        apNodes[iNodeId_p]->show();
        switch (iStatus_p)
        {
            case 0:
            {
                apNodes[iNodeId_p]->setPalette(palette());
                break;
            }
            case 1:
            {
                apNodes[iNodeId_p]->setPalette(PalYellowButton);
                break;
            }
            case 2:
            {
                apNodes[iNodeId_p]->setPalette(PalGreenButton);
                break;
            }
            case -1:
            default:
            {
                apNodes[iNodeId_p]->setPalette(PalRedButton);
            }
        }
    }
}

void EplState::setEplStatusLed(int iStatus_p)
{
    pEplStatusLed->show();
    switch (iStatus_p)
    {
        case 0:
        {
            pEplStatusLed->setPalette(palette());
            break;
        }
        case 1:
        {
            pEplStatusLed->setPalette(PalYellowButton);
            break;
        }
        case 2:
        {
            pEplStatusLed->setPalette(PalGreenButton);
            break;
        }
        case -1:
        default:
        {
            pEplStatusLed->setPalette(PalRedButton);
        }
    }
}

void EplState::setNmtStateText(const QString &strState_p)
{
    pNmtStateLabel->show();
    pNmtStateLabel->setText(strState_p);
}




