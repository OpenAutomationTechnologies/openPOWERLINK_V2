
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPalette>
#include <QColor>
#include <QLabel>

#include "EplState.h"




EplState::EplState(QWidget *parent)
    : QWidget(parent)
{
    int nIdx;

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
    pEplStateLayout->addStretch(0);
    QFrame *pFrameUpper = new QFrame;
    pFrameUpper->setFrameStyle(QFrame::HLine);
    pEplStateLayout->addWidget(pFrameUpper);
    pEplStateLayout->addSpacing(20);

    // MN State in the middle

    pNmtSectionLabel = new QLabel("NMT State:");
    QFont tmpFont3;
    tmpFont3.setBold(true);
    tmpFont3.setPointSize(18);
    pNmtSectionLabel->setFont(tmpFont3);
    pEplStateLayout->addWidget(pNmtSectionLabel);
    pEplStateLayout->addSpacing(20);

    pNmtStateLayout = new QHBoxLayout;
    pEplStateLayout->addLayout(pNmtStateLayout);
    pEplStatusLed = new QToolButton;
    //pEplStatusLed->setVisible(false);
    pNmtStateLayout->addSpacing(10);
    pNmtStateLayout->addWidget(pEplStatusLed);

    pNmtStateLabel = new QLabel("Off");
    //pNmtStateLabel->setVisible(false);
    QFont tmpFont2(pNmtStateLabel->font());
    tmpFont2.setBold(true);
    tmpFont2.setPointSize(18);
    pNmtStateLabel->setFont(tmpFont2);
    pNmtStateLayout->addSpacing(10);
    pNmtStateLayout->addWidget(pNmtStateLabel);
    
    pEplStateLayout->addStretch(0);
    QFrame *pFrameLower = new QFrame;
    pFrameLower->setFrameStyle(QFrame::HLine);
    pEplStateLayout->addWidget(pFrameLower);
    pEplStateLayout->addSpacing(20);

    // CN states at lower half
    pNodesSectionLabel = new QLabel("Controlled Nodes (CNs):");
    pNodesSectionLabel->setFont(tmpFont3);
    pEplStateLayout->addWidget(pNodesSectionLabel);
    pEplStateLayout->addSpacing(20);

    pNodesLayout = new QHBoxLayout;
    pEplStateLayout->addLayout(pNodesLayout);
    for(nIdx=0; nIdx <= NODE_ID_MAX; nIdx++)
    {
        apNodes[nIdx] = new QToolButton;
        QFont tmpFont4(apNodes[nIdx]->font());
	tmpFont4.setBold(true);
	tmpFont4.setPointSize(18);
        apNodes[nIdx]->setFont(tmpFont4);
        apNodes[nIdx]->setText(QString::number(nIdx));
        pNodesLayout->addWidget(apNodes[nIdx]);
        apNodes[nIdx]->hide();
    }

    pEplStateLayout->addStretch(0);

    // ---------------------------------------------------------------------
    // other init
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

}

void EplState::setLeds(unsigned int uiDataIn_p)
{
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
}

void EplState::addNode(int iNodeId_p)
{
    if ((iNodeId_p >= 0) && (iNodeId_p <= NODE_ID_MAX))
    {
        apNodes[iNodeId_p]->show();
        apNodes[iNodeId_p]->setFixedSize(iNodeWidth, iNodeHeight);
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


void EplState::resizeContents(int iWidth_p)
{
int iHeight;
int iWidth;
int nIdx;

    iWidth = iWidth_p/30;
    iHeight = iWidth;
    for (nIdx=0; nIdx<LED_NUM; nIdx++)
    {
        apLeds[nIdx]->setFixedSize(iWidth, iHeight);
        apLeds[nIdx]->update();
    }

    pEplStatusLed->setFixedSize(iWidth, iHeight);
    //pEplStatusLed->update();
    pNmtStateLayout->update();

    iWidth *= 1.5;
    iNodeWidth = iWidth;
    iNodeHeight = iHeight;
    for (nIdx=0; nIdx <= NODE_ID_MAX; nIdx++)
    {
        if (apNodes[nIdx]->isVisible())
        {
            apNodes[nIdx]->setFixedSize(iWidth, iHeight);
        }
    }
    pNodesLayout->update();

}




