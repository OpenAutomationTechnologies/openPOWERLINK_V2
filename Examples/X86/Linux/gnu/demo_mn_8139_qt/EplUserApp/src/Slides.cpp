
#include <QLCDNumber>
#include <QVBoxLayout>

#include "Slides.h"

Slides::Slides(QWidget *parent)
    : QWidget(parent)
{
    QVBoxLayout *pSlideLayout = new QVBoxLayout;
    setLayout(pSlideLayout);

    pLcd1 = new QLCDNumber(2);
    pLcd1->display(0);
    pLcd1->setSegmentStyle(QLCDNumber::Filled);
    pLcd1->setMaximumHeight(400);
    pLcd1->setFrameShape(QFrame::NoFrame);
    pLcd1->setMode(QLCDNumber::Hex);
    //pLcd2 = new QLCDNumber(1);
    //pLcd2->display(0);
    //pLcd2->setSegmentStyle(QLCDNumber::Filled);
    //pLcd2->setFrameShape(QFrame::NoFrame);
    //pLcd2->setMode(QLCDNumber::Hex);
    pSlideLayout->addStretch(1);
    pSlideLayout->addWidget(pLcd1, 4);
    //pSlideLayout->addWidget(pLcd2, 4);
    pSlideLayout->addStretch(1);

}

void Slides::setLcd(unsigned int uiDataIn_p)
{
    pLcd1->display((int)(uiDataIn_p & 0xFF));
    //pLcd2->display((int)(uiDataIn_p & 0x0F));
}

