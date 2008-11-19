
#ifndef SLIDES_H
#define SLIDES_H

#include <QWidget>

class QLCDNumber;
class Leds;

class Slides : public QWidget
{
    Q_OBJECT

public:
    Slides(QWidget *parent = 0);

public slots:
    void setValue(unsigned int uiDataIn_p);

signals:

private:
    QLCDNumber *pLcd1;
    Leds*       m_pDigiOutLeds;
//    QLCDNumber *pLcd2;

};

#endif

