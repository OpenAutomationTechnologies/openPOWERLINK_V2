
#ifndef SLIDES_H
#define SLIDES_H

#include <QWidget>

class QLCDNumber;

class Slides : public QWidget
{
    Q_OBJECT

public:
    Slides(QWidget *parent = 0);

public slots:
    void setLcd(unsigned int uiDataIn_p);

signals:

private:
    QLCDNumber *pLcd1;
    QLCDNumber *pLcd2;

};

#endif

