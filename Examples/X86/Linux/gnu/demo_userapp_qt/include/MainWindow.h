
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QWidget>

#include "EplApi.h"
#include "EplState.h"

class QLineEdit;
class QPushButton;
class QToolButton;
class QLabel;
class QFrame;
class Slides;

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);

    void       resizeHeadRegion();
    EplState*  getEplStateWidget() {return pEplState;};
    Slides*    getSlidesWidget() {return pSlides;};

private slots:
    void toggleWindowState();
    void startEpl();
    void stopEpl();
    void setLcd(unsigned int uiDataIn_p);

private:
    QHBoxLayout *pHeadRegion;
    QFrame      *pHeadBackground;

    QPixmap     *pLeftLogo;
    QPixmap     *pRightLogo;
    QLabel      *pLeftLabel;
    QLabel      *pMiddleLabel;
    QLabel      *pRightLabel;

    EplState    *pEplState;
    Slides      *pSlides;

    QLineEdit   *pNodeIdEdit;

    QPushButton *pToggleMax;
    QPushButton *pStartStopEpl;

    QFrame      *pFrameSepHeadMiddle;
    QFrame      *pFrameSepMiddle;
    QFrame      *pFrameSepMiddleFoot;

    EplApi      *pEplApi;

protected:
    void resizeEvent(QResizeEvent *event);
//    void paintEvent(QPaintEvent *event);

};

#endif

