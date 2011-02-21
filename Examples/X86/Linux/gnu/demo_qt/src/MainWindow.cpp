
#include <QtGui>

#include "MainWindow.h"
#include "EplState.h"
#include "Slides.h"
#include "EplApi.h"


MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
{

    pEplApi = NULL;

//    QTimer *timer = new QTimer(this);

    resize(800, 500);

    QVBoxLayout *pWindowLayout = new QVBoxLayout();
    pWindowLayout->setObjectName("MainLayout");
    setLayout(pWindowLayout);

    // ---------------------------------------------------------------------
    // Head Region
    // ---------------------------------------------------------------------

    pFrameSepHeadMiddle = new QFrame();
    pHeadBackground = new QFrame();
    pHeadBackground->setContentsMargins(0,0,0,0);
    QPalette tmpPal(pHeadBackground->palette());
    tmpPal.setColor(QPalette::Window, Qt::white);
    pHeadBackground->setPalette(tmpPal);
    pHeadBackground->setAutoFillBackground(true);
//    pHeadBackground->setGeometry(0,0,width(),80);

    pHeadRegion = new QHBoxLayout();
    pHeadRegion->setObjectName("HeadRegion");

//    pHeadRegion->addSpacing(10);
    pLeftLogo  = new QPixmap(":/img/sys_tec_logo.png");
    pLeftLabel = new QLabel(pHeadBackground);
    pHeadRegion->addWidget(pLeftLabel, 1, Qt::AlignLeft);
    pHeadRegion->addSpacing(10);

    pMiddleLabel = new QLabel("openPOWERLINK");
    //pMiddleLabel->setFrameShape(QFrame::Box);
    pMiddleLabel->setAlignment(Qt::AlignCenter);
    QFont tmpFont1("Arial", 34, QFont::Bold);
    pMiddleLabel->setFont(tmpFont1);
    pHeadRegion->addWidget(pMiddleLabel);
    pHeadRegion->addSpacing(10);

    pRightLogo  = new QPixmap(":/img/powerlink_logo.jpg");
    pRightLabel = new QLabel();
    pHeadRegion->addWidget(pRightLabel, 1, Qt::AlignRight);
//    pHeadRegion->addSpacing(10);

//    pWindowLayout->addLayout(pHeadRegion, 2);
    pHeadBackground->setLayout(pHeadRegion);
    pWindowLayout->addWidget(pHeadBackground, 0);

//    resizeHeadRegion();

    pWindowLayout->addSpacing(10);
    pFrameSepHeadMiddle->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepHeadMiddle);

    // ---------------------------------------------------------------------
    // Middle Region
    // ---------------------------------------------------------------------

    // separate in left and right side
    QHBoxLayout *pMiddleRegion = new QHBoxLayout();
    pMiddleRegion->setObjectName("MiddleRegion");

    // EPL network state at left side
    pEplState = new EplState;
    pMiddleRegion->addWidget(pEplState);

    pFrameSepMiddle = new QFrame();
    pFrameSepMiddle->setFrameStyle(QFrame::VLine);
    pMiddleRegion->addWidget(pFrameSepMiddle);

    // EPL presentation slides at right side
    pSlides = new Slides;
    pMiddleRegion->addWidget(pSlides);

//    connect(timer, SIGNAL(timeout()), this, SLOT(shiftLeds()));
//    timer->start(5000);

    pWindowLayout->addLayout(pMiddleRegion, 8);

    pFrameSepMiddleFoot = new QFrame();
    pFrameSepMiddleFoot->setFrameStyle(QFrame::HLine);
    pWindowLayout->addWidget(pFrameSepMiddleFoot);

    // ---------------------------------------------------------------------
    // Foot Region
    // ---------------------------------------------------------------------

    QHBoxLayout *pFootRegion = new QHBoxLayout();
    pFootRegion->setObjectName("FootRegion");

    QLabel* pNodeIdLabel = new QLabel("Node-ID:");
    pFootRegion->addWidget(pNodeIdLabel);

    QString sNodeId;
    sNodeId.setNum(EplApi::defaultNodeId());

    QValidator *pValidator = new QIntValidator(1, 254, this);
    pNodeIdEdit = new QLineEdit(sNodeId);
    pNodeIdEdit->setValidator(pValidator);
//    QSize size = pNodeIdEdit->sizeHint();
//    size.setWidth(80);
//    pNodeIdEdit->resize(size);
//    pNodeIdEdit->setMaxLength(3);
    pFootRegion->addWidget(pNodeIdEdit);

    pFootRegion->addSpacing(20);

    pStartStopEpl = new QPushButton(tr("Start EPL Stack"));
    pFootRegion->addWidget(pStartStopEpl);
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(startEpl()));

    pFootRegion->addStretch();

    pToggleMax = new QPushButton(tr("Full Screen"));
    pFootRegion->addWidget(pToggleMax);
    connect(pToggleMax, SIGNAL(clicked()),
            this, SLOT(toggleWindowState()));

    QPushButton *pQuit = new QPushButton(tr("Quit"));
    pFootRegion->addWidget(pQuit);
    connect(pQuit, SIGNAL(clicked()), qApp, SLOT(quit()));

    pWindowLayout->addLayout(pFootRegion, 0);

//    update();

//    resizeHeadRegion();

}


void MainWindow::setLcd(unsigned int uiDataIn_p)
{
    pSlides->setValue(uiDataIn_p);
}


void MainWindow::toggleWindowState()
{
    setWindowState(windowState() ^ Qt::WindowFullScreen);
    if (windowState() & Qt::WindowFullScreen)
    {
        pToggleMax->setText(tr("Window"));
    }
    else
    {
        pToggleMax->setText(tr("Full Screen"));
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    resizeHeadRegion();
//    pEplState->resizeContents(width());
//    update();

}


void MainWindow::resizeHeadRegion()
{
int iWidth;
int iPreferredWidth;
static int iPrevWidth = 0;

    iWidth = pHeadBackground->width();

    if (iPrevWidth != iWidth)
    {
        // calculate the remaining size for the left and right logo
        // considering the recommended width of the middle label and some space
        // (the recommended and not the actual width of the middle label
        // has to be used, because the latter one is no up to date on the
        // first calls to this function)
        iPreferredWidth = (iWidth - pMiddleLabel->sizeHint().width() - 60) / 2;

//        printf("resizeHeadRegion(): iWidth=%d, iPrefWidth=%d, middle label=%d hint=%d\n",
//                iWidth, iPreferredWidth, pMiddleLabel->width(), pMiddleLabel->sizeHint().width());

        pLeftLabel->setPixmap(pLeftLogo->scaledToWidth(iPreferredWidth, Qt::SmoothTransformation));

        pRightLabel->setPixmap(pRightLogo->scaledToWidth(iPreferredWidth, Qt::SmoothTransformation));

        iPrevWidth = iWidth;

        pHeadRegion->update();
    }
}


void MainWindow::startEpl()
{
bool fConvOk;
unsigned int uiNodeId;

    pNodeIdEdit->setEnabled(false);
    uiNodeId = pNodeIdEdit->text().toUInt(&fConvOk);
    if (fConvOk == false)
    {
        uiNodeId = EplApi::defaultNodeId();
    }

    // change the button to stop
    pStartStopEpl->setText(tr("Stop EPL Stack"));
    pStartStopEpl->disconnect(this, SLOT(startEpl()));
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(stopEpl()));

    pEplApi = new EplApi(this, uiNodeId);
}

void MainWindow::stopEpl()
{
    delete pEplApi;
    pStartStopEpl->setText(tr("Start EPL Stack"));
    pStartStopEpl->disconnect(this, SLOT(stopEpl()));
    connect(pStartStopEpl, SIGNAL(clicked()), this, SLOT(startEpl()));
    pNodeIdEdit->setEnabled(true);
}



