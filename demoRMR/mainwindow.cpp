#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
//kniznice na branie eventov
#include <QMouseEvent>
#include <QEvent>
///Boris Supak
///Martin Brandobur



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress= "192.168.1.14";//192.168.1.14toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"

    ui->setupUi(this);

    //pridanie na event z mysky
    ui->widget->installEventFilter(this);
    ui->widget->setMouseTracking(true);

    datacounter=0;
#ifndef DISABLE_OPENCV
    actIndex=-1;
    useCamera1=false;

#endif


    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect;
    rect= ui->widget->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);
    if(goalMarkerValid)
    {
        const double dx = goalXcm - curXcm;
        const double dy = goalYcm - curYcm;
        const double c = std::cos(curFiRad);
        const double s = std::sin(curFiRad);

        //transformacia w -> r
        const double fwd_cm   = dx * c + dy * s;
        const double right_cm = dx * s - dy * c;

        const double pxPerCm = 1.0;

        const double centerX = rect.width()  / 2.0 + rect.topLeft().x();
        const double centerY = rect.height() / 2.0 + rect.topLeft().y();

        //prevod z robotovych suradnic do obrazu
        const int gx = static_cast<int>(centerX + right_cm * pxPerCm);
        const int gy = static_cast<int>(centerY - fwd_cm   * pxPerCm);

        QPen pen;
        pen.setWidth(3);
        pen.setColor(Qt::yellow);
        painter.setPen(pen);

        if(rect.contains(gx, gy))
            painter.drawEllipse(QPoint(gx, gy), 6, 6);
    }



#ifndef DISABLE_OPENCV
    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
#endif
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;


            pero.setColor(Qt::red);//farba je zelena
            painter.setPen(pero);
            painter.drawEllipse(QPoint(rect.width()/2+rect.topLeft().x(), rect.height()/2+rect.topLeft().y()),15,15);
            painter.drawLine(QPoint(rect.width()/2+rect.topLeft().x(), rect.height()/2+rect.topLeft().y()),QPoint(rect.width()/2+rect.topLeft().x(), rect.height()/2+rect.topLeft().y()-15));
            pero.setColor(Qt::green);//farba je zelena
            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(const auto &k :copyOfLaserData)
            {
                int dist=k.scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-k.scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-k.scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
#ifndef DISABLE_SKELETON
    if(updateSkeletonPicture==1 )
    {
        painter.setPen(Qt::red);
        for(int i=0;i<75;i++)
        {
            int xp=rect.width()-rect.width() * skeleJoints.joints[i].x+rect.topLeft().x();
            int yp= (rect.height() *skeleJoints.joints[i].y)+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }
    }
#endif
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
    curXcm = robotX;
    curYcm = robotY;
    curFiRad = robotFi * M_PI / 180.0; // robotFi prichádza v stupňoch

    ui->lineEdit_2->setText(QString::number(robotX, 'f', 2));
    ui->lineEdit_3->setText(QString::number(robotY, 'f', 2));
    ui->lineEdit_4->setText(QString::number(robotFi, 'f', 2));
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(obj == ui->widget && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *me = static_cast<QMouseEvent*>(event);
        const QPoint p = me->pos(); // súradnice v rámci ui->widget

        // reakcia na rect.translate(0,15))
        const int yDrawOffset = 15;

        const double pxPerCm = 1.0;

        const double centerX = ui->widget->width()  / 2.0;
        const double centerY = ui->widget->height() / 2.0 + yDrawOffset;

        const double right_cm = (p.x() - centerX) / pxPerCm;   // doprava +
        const double fwd_cm   = (centerY - p.y()) / pxPerCm;   // dopredu (hore) +

        const double goalX = curXcm + fwd_cm * std::cos(curFiRad) + right_cm * std::sin(curFiRad);
        const double goalY = curYcm + fwd_cm * std::sin(curFiRad) - right_cm * std::cos(curFiRad);
        //ulozenie do globalnej premennej
        goalMarkerValid = true;
        goalXcm = goalX;
        goalYcm = goalY;
        update();
        //zapnutie regulacie polohy
        _robot.startPoseControl(goalX, goalY);
        return true;
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::on_pushButton_9_clicked() //start button
{
    //ziskanie joystickov


    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota



    connect(&_robot,SIGNAL(publishPosition(double,double,double)),this,SLOT(setUiValues(double,double,double)));
    connect(&_robot,SIGNAL(publishLidar(const std::vector<LaserData> &)),this,SLOT(paintThisLidar(const std::vector<LaserData> &)));
#ifndef DISABLE_OPENCV
    connect(&_robot,SIGNAL(publishCamera(const cv::Mat &)),this,SLOT(paintThisCamera(const cv::Mat &)));
#endif
#ifndef DISABLE_SKELETON
    connect(&_robot,SIGNAL(publishSkeleton(const skeleton &)),this,SLOT(paintThisSkeleton(const skeleton &)));
#endif

    _robot.initAndStartRobot(ipaddress);

    #ifndef DISABLE_JOYSTICK
        instance = QJoysticks::getInstance();
    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
                instance, &QJoysticks::axisChanged,
                [this]( const int js, const int axis, const qreal value) {
        double forw=0, rot=0;
        if(/*js==0 &&*/ axis==1){forw=-value*300;}
        if(/*js==0 &&*/ axis==0){rot=-value*(3.14159/2.0);}
        this->_robot.setSpeedVal(forw,rot);
    }
    );
#endif
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    _robot.setSpeed(500,0);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    _robot.setSpeed(-250,0);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    _robot.setSpeed(0,3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    _robot.setSpeed(0,-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    _robot.setSpeed(0,0);

}




void MainWindow::on_pushButton_clicked()
{
#ifndef DISABLE_OPENCV
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
#endif
}





int MainWindow::paintThisLidar(const std::vector<LaserData> &laserData)
{
    copyOfLaserData=laserData;
    //memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;

    update();
    return 0;
}

#ifndef DISABLE_OPENCV

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int MainWindow::paintThisCamera(const cv::Mat &cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka


    updateLaserPicture=1;

    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int MainWindow::paintThisSkeleton(const skeleton &skeledata)
{
    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;
    return 0;
}
#endif
