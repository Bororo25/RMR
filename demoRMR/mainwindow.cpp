#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QColor>
#include <QEvent>
#include <QFileDialog>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QWidget>
#include <cmath>
#include <iostream>

namespace {
class MapWindow : public QWidget
{
public:
    explicit MapWindow(QWidget *parent = nullptr) : QWidget(parent)
    {
        setWindowTitle("Okupačná mapa");
        resize(840, 840);
    }

    void updateMap(const std::vector<std::vector<int8_t>> &newGrid)
    {
        grid = newGrid;
        update();
    }

    void updatePath(const std::vector<std::pair<int, int>> &newRawPath,
                    const std::vector<std::pair<int, int>> &newCorners)
    {
        rawPath = newRawPath;
        corners = newCorners;
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);

        const int h = static_cast<int>(grid.size());
        if(h == 0) return;
        const int w = static_cast<int>(grid[0].size());
        if(w == 0) return;

        const double cellW = static_cast<double>(width()) / static_cast<double>(w);
        const double cellH = static_cast<double>(height()) / static_cast<double>(h);

        for(int my = 0; my < h; ++my)
        {
            for(int mx = 0; mx < w; ++mx)
            {
                const int8_t val = grid[my][mx];
                QRectF r(mx * cellW, my * cellH, cellW, cellH);

                if(val >= 50)
                    painter.fillRect(r, Qt::white);
                else if(val == 0)
                    painter.fillRect(r, QColor(45, 45, 45));
            }
        }
        auto cellCenter = [&](int mx, int my) -> QPointF
        {
            return QPointF((static_cast<double>(mx) + 0.5) * cellW,
                           (static_cast<double>(my) + 0.5) * cellH);
        };

        // celá vypočítaná cesta cez bunky
        if(rawPath.size() >= 2)
        {
            QPen pathPen(Qt::green);
            pathPen.setWidth(2);
            painter.setPen(pathPen);

            for(size_t i = 1; i < rawPath.size(); ++i)
            {
                QPointF a = cellCenter(rawPath[i - 1].first, rawPath[i - 1].second);
                QPointF b = cellCenter(rawPath[i].first, rawPath[i].second);
                painter.drawLine(a, b);
            }
        }

        // zlomové body / waypointy
        if(!corners.empty())
        {
            QPen cornerPen(Qt::yellow);
            cornerPen.setWidth(2);
            painter.setPen(cornerPen);
            painter.setBrush(Qt::yellow);

            const double r = 4.0;

            for(const auto &p : corners)
            {
                QPointF c = cellCenter(p.first, p.second);
                painter.drawEllipse(c, r, r);
            }
        }
    }

private:
    std::vector<std::vector<int8_t>> grid;
    std::vector<std::pair<int, int>> rawPath;
    std::vector<std::pair<int, int>> corners;
};
}

static MapWindow *g_mapWindow = nullptr;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // na simulátor daj "127.0.0.1", na reálneho robota jeho IP adresu
    ipaddress = "127.0.0.1";

    ui->setupUi(this);

    ui->widget->installEventFilter(this);
    ui->widget->setMouseTracking(true);

    datacounter = 0;
    updateLaserPicture = 0;

    QPushButton *saveMapBtn = new QPushButton("Uložiť mapu", this);
    saveMapBtn->setGeometry(20, 20, 140, 35);
    saveMapBtn->show();
    connect(saveMapBtn, &QPushButton::clicked, this, &MainWindow::saveMapToImage);

    QPushButton *saveMapFileBtn = new QPushButton("Uložiť grid", this);
    saveMapFileBtn->setGeometry(20, 60, 140, 35);
    saveMapFileBtn->show();
    connect(saveMapFileBtn, &QPushButton::clicked, this, &MainWindow::saveMapToFile);

    QPushButton *loadMapFileBtn = new QPushButton("Načítať grid", this);
    loadMapFileBtn->setGeometry(20, 100, 140, 35);
    loadMapFileBtn->show();
    connect(loadMapFileBtn, &QPushButton::clicked, this, &MainWindow::loadMapFromFile);

#ifndef DISABLE_OPENCV
    actIndex = -1;
    useCamera1 = false;
#endif
#ifndef DISABLE_SKELETON
    updateSkeletonPicture = 0;
#endif
}

MainWindow::~MainWindow()
{
    if(g_mapWindow)
    {
        g_mapWindow->close();
        delete g_mapWindow;
        g_mapWindow = nullptr;
    }

    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setBrush(Qt::black);

    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    QRect rect = ui->widget->geometry();
    rect.translate(0, 15);
    painter.drawRect(rect);

    if(goalMarkerValid)
    {
        const double dx = goalXcm - curXcm;
        const double dy = goalYcm - curYcm;
        const double c = std::cos(curFiRad);
        const double s = std::sin(curFiRad);

        const double fwd_cm   = dx * c + dy * s;
        const double right_cm = dx * s - dy * c;
        const double pxPerCm = 1.0;

        const double centerX = rect.width()  / 2.0 + rect.topLeft().x();
        const double centerY = rect.height() / 2.0 + rect.topLeft().y();

        const int gx = static_cast<int>(centerX + right_cm * pxPerCm);
        const int gy = static_cast<int>(centerY - fwd_cm * pxPerCm);

        QPen pen;
        pen.setWidth(3);
        pen.setColor(Qt::yellow);
        painter.setPen(pen);

        if(rect.contains(gx, gy))
            painter.drawEllipse(QPoint(gx, gy), 6, 6);
    }

#ifndef DISABLE_OPENCV
    if(useCamera1 == true && actIndex > -1)
    {
        QImage image = QImage((uchar*)frame[actIndex].data,
                              frame[actIndex].cols,
                              frame[actIndex].rows,
                              frame[actIndex].step,
                              QImage::Format_RGB888);
        painter.drawImage(rect, image.rgbSwapped());
    }
    else
#endif
    {
        if(updateLaserPicture == 1)
        {
            updateLaserPicture = 0;

            pero.setColor(Qt::red);
            painter.setPen(pero);
            painter.drawEllipse(QPoint(rect.width()/2 + rect.topLeft().x(), rect.height()/2 + rect.topLeft().y()), 15, 15);
            painter.drawLine(QPoint(rect.width()/2 + rect.topLeft().x(), rect.height()/2 + rect.topLeft().y()),
                             QPoint(rect.width()/2 + rect.topLeft().x(), rect.height()/2 + rect.topLeft().y() - 15));

            pero.setColor(Qt::green);
            painter.setPen(pero);

            for(const auto &k : copyOfLaserData)
            {
                int dist = k.scanDistance / 20;
                int xp = rect.width()  - (rect.width()/2  + dist * 2 * std::sin((360.0 - k.scanAngle) * 3.14159 / 180.0)) + rect.topLeft().x();
                int yp = rect.height() - (rect.height()/2 + dist * 2 * std::cos((360.0 - k.scanAngle) * 3.14159 / 180.0)) + rect.topLeft().y();

                if(rect.contains(xp, yp))
                    painter.drawEllipse(QPoint(xp, yp), 2, 2);
            }
        }
    }

#ifndef DISABLE_SKELETON
    if(updateSkeletonPicture == 1)
    {
        painter.setPen(Qt::red);
        for(int i = 0; i < 75; i++)
        {
            int xp = rect.width() - rect.width() * skeleJoints.joints[i].x + rect.topLeft().x();
            int yp = (rect.height() * skeleJoints.joints[i].y) + rect.topLeft().y();
            if(rect.contains(xp, yp))
                painter.drawEllipse(QPoint(xp, yp), 2, 2);
        }
    }
#endif
}

void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
    curXcm = robotX;
    curYcm = robotY;
    curFiRad = robotFi * M_PI / 180.0;

    ui->lineEdit_2->setText(QString::number(robotX, 'f', 2));
    ui->lineEdit_3->setText(QString::number(robotY, 'f', 2));
    ui->lineEdit_4->setText(QString::number(robotFi, 'f', 2));

    if(g_mapWindow)
        g_mapWindow->updateMap(_robot.getOccupancyGrid());
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(obj == ui->widget && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *me = static_cast<QMouseEvent*>(event);
        const QPoint p = me->pos();

        const int yDrawOffset = 15;
        const double pxPerCm = 1.0;

        const double centerX = ui->widget->width()  / 2.0;
        const double centerY = ui->widget->height() / 2.0 + yDrawOffset;

        const double right_cm = (p.x() - centerX) / pxPerCm;
        const double fwd_cm   = (centerY - p.y()) / pxPerCm;

        const double goalX = curXcm + fwd_cm * std::cos(curFiRad) + right_cm * std::sin(curFiRad);
        const double goalY = curYcm + fwd_cm * std::sin(curFiRad) - right_cm * std::cos(curFiRad);

        goalMarkerValid = true;
        goalXcm = goalX;
        goalYcm = goalY;
        update();

        const bool planned = _robot.planPathToGoal(goalX, goalY);

        if(g_mapWindow)
        {
            g_mapWindow->updateMap(_robot.getOccupancyGrid());

            if(planned)
            {
                g_mapWindow->updatePath(_robot.getLastRawPathCells(),
                                        _robot.getLastCornerPathCells());
            }
            else
            {
                g_mapWindow->updatePath(std::vector<std::pair<int, int>>{},
                                        std::vector<std::pair<int, int>>{});
            }
        }

        if(!planned)
            _robot.startPoseControl(goalX, goalY);

        return true;
    }

    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::saveMapToImage()
{
    auto grid = _robot.getOccupancyGrid();

    const int h = static_cast<int>(grid.size());
    if(h == 0) return;

    const int w = static_cast<int>(grid[0].size());
    if(w == 0) return;

    QImage img(w, h, QImage::Format_RGB888);

    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            const int8_t val = grid[y][x];
            if(val == 100)
                img.setPixelColor(x, y, QColor(255, 255, 255));
            else if(val == 0)
                img.setPixelColor(x, y, QColor(60, 60, 60));
            else
                img.setPixelColor(x, y, QColor(0, 0, 0));
        }
    }

    QImage scaled = img.scaled(w * 3, h * 3);

    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Uložiť mapu",
        "mapa.png",
        "PNG Images (*.png);;JPEG Images (*.jpg)");

    if(fileName.isEmpty())
        return;

    scaled.save(fileName);
}

//uloha4
void MainWindow::saveMapToFile()
{
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Uložiť occupancyGrid mapu",
        "occupancy_map.txt",
        "Text Files (*.txt)");

    if(fileName.isEmpty())
        return;

    if(_robot.saveOccupancyGridToFile(fileName.toStdString()))
        std::cout << "OccupancyGrid mapa bola ulozena." << std::endl;
    else
        std::cout << "OccupancyGrid mapu sa nepodarilo ulozit." << std::endl;
}

//uloha4
void MainWindow::loadMapFromFile()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Načítať occupancyGrid mapu",
        "occupancy_map.txt",
        "Text Files (*.txt)");

    if(fileName.isEmpty())
        return;

    if(_robot.loadOccupancyGridFromFile(fileName.toStdString()))
    {
        std::cout << "OccupancyGrid mapa bola nacitana." << std::endl;

        _robot.setUseLoadedMapOnly(true);

        if(g_mapWindow)
        {
            g_mapWindow->updateMap(_robot.getOccupancyGrid());
            g_mapWindow->updatePath(std::vector<std::pair<int, int>>{},
                                    std::vector<std::pair<int, int>>{});
        }

        update();
    }
    else
    {
        std::cout << "OccupancyGrid mapu sa nepodarilo nacitat." << std::endl;
    }
}

void MainWindow::on_pushButton_9_clicked()
{
    connect(&_robot, SIGNAL(publishPosition(double,double,double)), this, SLOT(setUiValues(double,double,double)));
    connect(&_robot, SIGNAL(publishLidar(const std::vector<LaserData> &)), this, SLOT(paintThisLidar(const std::vector<LaserData> &)));
#ifndef DISABLE_OPENCV
    connect(&_robot, SIGNAL(publishCamera(const cv::Mat &)), this, SLOT(paintThisCamera(const cv::Mat &)));
#endif
#ifndef DISABLE_SKELETON
    connect(&_robot, SIGNAL(publishSkeleton(const skeleton &)), this, SLOT(paintThisSkeleton(const skeleton &)));
#endif

    _robot.initAndStartRobot(ipaddress);

    if(!g_mapWindow)
    {
        g_mapWindow = new MapWindow();
        g_mapWindow->show();
    }

#ifndef DISABLE_JOYSTICK
    instance = QJoysticks::getInstance();
    connect(instance, &QJoysticks::axisChanged,
            [this](const int js, const int axis, const qreal value) {
                Q_UNUSED(js);
                double forw = 0.0;
                double rot = 0.0;
                if(axis == 1) { forw = -value * 300.0; }
                if(axis == 0) { rot = -value * (3.14159 / 2.0); }
                this->_robot.setSpeedVal(forw, rot);
            });
#endif
}

void MainWindow::on_pushButton_2_clicked()
{
    _robot.setSpeed(500, 0);
}

void MainWindow::on_pushButton_3_clicked()
{
    _robot.setSpeed(-250, 0);
}

void MainWindow::on_pushButton_6_clicked()
{
    _robot.setSpeed(0, 3.14159 / 2.0);
}

void MainWindow::on_pushButton_5_clicked()
{
    _robot.setSpeed(0, -3.14159 / 2.0);
}

void MainWindow::on_pushButton_4_clicked()
{
    _robot.setSpeed(0, 0);
    _robot.stopPoseControl();
}

void MainWindow::on_pushButton_clicked()
{
#ifndef DISABLE_OPENCV
    if(useCamera1 == true)
    {
        useCamera1 = false;
        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1 = true;
        ui->pushButton->setText("use laser");
    }
#endif
}

int MainWindow::paintThisLidar(const std::vector<LaserData> &laserData)
{
    copyOfLaserData = laserData;
    updateLaserPicture = 1;
    update();
    return 0;
}

#ifndef DISABLE_OPENCV
int MainWindow::paintThisCamera(const cv::Mat &cameraData)
{
    cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;
    updateLaserPicture = 1;
    update();
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int MainWindow::paintThisSkeleton(const skeleton &skeledata)
{
    memcpy(&skeleJoints, &skeledata, sizeof(skeleton));
    updateSkeletonPicture = 1;
    update();
    return 0;
}
#endif
