#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <vector>

#include "robot.h"
#ifndef DISABLE_JOYSTICK
#include <QJoysticks.h>
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

public:
#ifndef DISABLE_OPENCV
    bool useCamera1;
    int actIndex;
    cv::Mat frame[3];
#endif

#ifndef DISABLE_SKELETON
    int updateSkeletonPicture;
    skeleton skeleJoints;
#endif
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_9_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_clicked();

    // úloha 3
    void saveMapToImage();

    int paintThisLidar(const std::vector<LaserData> &laserData);
#ifndef DISABLE_OPENCV
    int paintThisCamera(const cv::Mat &cameraData);
#endif
#ifndef DISABLE_SKELETON
    int paintThisSkeleton(const skeleton &skeledata);
#endif

private:
    robot _robot;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);
    int updateLaserPicture = 0;
    std::vector<LaserData> copyOfLaserData;
    int datacounter = 0;
    std::string ipaddress;

    // aktuálna poloha robota
    double curXcm = 0.0;
    double curYcm = 0.0;
    double curFiRad = 0.0;

    // cieľ z kliknutia
    bool goalMarkerValid = false;
    double goalXcm = 0.0;
    double goalYcm = 0.0;

    QTimer *timer = nullptr;
#ifndef DISABLE_JOYSTICK
    QJoysticks *instance = nullptr;
#endif

public slots:
    void setUiValues(double robotX, double robotY, double robotFi);
};

#endif // MAINWINDOW_H
