#ifndef ROBOT_H
#define ROBOT_H

#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <queue>
#include <vector>

//uloha4
#include <fstream>
#include <string>
#include <iostream>
#include <utility>

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)

class robot : public QObject {
    Q_OBJECT
public:
    explicit robot(QObject *parent = nullptr);

    void initAndStartRobot(std::string ipaddress);

    // nastaví rýchlosti, fyzicky sa pošlú v callbacku processThisRobot
    void setSpeedVal(double forw, double rots);
    void setSpeed(double forw, double rots);

    // úloha 1/2 - polohovanie
    void startPoseControl(double goalX_cm, double goalY_cm);
    void stopPoseControl();

    // úloha 3 - okupačná mriežka
    std::vector<std::vector<int8_t>> getOccupancyGrid();

    //uloha4
    bool saveOccupancyGridToFile(const std::string &filename);
    bool loadOccupancyGridFromFile(const std::string &filename);
    void setUseLoadedMapOnly(bool value);


    // úloha 4 - plánovanie v mape
    bool planPathToGoal(double goalX_cm, double goalY_cm);
    std::vector<std::pair<int, int>> getLastRawPathCells();
    std::vector<std::pair<int, int>> getLastCornerPathCells();

signals:
    void publishPosition(double x, double y, double z);
    void publishLidar(const std::vector<LaserData> &lidata);
#ifndef DISABLE_OPENCV
    void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
    void publishSkeleton(const skeleton &skeledata);
#endif

private:
    // --- ODOMETRIA ---
    double x = 0.0;   // [cm]
    double y = 0.0;   // [cm]
    double fi = 0.0;  // [rad]

    bool odomInitialized = false;
    std::uint16_t lastEncL = 0;
    std::uint16_t lastEncR = 0;
    double gyroOffsetRad = 0.0;

    // --- RAMPA ---
    double curForwCmd = 0.0; // [mm/s]
    double curRotCmd  = 0.0; // [rad/s]
    double maxAccelForw = 250.0; // [mm/s^2]
    double maxAccelRot  = 2.5;   // [rad/s^2]

    std::uint32_t lastRobotTimestampUs = 0;
    bool rampTimestampInitialized = false;

    // --- POLOHOVANIE / VFH+ ---
    std::mutex controlMtx;
    bool poseControlActive = false;
    double goalX_cm = 0.0;
    double goalY_cm = 0.0;

    std::mutex lidarMtx;
    std::vector<LaserData> latestLidar;

    bool avoidanceEnabled = true;

    int vfhSectorCount = 90;
    double vfhMinAngleDeg = -180.0;
    double vfhMaxAngleDeg =  180.0;

    double histogramRangeCm = 180.0;
    double robotRadiusCm    = 15.0;
    double safetyMarginCm   = 15.0;
    double frontStopCm      = 24.0;

    double minTurnRadiusCm = 50.0;
    double maskMarginCm    = 5.0;

    double wideGapDeg       = 20.0;
    double edgeOffsetDeg    = 5.0;

    double histLow  = 12.0;
    double histHigh = 18.0;

    double prevFiRad = 0.0;
    bool prevFiInitialized = false;

    std::vector<bool> prevBlocked;
    bool prevBlockedInitialized = false;
    double obstacleSlowBandCm = 200.0;

    double prevChosenDirRad = 0.0;

    double kpDist = 6.0;
    double kpAng  = 1.3;
    double vMax = 320.0;
    double posDeadbandCm = 7.0;
    double wMax = (kPi / 2.0);

    // --- ÚLOHA 3: MAPOVANIE / SYNCHRONIZÁCIA ---
    struct TimedPose
    {
        std::uint32_t ts_us;
        double x_cm;
        double y_cm;
        double fi_rad;
    };

    std::mutex poseHistoryMtx;
    std::deque<TimedPose> poseHistory;
    double currentOmegaRad = 0.0;

    std::mutex mapMtx;

    double mapResolutionCm = 8.0;
    int mapWidthCells  = 280;
    int mapHeightCells = 280;
    int mapOriginCellX = mapWidthCells / 2;
    int mapOriginCellY = mapHeightCells / 2;

    std::vector<std::vector<int8_t>> occupancyGrid;
    std::vector<std::vector<uint16_t>> hitGrid;
    std::vector<std::vector<uint16_t>> freeGrid;

    // --- ÚLOHA 4: PLÁNOVANIE ---
    bool useLoadedMapOnly = false;
    struct GridPoint
    {
        int x;
        int y;
    };

    struct WorldPoint
    {
        double x_cm;
        double y_cm;
    };

    std::vector<WorldPoint> plannedPathCm;
    int plannedPathIndex = 0;
    bool followingPlannedPath = false;

    std::vector<GridPoint> lastRawPathGrid;
    std::vector<GridPoint> lastCornerPathGrid;

    // true = neznáme bunky sa pri plánovaní berú ako voľné
    bool planUnknownAsFree = true;

    // mapovanie / mriežka
    void initOccupancyGrid();
    std::vector<std::vector<int8_t>> getOccupancyGridUnsafe() const;
    bool worldToMap(double wx_cm, double wy_cm, int &mx, int &my) const;
    bool mapToWorld(int mx, int my, double &wx_cm, double &wy_cm) const;
    void markCellFree(int mx, int my);
    void markCellOccupied(int mx, int my);
    void raytraceFreeCells(int x0, int y0, int x1, int y1);
    void updateMapFromLidar(const std::vector<LaserData> &laserData);

    // synchronizácia polohy
    bool interpolatePose(std::uint32_t ts_us, double &ix, double &iy, double &ifi);
    static double interpAngle(double a0, double a1, double t);

    // plánovanie
    bool makeInflatedObstacleGrid(std::vector<std::vector<uint8_t>> &blocked) const;
    bool extractWavePath(const std::vector<std::vector<int>> &wave,
                         const GridPoint &start,
                         const GridPoint &goal,
                         std::vector<GridPoint> &rawPath) const;
    std::vector<GridPoint> simplifyPathToCorners(const std::vector<GridPoint> &rawPath) const;
    void startNextPlannedWaypoint();

    // VFH+
    double computeAvoidanceDirection(double goalDirRad,
                                     double &frontMinCm,
                                     bool &haveCandidate);
    bool canGoDirectlyToGoal(double goalDirRad, double goalDistCm);

    static constexpr double kPi = 3.14159265358979323846;

    static inline double deg2rad(double d) { return d * (kPi / 180.0); }
    static inline double rad2deg(double r) { return r * (180.0 / kPi); }

    static inline double normalizeAngleDeg(double a)
    {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    static inline double normalizeAngleRad(double a)
    {
        while (a > kPi) a -= 2.0 * kPi;
        while (a < -kPi) a += 2.0 * kPi;
        return a;
    }

    static inline double clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    static inline std::int16_t ticksDiff(std::uint16_t now, std::uint16_t prev)
    {
        const std::uint16_t delta = static_cast<std::uint16_t>(now - prev);
        return static_cast<std::int16_t>(delta);
    }

    static inline double stepTowards(double cur, double target, double maxStep)
    {
        const double diff = target - cur;
        if (std::fabs(diff) <= maxStep) return target;
        return cur + (diff > 0.0 ? maxStep : -maxStep);
    }

    static inline double gyroRawToRad(double gyroRaw)
    {
        const double deg = gyroRaw / 100.0;
        return deg * (kPi / 180.0);
    }

    // rýchlosti nastavené cez setSpeedVal
    double forwardspeed = 0.0;  // [mm/s]
    double rotationspeed = 0.0; // [rad/s]

    int processThisLidar(const std::vector<LaserData> &laserData);
    int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
    int processThisCamera(cv::Mat cameraData);
#endif

    std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
    cv::Mat frame[3];
#endif

    libRobot robotCom;

    int datacounter = 0;
#ifndef DISABLE_OPENCV
    bool useCamera1 = false;
    int actIndex = -1;
#endif

#ifndef DISABLE_SKELETON
    int processThisSkeleton(skeleton skeledata);
    int updateSkeletonPicture = 0;
    skeleton skeleJoints;
#endif
    int useDirectCommands = 0;
};

#endif // ROBOT_H
