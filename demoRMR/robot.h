#ifndef ROBOT_H
#define ROBOT_H

#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <chrono>
#include <limits>
#include <vector>
#include <deque>

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

class robot : public QObject
{
    Q_OBJECT

public:
    explicit robot(QObject *parent = nullptr);

    void initAndStartRobot(std::string ipaddress);

    void setSpeedVal(double forw, double rots);
    void setSpeed(double forw, double rots);

    void startPoseControl(double goalX_cm, double goalY_cm);
    void stopPoseControl();

    std::vector<std::vector<int8_t>> getOccupancyGrid();

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
    double x;   // [cm]
    double y;   // [cm]
    double fi;  // [rad]

    std::mutex poseMtx;

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

    // --- POLOHOVANIE ---
    std::mutex controlMtx;
    bool poseControlActive = false;
    double goalX_cm = 0.0;
    double goalY_cm = 0.0;

    // --- VYHÝBANIE SA PREKÁŽKAM (VFH+) ---
    std::mutex lidarMtx;
    std::vector<LaserData> latestLidar;

    bool avoidanceEnabled = true;

    int vfhSectorCount = 90;
    double vfhMinAngleDeg = -180.0;
    double vfhMaxAngleDeg = 180.0;

    double histogramRangeCm = 180.0;
    double robotRadiusCm    = 15.0;
    double safetyMarginCm   = 8.0;
    double frontStopCm      = 30.0;

    double wideGapDeg       = 30.0;
    double edgeOffsetDeg    = 0.0;

    double histLow  = 110.0;
    double histHigh = 140.0;

    std::vector<bool> prevBlocked;
    bool prevBlockedInitialized = false;
    double obstacleSlowBandCm = 80.0;

    double prevChosenDirRad = 0.0;

    double minTurnRadiusCm = 50.0;
    double maskMarginCm    = 5.0;

    // --- MAPOVANIE / SYNCHRONIZÁCIA ---
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
    int mapOriginCellX = 140;
    int mapOriginCellY = 140;

    std::vector<std::vector<int8_t>> occupancyGrid;
    std::vector<std::vector<uint16_t>> hitGrid;
    std::vector<std::vector<uint16_t>> freeGrid;

    bool interpolatePose(std::uint32_t ts_us, double &ix, double &iy, double &ifi);
    static double interpAngle(double a0, double a1, double t);

    void initOccupancyGrid();
    bool worldToMap(double wx_cm, double wy_cm, int &mx, int &my) const;
    void markCellFree(int mx, int my);
    void markCellOccupied(int mx, int my);
    void raytraceFreeCells(int x0, int y0, int x1, int y1);
    void updateMapFromLidar(const std::vector<LaserData> &laserData);

    static inline double deg2rad(double d)
    {
        return d * (kPi / 180.0);
    }

    static inline double rad2deg(double r)
    {
        return r * (180.0 / kPi);
    }

    static inline double normalizeAngleDeg(double a)
    {
        while(a > 180.0) a -= 360.0;
        while(a < -180.0) a += 360.0;
        return a;
    }

    double computeAvoidanceDirection(double goalDirRad,
                                     double &frontMinCm,
                                     bool &haveCandidate);

    bool canGoDirectlyToGoal(double goalDirRad, double goalDistCm);

    double kpDist = 6.0;
    double kpAng  = 1.6;

    double vMax = 350.0;         // [mm/s]
    double wMax = (kPi / 2.0);   // [rad/s]

    double posDeadbandCm = 5.0;
    double rotateOnlyRad = (45.0 * kPi / 180.0);

    static inline double clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    static constexpr double kPi = 3.14159265358979323846;

    static inline std::int16_t ticksDiff(std::uint16_t now, std::uint16_t prev)
    {
        const std::uint16_t delta = static_cast<std::uint16_t>(now - prev);
        return static_cast<std::int16_t>(delta);
    }

    static inline double normalizeAngleRad(double a)
    {
        while(a > kPi) a -= 2.0 * kPi;
        while(a < -kPi) a += 2.0 * kPi;
        return a;
    }

    static inline double stepTowards(double cur, double target, double maxStep)
    {
        const double diff = target - cur;
        if(std::fabs(diff) <= maxStep) return target;
        return cur + (diff > 0.0 ? maxStep : -maxStep);
    }

    static inline double gyroRawToRad(double gyroRaw)
    {
        const double deg = gyroRaw / 100.0;
        return deg * (kPi / 180.0);
    }

    // --- RIADENIE / CALLBACKY ---
    double forwardspeed;   // [mm/s]
    double rotationspeed;  // [rad/s]

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
