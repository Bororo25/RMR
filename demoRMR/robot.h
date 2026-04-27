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

//uolha3
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
class robot : public QObject {
  Q_OBJECT
public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);

  //funkcie na polohovanie
  void startPoseControl(double goalX_cm, double goalY_cm);
  void stopPoseControl();

  //uloha3
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
  /// toto su vase premenne na vasu odometriu

  double x;  // [cm]
  double y;  // [cm]
  double fi; // [rad]
  // --- ODOMETRIA  ---
  bool odomInitialized = false;
  std::uint16_t lastEncL = 0;
  std::uint16_t lastEncR = 0;
  double gyroOffsetRad = 0.0;

  // --- RAMPA ---
  double curForwCmd = 0.0; // [mm/s]
  double curRotCmd  = 0.0; // [rad/s]

  // akcelerácie
  double maxAccelForw = 250; // [mm/s^2]
  double maxAccelRot  = 2.5;   // [rad/s^2]


  double minTurnRadiusCm = 50.0;     // minimálny polomer oblúka robota 15
  double maskMarginCm    = 5.0;      // malá rezerva navyše

  // čas poslednej rampy
  std::uint32_t lastRobotTimestampUs = 0;
  bool rampTimestampInitialized = false;
  // --- POLOHOVANIE (združený regulátor) ---
  std::mutex controlMtx;
  bool poseControlActive = false;
  double goalX_cm = 0.0;
  double goalY_cm = 0.0;

  std::mutex lidarMtx;
  std::vector<LaserData> latestLidar;

  bool avoidanceEnabled = true;

  int vfhSectorCount = 90;              // 90 sektorov => 2° na sektor
  double vfhMinAngleDeg = -180.0;
  double vfhMaxAngleDeg =  180.0;

  double histogramRangeCm = 180.0;
  double robotRadiusCm    = 15.0;
  double safetyMarginCm   = 10;
  double frontStopCm      = 24.0; //30.0 20.0

  double wideGapDeg       = 20.0;       // od tejto šírky ber medzeru ako "širokú" 30 20
  double edgeOffsetDeg    = 5.00;       // kandidát vo vnútri kraja priechodu

  double histLow  = 12;   // dolný prah hysterézie //120
  double histHigh = 18;   // horný prah hysterézie  //140

  double prevFiRad = 0.0;
  bool prevFiInitialized = false;

  std::vector<bool> prevBlocked;
  bool prevBlockedInitialized = false;     // prah blokovaného sektora
  double obstacleSlowBandCm = 200.0;     // pri blízkej prekážke spomaľuj 80.0 50.0

  double prevChosenDirRad = 0.0;        // kvôli hladšiemu výberu kandidáta

  //uloha3
  // --- MAPOVANIE / SYNCHRONIZÁCIA ---
  std::mutex poseMtx;

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

  double mapResolutionCm = 10.0;
  int mapWidthCells  = 280;
  int mapHeightCells = 280;
  int mapOriginCellX = mapWidthCells / 2;
  int mapOriginCellY = mapHeightCells / 2;

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
      while (a > 180.0) a -= 360.0;
      while (a < -180.0) a += 360.0;
      return a;
  }

  double computeAvoidanceDirection(double goalDirRad,
                                   double &frontMinCm,
                                   bool &haveCandidate);
  bool canGoDirectlyToGoal(double goalDirRad, double goalDistCm);


  double kpDist = 6;
  double kpAng  = 1.3; //1.6

  double vMax = 250.0;
  double posDeadbandCm = 5.0;                  // [mm/s]
  double wMax = (kPi/2);                // [rad/s]

  double rotateOnlyRad = (45.0 * kPi/180.0);

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
      while (a > kPi) a -= 2.0 * kPi;
      while (a < -kPi) a += 2.0 * kPi;
      return a;
  }

//pohyb dopredu o maxstep
  static inline double stepTowards(double cur, double target, double maxStep)
  {
      const double diff = target - cur;
      if (std::fabs(diff) <= maxStep) return target;
      return cur + (diff > 0 ? maxStep : -maxStep);
  }

  static inline double gyroRawToRad(double gyroRaw)
  {
      const double deg = gyroRaw / 100.0;
      return deg * (kPi / 180.0);
  }


  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;
};

#endif // ROBOT_H
