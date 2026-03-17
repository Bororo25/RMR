#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <chrono>

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

  // max. akcelerácie (rozbeh)
  double maxAccelForw = 250; // [mm/s^2]
  double maxAccelRot  = 2.5;   // [rad/s^2]

  // čas poslednej rampy
  std::chrono::steady_clock::time_point lastRampTs;
  // --- POLOHOVANIE (združený regulátor) ---
  std::mutex controlMtx;
  bool poseControlActive = false;
  double goalX_cm = 0.0;
  double goalY_cm = 0.0;

  double kpDist = 7;
  double kpAng  = 3;

  double vMax = 400;                    // [mm/s]
  double wMax = (kPi/2);                // [rad/s]

  double posDeadbandCm = 5.0;
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
