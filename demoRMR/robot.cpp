#include "robot.h"

// polomer robota 15cm
robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
#ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
#endif
}

double robot::interpAngle(double a0, double a1, double t)
{
    double d = normalizeAngleRad(a1 - a0);
    return normalizeAngleRad(a0 + t * d);
}

bool robot::interpolatePose(std::uint32_t ts_us, double &ix, double &iy, double &ifi)
{
    std::lock_guard<std::mutex> lk(poseHistoryMtx);

    if(poseHistory.size() < 2)
        return false;

    if(ts_us <= poseHistory.front().ts_us)
        return false;

    if(ts_us >= poseHistory.back().ts_us)
        return false;

    for(size_t i = 1; i < poseHistory.size(); ++i)
    {
        const auto &p0 = poseHistory[i - 1];
        const auto &p1 = poseHistory[i];

        if(ts_us >= p0.ts_us && ts_us <= p1.ts_us)
        {
            const double dt = static_cast<double>(p1.ts_us - p0.ts_us);
            if(dt <= 0.0)
                return false;

            const double alpha = static_cast<double>(ts_us - p0.ts_us) / dt;

            ix  = p0.x_cm + alpha * (p1.x_cm - p0.x_cm);
            iy  = p0.y_cm + alpha * (p1.y_cm - p0.y_cm);
            ifi = interpAngle(p0.fi_rad, p1.fi_rad, alpha);
            return true;
        }
    }

    return false;
}

void robot::initOccupancyGrid()
{
    std::lock_guard<std::mutex> lk(mapMtx);

    occupancyGrid.assign(
        mapHeightCells,
        std::vector<int8_t>(mapWidthCells, -1)
        );

    hitGrid.assign(
        mapHeightCells,
        std::vector<uint16_t>(mapWidthCells, 0)
        );

    freeGrid.assign(
        mapHeightCells,
        std::vector<uint16_t>(mapWidthCells, 0)
        );
}

std::vector<std::vector<int8_t>> robot::getOccupancyGrid()
{
    std::lock_guard<std::mutex> lk(mapMtx);
    return occupancyGrid;
}

bool robot::worldToMap(double wx_cm, double wy_cm, int &mx, int &my) const
{
    mx = static_cast<int>(std::round(wx_cm / mapResolutionCm)) + mapOriginCellX;
    my = mapOriginCellY - static_cast<int>(std::round(wy_cm / mapResolutionCm));

    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return false;

    return true;
}

void robot::markCellFree(int mx, int my)
{
    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return;

    freeGrid[my][mx] = std::min<uint16_t>(
        static_cast<uint16_t>(freeGrid[my][mx] + 1),
        static_cast<uint16_t>(1000));

    if(freeGrid[my][mx] > hitGrid[my][mx] + 2)
        occupancyGrid[my][mx] = 0;
}

void robot::markCellOccupied(int mx, int my)
{
    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return;

    hitGrid[my][mx] = std::min<uint16_t>(
        static_cast<uint16_t>(hitGrid[my][mx] + 1),
        static_cast<uint16_t>(1000));

    int neededHits = 3;

    if(std::fabs(currentOmegaRad) > 0.6)
        neededHits = 5;

    if(std::fabs(currentOmegaRad) > 1.0)
        neededHits = 10000;

    if(hitGrid[my][mx] >= neededHits && hitGrid[my][mx] > freeGrid[my][mx] + 1)
        occupancyGrid[my][mx] = 100;
}

void robot::raytraceFreeCells(int x0, int y0, int x1, int y1)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while(true)
    {
        if(x == x1 && y == y1)
            break;

        markCellFree(x, y);

        int e2 = 2 * err;
        if(e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if(e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

void robot::updateMapFromLidar(const std::vector<LaserData> &laserData)
{
    std::lock_guard<std::mutex> lk(mapMtx);

    for(const auto &ld : laserData)
    {
        const double distCm = static_cast<double>(ld.scanDistance) / 10.0; // mm -> cm

        if(distCm < 10.0 || distCm > 250.0)
            continue;

        double rx, ry, rfi;
        if(!interpolatePose(ld.timestamp, rx, ry, rfi))
            continue;

        int robotMx, robotMy;
        if(!worldToMap(rx, ry, robotMx, robotMy))
            continue;

        const double localAngleRad = deg2rad(-static_cast<double>(ld.scanAngle)); // lidar je ľavotočivý
        const double beamAngle = rfi + localAngleRad;

        const double hitX = rx + distCm * std::cos(beamAngle);
        const double hitY = ry + distCm * std::sin(beamAngle);

        int hitMx, hitMy;
        if(!worldToMap(hitX, hitY, hitMx, hitMy))
            continue;

        raytraceFreeCells(robotMx, robotMy, hitMx, hitMy);
        markCellOccupied(hitMx, hitMy);
    }
}

void robot::initAndStartRobot(std::string ipaddress)
{
    forwardspeed = 0;
    rotationspeed = 0;

    x = 0.0;
    y = 0.0;
    fi = 0.0;

    odomInitialized = false;
    lastEncL = 0;
    lastEncR = 0;
    gyroOffsetRad = 0.0;

    curForwCmd = 0.0;
    curRotCmd  = 0.0;
    currentOmegaRad = 0.0;

    lastRobotTimestampUs = 0;
    rampTimestampInitialized = false;

    prevBlocked.assign(vfhSectorCount, false);
    prevBlockedInitialized = true;
    prevChosenDirRad = 0.0;

    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        latestLidar.clear();
    }

    {
        std::lock_guard<std::mutex> lk(poseHistoryMtx);
        poseHistory.clear();
    }

    initOccupancyGrid();

    datacounter = 0;
    useDirectCommands = 0;

    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int { return processThisLidar(dat); }, ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int { return processThisRobot(dat); }, ipaddress);

#ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera, this, std::placeholders::_1),
                                 "http://" + ipaddress + ":8000/stream.mjpg");
#endif
#ifndef DISABLE_SKELETON
    robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton, this, std::placeholders::_1));
#endif

    robotCom.robotStart();
}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed = forw;
    rotationspeed = rots;
    useDirectCommands = 0;
}

void robot::setSpeed(double forw, double rots)
{
    setSpeedVal(forw, rots);
}

void robot::startPoseControl(double gx_cm, double gy_cm)
{
    std::lock_guard<std::mutex> lk(controlMtx);
    goalX_cm = gx_cm;
    goalY_cm = gy_cm;
    poseControlActive = true;
    useDirectCommands = 0;
    prevChosenDirRad = 0.0;
}

void robot::stopPoseControl()
{
    std::lock_guard<std::mutex> lk(controlMtx);
    poseControlActive = false;
    forwardspeed = 0.0;
    rotationspeed = 0.0;
}

bool robot::canGoDirectlyToGoal(double goalDirRad, double goalDistCm)
{
    std::vector<LaserData> lidar;
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        lidar = latestLidar;
    }

    if(lidar.empty())
        return false;

    const double corridorHalfWidthCm = robotRadiusCm + safetyMarginCm;

    for(const auto &ld : lidar)
    {
        const double distCm = static_cast<double>(ld.scanDistance) / 10.0;
        if(distCm < 1.0)
            continue;

        double angRad = deg2rad(normalizeAngleDeg(-static_cast<double>(ld.scanAngle)));

        const double px = distCm * std::cos(angRad);
        const double py = distCm * std::sin(angRad);

        const double gx = std::cos(goalDirRad);
        const double gy = std::sin(goalDirRad);

        const double along = px * gx + py * gy;
        const double perp  = std::fabs(-gy * px + gx * py);

        if(along >= 0.0 && along <= goalDistCm && perp <= corridorHalfWidthCm)
            return false;
    }

    return true;
}

double robot::computeAvoidanceDirection(double goalDirRad,
                                        double &frontMinCm,
                                        bool &haveCandidate)
{
    std::vector<LaserData> lidar;
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        lidar = latestLidar;
    }

    frontMinCm = std::numeric_limits<double>::infinity();
    haveCandidate = false;

    if(lidar.empty())
        return goalDirRad;

    const double sectorWidthDeg =
        (vfhMaxAngleDeg - vfhMinAngleDeg) / static_cast<double>(vfhSectorCount);

    std::vector<double> hist(vfhSectorCount, 0.0);
    std::vector<bool> blocked(vfhSectorCount, false);
    std::vector<bool> maskBlocked(vfhSectorCount, false);

    auto angleToSector = [&](double angDeg) -> int
    {
        int idx = static_cast<int>((angDeg - vfhMinAngleDeg) / sectorWidthDeg);
        if(idx < 0) idx = 0;
        if(idx >= vfhSectorCount) idx = vfhSectorCount - 1;
        return idx;
    };

    auto sectorCenterDeg = [&](int idx) -> double
    {
        return vfhMinAngleDeg + (static_cast<double>(idx) + 0.5) * sectorWidthDeg;
    };

    for(const auto &ld : lidar)
    {
        const double distCm = static_cast<double>(ld.scanDistance) / 10.0;
        if(distCm < 1.0 || distCm > histogramRangeCm)
            continue;

        double angDeg = normalizeAngleDeg(-static_cast<double>(ld.scanAngle));
        if(angDeg < vfhMinAngleDeg || angDeg > vfhMaxAngleDeg)
            continue;

        if(std::fabs(angDeg) <= 15.0)
            frontMinCm = std::min(frontMinCm, distCm);

        const double safeRadius = robotRadiusCm + safetyMarginCm;
        const double ratio = clamp(safeRadius / std::max(distCm, safeRadius + 1.0), 0.0, 1.0);
        const double enlargeDeg = rad2deg(std::asin(ratio));

        const double mag = histogramRangeCm - distCm;

        const int s0 = angleToSector(angDeg - enlargeDeg);
        const int s1 = angleToSector(angDeg + enlargeDeg);

        for(int s = s0; s <= s1; ++s)
            hist[s] = std::max(hist[s], mag);

        const double angRad = deg2rad(angDeg);

        const double px = distCm * std::cos(angRad);   // dopredu +
        const double py = distCm * std::sin(angRad);   // vlavo +

        const double Rmask = minTurnRadiusCm;
        const double safeMaskRadius = robotRadiusCm + safetyMarginCm + maskMarginCm;

        const double dLeft  = std::sqrt(px*px + (py - Rmask)*(py - Rmask));
        const double dRight = std::sqrt(px*px + (py + Rmask)*(py + Rmask));

        if(py >= 0.0 && dLeft <= (Rmask + safeMaskRadius))
        {
            for(int s = s0; s <= s1; ++s)
            {
                const double cdeg = sectorCenterDeg(s);
                if(cdeg >= 0.0)
                    maskBlocked[s] = true;
            }
        }

        if(py <= 0.0 && dRight <= (Rmask + safeMaskRadius))
        {
            for(int s = s0; s <= s1; ++s)
            {
                const double cdeg = sectorCenterDeg(s);
                if(cdeg <= 0.0)
                    maskBlocked[s] = true;
            }
        }
    }

    if(!prevBlockedInitialized || static_cast<int>(prevBlocked.size()) != vfhSectorCount)
    {
        prevBlocked.assign(vfhSectorCount, false);
        prevBlockedInitialized = true;
    }

    for(int s = 0; s < vfhSectorCount; ++s)
    {
        if(hist[s] >= histHigh)
            blocked[s] = true;
        else if(hist[s] <= histLow)
            blocked[s] = false;
        else
            blocked[s] = prevBlocked[s];
    }

    for(int s = 0; s < vfhSectorCount; ++s)
    {
        if(maskBlocked[s])
            blocked[s] = true;
    }

    prevBlocked = blocked;
    const double goalDeg = rad2deg(goalDirRad);

    struct Gap
    {
        int a;
        int b;
    };
    std::vector<Gap> gaps;

    int i = 0;
    while(i < vfhSectorCount)
    {
        while(i < vfhSectorCount && blocked[i]) ++i;
        if(i >= vfhSectorCount) break;

        const int start = i;
        while(i < vfhSectorCount && !blocked[i]) ++i;
        const int end = i - 1;

        gaps.push_back({start, end});
    }

    if(gaps.empty())
    {
        double leftSum = 0.0;
        double rightSum = 0.0;

        for(int s = 0; s < vfhSectorCount; ++s)
        {
            const double c = sectorCenterDeg(s);
            if(c >= 0.0) leftSum += hist[s];
            else         rightSum += hist[s];
        }

        haveCandidate = true;
        prevChosenDirRad = (leftSum < rightSum) ? deg2rad(60.0) : deg2rad(-60.0);
        return prevChosenDirRad;
    }

    std::vector<double> candidatesDeg;

    for(const auto &g : gaps)
    {
        const double leftDeg  = sectorCenterDeg(g.a);
        const double rightDeg = sectorCenterDeg(g.b);
        const double widthDeg = (g.b - g.a + 1) * sectorWidthDeg;

        const bool goalInside = (goalDeg >= leftDeg && goalDeg <= rightDeg);
        if(goalInside)
            candidatesDeg.push_back(goalDeg);

        if(widthDeg < wideGapDeg)
        {
            candidatesDeg.push_back(0.5 * (leftDeg + rightDeg));
        }
        else
        {
            candidatesDeg.push_back(leftDeg  + edgeOffsetDeg);
            candidatesDeg.push_back(rightDeg - edgeOffsetDeg);
        }
    }

    if(candidatesDeg.empty())
        candidatesDeg.push_back(goalDeg);

    const double prevDeg = rad2deg(prevChosenDirRad);

    double bestDeg = candidatesDeg.front();
    double bestCost = std::numeric_limits<double>::infinity();

    for(double cand : candidatesDeg)
    {
        cand = clamp(cand, vfhMinAngleDeg, vfhMaxAngleDeg);

        const double goalErr   = std::fabs(normalizeAngleDeg(cand - goalDeg));
        const double smoothErr = std::fabs(normalizeAngleDeg(cand - prevDeg));
        const double centerErr = std::fabs(cand);

        const double cost = 1.0 * goalErr + 0.3 * centerErr + 0.3 * smoothErr;

        if(cost < bestCost)
        {
            bestCost = cost;
            bestDeg = cand;
        }
    }

    haveCandidate = true;
    prevChosenDirRad = deg2rad(bestDeg);
    return prevChosenDirRad;
}

int robot::processThisRobot(const TKobukiData &robotdata)
{
    static std::uint32_t prevTsOmega = 0;
    static double prevFiOmega = 0.0;

    const double gyroRadAbs = gyroRawToRad(static_cast<double>(robotdata.GyroAngle));

    if(!odomInitialized)
    {
        lastEncL = static_cast<std::uint16_t>(robotdata.EncoderLeft);
        lastEncR = static_cast<std::uint16_t>(robotdata.EncoderRight);

        gyroOffsetRad = gyroRadAbs;

        {
            std::lock_guard<std::mutex> lk(poseMtx);
            x = 0.0;
            y = 0.0;
            fi = 0.0;
        }

        prevTsOmega = robotdata.synctimestamp;
        prevFiOmega = 0.0;
        odomInitialized = true;
    }
    else
    {
        const double newFi = normalizeAngleRad(gyroRadAbs - gyroOffsetRad);

        const std::uint16_t encL = static_cast<std::uint16_t>(robotdata.EncoderLeft);
        const std::uint16_t encR = static_cast<std::uint16_t>(robotdata.EncoderRight);

        const std::int16_t dTicksL = ticksDiff(encL, lastEncL);
        const std::int16_t dTicksR = ticksDiff(encR, lastEncR);

        lastEncL = encL;
        lastEncR = encR;

        const double tickToMeter = static_cast<double>(robotCom.getTickToMeter());
        const double dL = static_cast<double>(dTicksL) * tickToMeter;
        const double dR = static_cast<double>(dTicksR) * tickToMeter;

        const double l_cm = 0.5 * (dL + dR) * 100.0;

        {
            std::lock_guard<std::mutex> lk(poseMtx);
            const double oldFi = fi;
            const double fiMid = normalizeAngleRad(oldFi + 0.5 * normalizeAngleRad(newFi - oldFi));

            x += l_cm * std::cos(fiMid);
            y += l_cm * std::sin(fiMid);
            fi = newFi;
        }
    }

    {
        double px, py, pfi;
        {
            std::lock_guard<std::mutex> lk(poseMtx);
            px = x;
            py = y;
            pfi = fi;
        }

        if(prevTsOmega != 0 && robotdata.synctimestamp > prevTsOmega)
        {
            const double dt = static_cast<double>(robotdata.synctimestamp - prevTsOmega) / 1e6;
            if(dt > 0.0)
                currentOmegaRad = normalizeAngleRad(pfi - prevFiOmega) / dt;
        }

        prevTsOmega = robotdata.synctimestamp;
        prevFiOmega = pfi;

        std::lock_guard<std::mutex> lk(poseHistoryMtx);
        TimedPose tp;
        tp.ts_us  = robotdata.synctimestamp;
        tp.x_cm   = px;
        tp.y_cm   = py;
        tp.fi_rad = pfi;
        poseHistory.push_back(tp);

        while(poseHistory.size() > 4000)
            poseHistory.pop_front();
    }

    if(datacounter % 5 == 0)
    {
        double px, py, pfi;
        {
            std::lock_guard<std::mutex> lk(poseMtx);
            px = x;
            py = y;
            pfi = fi;
        }

        const double fiDeg = pfi * 180.0 / kPi;
        emit publishPosition(px, py, fiDeg);
    }

    if(useDirectCommands == 0)
    {
        double desForw = forwardspeed;
        double desRot  = rotationspeed;

        bool active;
        double gx, gy;
        {
            std::lock_guard<std::mutex> lk(controlMtx);
            active = poseControlActive;
            gx = goalX_cm;
            gy = goalY_cm;
        }

        double px, py, pfi;
        {
            std::lock_guard<std::mutex> lk(poseMtx);
            px = x;
            py = y;
            pfi = fi;
        }

        if(active)
        {
            const double dx = gx - px;
            const double dy = gy - py;
            const double rho = std::sqrt(dx*dx + dy*dy);

            if(rho <= posDeadbandCm)
            {
                desForw = 0.0;
                desRot  = 0.0;
                curForwCmd = 0.0;
                curRotCmd  = 0.0;

                std::lock_guard<std::mutex> lk(controlMtx);
                poseControlActive = false;
            }
            else
            {
                const double heading = std::atan2(dy, dx);
                const double alphaGoal = normalizeAngleRad(heading - pfi);

                double frontMinCm = std::numeric_limits<double>::infinity();
                bool haveCandidate = false;

                double alphaAvoid = alphaGoal;

                const double directGoalThresholdCm = 35.0;
                const bool directToGoal =
                    (rho < directGoalThresholdCm) && canGoDirectlyToGoal(alphaGoal, rho);

                if(rho < 20.0)
                {
                    alphaAvoid = alphaGoal;
                    frontMinCm = std::numeric_limits<double>::infinity();
                }
                else if(directToGoal)
                {
                    alphaAvoid = alphaGoal;
                    frontMinCm = std::numeric_limits<double>::infinity();
                }
                else if(avoidanceEnabled)
                {
                    alphaAvoid = computeAvoidanceDirection(alphaGoal, frontMinCm, haveCandidate);
                }

                double alphaCmd = alphaAvoid;
                if(rho < 30.0)
                {
                    const double blend = clamp((30.0 - rho) / 20.0, 0.0, 1.0);
                    alphaCmd = normalizeAngleRad((1.0 - blend) * alphaAvoid + blend * alphaGoal);
                }

                const double steerScale = clamp(std::cos(std::fabs(alphaCmd)), 0.0, 1.0);

                double frontScale = 1.0;
                if(frontMinCm < obstacleSlowBandCm)
                {
                    frontScale = clamp((frontMinCm - frontStopCm) /
                                           (obstacleSlowBandCm - frontStopCm),
                                       0.0, 1.0);
                }

                const double goalScale = clamp(rho / 10.0, 0.20, 1.0);

                desForw = clamp(kpDist * rho * steerScale * frontScale * goalScale, 0.0, vMax);
                desRot  = clamp(kpAng * alphaCmd, -wMax * 0.5, wMax * 0.5);  //*0.5

                if(frontMinCm < frontStopCm)
                    desForw = 0.0;

                if(std::fabs(alphaCmd) > rotateOnlyRad)
                    desForw = 0.0;
            }
        }

        const double forwDeadband = 8.0;
        const double rotDeadband  = 0.05;
        if(std::fabs(desForw) <= forwDeadband) desForw = 0.0;
        if(std::fabs(desRot)  <= rotDeadband)  desRot  = 0.0;

        if(!active && desForw != 0.0 && desRot != 0.0)
            desForw = 0.0;

        std::uint32_t nowUs = robotdata.synctimestamp;
        double dt = 0.02;

        if(!rampTimestampInitialized)
        {
            lastRobotTimestampUs = nowUs;
            rampTimestampInitialized = true;
        }
        else
        {
            std::uint32_t dUs = 0;

            if(nowUs >= lastRobotTimestampUs)
                dUs = nowUs - lastRobotTimestampUs;
            else
                dUs = (std::numeric_limits<std::uint32_t>::max() - lastRobotTimestampUs) + nowUs + 1u;

            lastRobotTimestampUs = nowUs;
            dt = static_cast<double>(dUs) * 1e-6;
        }

        dt = clamp(dt, 0.005, 0.120);

        auto accelOnly = [&](double &cur, double des, double maxAccel)
        {
            if(cur != 0.0 && des != 0.0 && ((cur > 0.0) != (des > 0.0)))
            {
                cur = 0.0;
                return;
            }

            if(std::fabs(des) > std::fabs(cur))
            {
                const double step = maxAccel * dt;
                cur = stepTowards(cur, des, step);
            }
            else
            {
                cur = des;
            }
        };

        accelOnly(curForwCmd, desForw, maxAccelForw);
        accelOnly(curRotCmd,  desRot,  maxAccelRot);

        if(curForwCmd == 0.0 && curRotCmd == 0.0)
        {
            robotCom.setTranslationSpeed(0);
        }
        else if(curForwCmd == 0.0)
        {
            robotCom.setRotationSpeed(curRotCmd);
        }
        else if(curRotCmd == 0.0)
        {
            robotCom.setTranslationSpeed(static_cast<int>(curForwCmd));
        }
        else
        {
            double radius = curForwCmd / curRotCmd;
            radius = clamp(radius, -100000.0, 100000.0);
            robotCom.setArcSpeed(static_cast<int>(curForwCmd), static_cast<int>(radius));
        }
    }

    datacounter++;
    return 0;
}

int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        latestLidar = laserData;
    }

    updateMapFromLidar(laserData);

    copyOfLaserData = laserData;
    emit publishLidar(copyOfLaserData);

    return 0;
}

#ifndef DISABLE_OPENCV
int robot::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int robot::processThisSkeleton(skeleton skeledata)
{
    memcpy(&skeleJoints, &skeledata, sizeof(skeleton));
    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
