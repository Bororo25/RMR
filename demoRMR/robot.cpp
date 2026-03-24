#include "robot.h"
//polomer robota 15cm
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

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    x = 0.0;
    y = 0.0;
    fi = 0.0;

    odomInitialized = false;
    lastEncL = 0;
    lastEncR = 0;
    gyroOffsetRad = 0.0;

    curForwCmd = 0.0;
    curRotCmd  = 0.0;

    prevBlocked.assign(vfhSectorCount, false);
    prevBlockedInitialized = true;

    prevChosenDirRad = 0.0;
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        latestLidar.clear();
    }

    lastRampTs = std::chrono::steady_clock::now();
    datacounter = 0;
    useDirectCommands = 0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    setSpeedVal(forw, rots);
}
void robot::startPoseControl(double gx_cm, double gy_cm)
{
    //dostane cielove suradnice
    std::lock_guard<std::mutex> lk(controlMtx);
    goalX_cm = gx_cm;
    goalY_cm = gy_cm;
    poseControlActive = true;
    useDirectCommands = 0;
    prevChosenDirRad = 0.0;
}
//zastavenie
void robot::stopPoseControl()
{
    std::lock_guard<std::mutex> lk(controlMtx);
    poseControlActive = false;
    forwardspeed = 0.0;
    rotationspeed = 0.0;
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

        const double distCm = static_cast<double>(ld.scanDistance) / 10.0; // mm -> cm
        if(distCm < 1.0 || distCm > histogramRangeCm) //ignorovanie prilis blizkych alebo vzdialenych bodov
            continue;

        // laser je ľavotočivý
        double angDeg = normalizeAngleDeg(-static_cast<double>(ld.scanAngle));

        if(angDeg < vfhMinAngleDeg || angDeg > vfhMaxAngleDeg)
            continue;

        //sledovanie minimalnej prekazky pred robotom - uvidime ci nechame
        if(std::fabs(angDeg) <= 15.0)
            frontMinCm = std::min(frontMinCm, distCm);

        const double safeRadius = robotRadiusCm + safetyMarginCm; //15+7
        const double ratio = clamp(safeRadius / std::max(distCm, safeRadius + 1.0), 0.0, 1.0); //pomer do asin
        const double enlargeDeg = rad2deg(std::asin(ratio));

        const double mag = histogramRangeCm - distCm;

        //interval prekazky z 6 strany
        const int s0 = angleToSector(angDeg - enlargeDeg);
        const int s1 = angleToSector(angDeg + enlargeDeg);
        //zapis do histogramu
        for(int s = s0; s <= s1; ++s)
            hist[s] = std::max(hist[s], mag);
    }
    // podmienky 8 strana
    if(!prevBlockedInitialized || static_cast<int>(prevBlocked.size()) != vfhSectorCount)
    {
        prevBlocked.assign(vfhSectorCount, false);
        prevBlockedInitialized = true;
    }

    for(int s = 0; s < vfhSectorCount; ++s)
    {
        if(hist[s] >= histHigh)
        {
            blocked[s] = true;
        }
        else if(hist[s] <= histLow)
        {
            blocked[s] = false;
        }
        else
        {
            blocked[s] = prevBlocked[s];
        }
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
        while(i < vfhSectorCount && blocked[i]) ++i; //zacne pri neblokovanom
        if(i >= vfhSectorCount) break;

        const int start = i;
        while(i < vfhSectorCount && !blocked[i]) ++i; //konci na blokovanom
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
        const double leftDeg  = sectorCenterDeg(g.a); //lavy okraj(uhol)
        const double rightDeg = sectorCenterDeg(g.b); //pravy okraj(uhol)
        const double widthDeg = (g.b - g.a + 1) * sectorWidthDeg; //sirka priechodu

        const bool goalInside = (goalDeg >= leftDeg && goalDeg <= rightDeg);
        if(goalInside)
            candidatesDeg.push_back(goalDeg);

        if(widthDeg < wideGapDeg) //ak uzky priechod (<24)
        {
            candidatesDeg.push_back(0.5 * (leftDeg + rightDeg)); //stred
        }
        else //na kraje str.12
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

        const double cost = 1.0 * goalErr + 0.3 * centerErr + 0.2 * smoothErr; //rovnica 13.str

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
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{


    ///tu mozete robit s datami z robota
    //ODOMETRIA
    const double gyroRadAbs = gyroRawToRad(static_cast<double>(robotdata.GyroAngle));

    if(!odomInitialized)
    {
        lastEncL = static_cast<std::uint16_t>(robotdata.EncoderLeft);
        lastEncR = static_cast<std::uint16_t>(robotdata.EncoderRight);

        gyroOffsetRad = gyroRadAbs; // fi=0 pri štarte
        fi = 0.0;

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

        const double tickToMeter = static_cast<double>(robotCom.getTickToMeter()); // [m/tick]
        const double dL = static_cast<double>(dTicksL) * tickToMeter; // [m]
        const double dR = static_cast<double>(dTicksR) * tickToMeter; // [m]

        // krok v centimetroch
        const double l_cm = 0.5 * (dL + dR) * 100.0;
        //vzorce 10. str
        x += l_cm * std::cos(fi);
        y += l_cm * std::sin(fi);

        fi = newFi;
    }



///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        const double fiDeg = fi * 180.0 / kPi;
        emit publishPosition(x, y, fiDeg);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        double desForw = forwardspeed;   // [mm/s]
        double desRot  = rotationspeed;  // [rad/s]
        // ZDRUZENÝ REGULÁTOR POLOHY
        bool active;
        double gx, gy;
        {
            std::lock_guard<std::mutex> lk(controlMtx);
            active = poseControlActive;
            gx = goalX_cm;
            gy = goalY_cm;
        }

        if(active)
        {
            //rozdiely
            const double dx = gx - x;
            const double dy = gy - y;
            const double rho = std::sqrt(dx*dx + dy*dy); // [cm]

            if(rho <= posDeadbandCm)
            {
                desForw = 0.0;
                desRot  = 0.0;
                std::lock_guard<std::mutex> lk(controlMtx);
                poseControlActive = false;
            }
            else
            {
                //smerovanie
                const double heading = std::atan2(dy, dx);
                //chyba uhlova
                const double alphaGoal = normalizeAngleRad(heading - fi);

                double frontMinCm = std::numeric_limits<double>::infinity();
                bool haveCandidate = false;

                double alphaCmd = alphaGoal;
                if(avoidanceEnabled)
                    alphaCmd = computeAvoidanceDirection(alphaGoal, frontMinCm, haveCandidate);

                // ak v predu prekazka tak zastav
                if(frontMinCm < frontStopCm && std::fabs(alphaCmd) < deg2rad(20.0))
                {
                    desForw = 0.0;
                    desRot  = clamp(kpAng * alphaCmd, -wMax, wMax);
                }
                else if(std::fabs(alphaCmd) > rotateOnlyRad)
                {
                    desForw = 0.0;
                    desRot  = clamp(kpAng * alphaCmd, -wMax, wMax);
                }
                else
                {
                    double steerScale = clamp(1.0 - std::fabs(alphaCmd) / deg2rad(90.0), 0.20, 1.0);
                    double frontScale = 1.0;

                    if(frontMinCm < obstacleSlowBandCm)
                    {
                        frontScale = clamp((frontMinCm - frontStopCm) /
                                               (obstacleSlowBandCm - frontStopCm),
                                           0.0, 1.0);
                    }

                    double rotScale = 1.0;
                    if(frontMinCm < obstacleSlowBandCm)
                        rotScale = 0.5;

                    desRot = clamp(kpAng * alphaCmd, -wMax * rotScale, wMax * rotScale);

                    desForw = clamp(kpDist * rho * steerScale * frontScale, 0.0, vMax);
                }
            }
        }
        // deadband
        const double forwDeadband = 5.0;
        const double rotDeadband  = 0.05;
        if (std::fabs(desForw) <= forwDeadband) desForw = 0.0;
        if (std::fabs(desRot)  <= rotDeadband)  desRot  = 0.0;

        if(!active && desForw != 0.0 && desRot != 0.0)
            desForw = 0.0;

        // cas pre rampu
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastRampTs).count();
        lastRampTs = now;

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

        // Pošli do robota
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
            double radius = curForwCmd / curRotCmd; // mm
            radius = clamp(radius, -100000.0, 100000.0); // bezpečný limit
            robotCom.setArcSpeed(static_cast<int>(curForwCmd), static_cast<int>(radius));
        }
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        latestLidar = laserData;
    }

    copyOfLaserData = laserData;
    emit publishLidar(copyOfLaserData);

    return 0;
}

  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
