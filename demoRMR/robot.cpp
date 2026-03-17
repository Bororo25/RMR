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
    // Namiesto priameho posielania to necháme poslať callback (kvôli rampe)
    setSpeedVal(forw, rots);
}
void robot::startPoseControl(double gx_cm, double gy_cm)
{
    std::lock_guard<std::mutex> lk(controlMtx);
    goalX_cm = gx_cm;
    goalY_cm = gy_cm;
    poseControlActive = true;
    useDirectCommands = 0; // nech to riadi callback
}

void robot::stopPoseControl()
{
    std::lock_guard<std::mutex> lk(controlMtx);
    poseControlActive = false;
    forwardspeed = 0.0;
    rotationspeed = 0.0;
}
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{


    ///tu mozete robit s datami z robota
    // --- ODOMETRIA: vzdialenosť z enkóderov, uhol z gyra ---
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
        const double l_cm = 0.5 * (dL + dR) * 100.0; // [cm]

        // x,y budú odteraz v cm
        x += l_cm * std::cos(fi);
        y += l_cm * std::sin(fi);

        // fi berieme z gyra
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
        // --- ZDRUŽENÝ REGULÁTOR POLOHY (ak je aktívny) ---
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
            const double dx = gx - x;     // x,y sú v cm
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
                const double heading = std::atan2(dy, dx);
                const double alpha = normalizeAngleRad(heading - fi);

                if(std::fabs(alpha) > rotateOnlyRad)
                {
                    desForw = 0.0;
                    desRot  = clamp(kpAng * alpha, -wMax, wMax);
                }
                else
                {
                    desForw = clamp(kpDist * rho, 0.0, vMax);       // [mm/s]
                    desRot  = clamp(kpAng  * alpha, -wMax, wMax);    // [rad/s]
                }
            }
        }
        // deadband (jemné potlačenie šumu)
        const double forwDeadband = 5.0;
        const double rotDeadband  = 0.05;
        if (std::fabs(desForw) <= forwDeadband) desForw = 0.0;
        if (std::fabs(desRot)  <= rotDeadband)  desRot  = 0.0;

        // ŽIADNA KRIVKA len v manuáli (polohovanie môže použiť oblúk)
        if(!active && desForw != 0.0 && desRot != 0.0)
            desForw = 0.0;

        // RAMPA podľa času (rozbeh aj brzdenie)
        // RAMPA podľa času – iba ROZBEH, dobeh/brzdenie bez rampy
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastRampTs).count();
        lastRampTs = now;

        // ošetrenie: keď dt vyjde divné (napr. pri štarte alebo lag)
        dt = clamp(dt, 0.005, 0.120);

        auto accelOnly = [&](double &cur, double des, double maxAccel)
        {
            // zmena smeru: najprv okamžite na 0
            if(cur != 0.0 && des != 0.0 && ((cur > 0.0) != (des > 0.0)))
            {
                cur = 0.0;
                return;
            }

            // rampa iba keď zväčšujem absolútnu hodnotu rýchlosti
            if(std::fabs(des) > std::fabs(cur))
            {
                const double step = maxAccel * dt;
                cur = stepTowards(cur, des, step);
            }
            else
            {
                // dobeh / brzdenie bez rampy
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

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


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
