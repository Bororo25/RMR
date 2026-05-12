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

//uloha3
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

    occupancyGrid.assign(mapHeightCells, std::vector<int8_t>(mapWidthCells, -1));
    hitGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));
    freeGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));
}

std::vector<std::vector<int8_t>> robot::getOccupancyGrid()
{
    std::lock_guard<std::mutex> lk(mapMtx);
    return occupancyGrid;
}

void robot::setMappingEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lk(mapMtx);
    mappingEnabled = enabled;
}

bool robot::isMappingEnabled()
{
    std::lock_guard<std::mutex> lk(mapMtx);
    return mappingEnabled;
}

bool robot::saveOccupancyMapTxt(const QString &fileName)
{
    std::lock_guard<std::mutex> lk(mapMtx);

    if(occupancyGrid.empty() || occupancyGrid[0].empty())
        return false;

    std::ofstream file(fileName.toStdString());

    if(!file.is_open())
        return false;

    file << mapWidthCells << " "
         << mapHeightCells << " "
         << mapResolutionCm << " "
         << mapOriginCellX << " "
         << mapOriginCellY << "\n";

    for(int y = 0; y < mapHeightCells; ++y)
    {
        for(int x = 0; x < mapWidthCells; ++x)
        {
            file << static_cast<int>(occupancyGrid[y][x]);

            if(x + 1 < mapWidthCells)
                file << " ";
        }

        file << "\n";
    }

    return true;
}
bool robot::loadOccupancyMapTxt(const QString &fileName)
{
    std::ifstream file(fileName.toStdString());

    if(!file.is_open())
        return false;

    int loadedWidth = 0;
    int loadedHeight = 0;
    double loadedResolution = 0.0;
    int loadedOriginX = 0;
    int loadedOriginY = 0;

    file >> loadedWidth
        >> loadedHeight
        >> loadedResolution
        >> loadedOriginX
        >> loadedOriginY;

    if(!file.good())
        return false;

    if(loadedWidth <= 0 || loadedHeight <= 0 || loadedResolution <= 0.0)
        return false;

    std::vector<std::vector<int8_t>> newGrid(
        loadedHeight,
        std::vector<int8_t>(loadedWidth, -1)
        );

    std::vector<std::vector<uint16_t>> newHitGrid(
        loadedHeight,
        std::vector<uint16_t>(loadedWidth, 0)
        );

    std::vector<std::vector<uint16_t>> newFreeGrid(
        loadedHeight,
        std::vector<uint16_t>(loadedWidth, 0)
        );

    for(int y = 0; y < loadedHeight; ++y)
    {
        for(int x = 0; x < loadedWidth; ++x)
        {
            int value = -1;
            file >> value;

            if(!file.good())
                return false;

            if(value >= 50)
            {
                newGrid[y][x] = 100;
                newHitGrid[y][x] = 10;
            }
            else if(value == 0)
            {
                newGrid[y][x] = 0;
                newFreeGrid[y][x] = 10;
            }
            else
            {
                newGrid[y][x] = -1;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lk(mapMtx);

        mapWidthCells = loadedWidth;
        mapHeightCells = loadedHeight;
        mapResolutionCm = loadedResolution;
        mapOriginCellX = loadedOriginX;
        mapOriginCellY = loadedOriginY;

        occupancyGrid = std::move(newGrid);
        hitGrid = std::move(newHitGrid);
        freeGrid = std::move(newFreeGrid);
    }

    return true;
}


//uloha4
std::vector<std::pair<double, double>> robot::getPlannedPathCm()
{
    std::lock_guard<std::mutex> lk(controlMtx);

    std::vector<std::pair<double, double>> out;
    out.reserve(plannedPathCm.size());

    for(const auto &p : plannedPathCm)
        out.push_back({p.x_cm, p.y_cm});

    return out;
}

bool robot::loadOccupancyGridFromFile(const std::string &filename)
{
    std::lock_guard<std::mutex> lk(mapMtx);

    std::ifstream file(filename);

    if(!file.is_open())
    {
        std::cout << "Nepodarilo sa otvorit subor na citanie: " << filename << std::endl;
        return false;
    }

    int width = 0;
    int height = 0;
    double resolution = 0.0;
    int originX = 0;
    int originY = 0;

    file >> width >> height >> resolution >> originX >> originY;

    if(!file.good())
    {
        std::cout << "Chyba pri citani hlavicky mapy." << std::endl;
        return false;
    }

    if(width <= 0 || height <= 0 || resolution <= 0.0)
    {
        std::cout << "Neplatne parametre mapy." << std::endl;
        return false;
    }

    mapWidthCells = width;
    mapHeightCells = height;
    mapResolutionCm = resolution;
    mapOriginCellX = originX;
    mapOriginCellY = originY;

    occupancyGrid.assign(mapHeightCells, std::vector<int8_t>(mapWidthCells, -1));
    hitGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));
    freeGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));

    for(int y = 0; y < mapHeightCells; ++y)
    {
        for(int x = 0; x < mapWidthCells; ++x)
        {
            int val = -1;
            file >> val;

            if(!file.good())
            {
                std::cout << "Chyba pri citani bunky mapy." << std::endl;
                return false;
            }

            if(val < -1)
                val = -1;

            if(val > 100)
                val = 100;

            occupancyGrid[y][x] = static_cast<int8_t>(val);

            if(val >= 50)
            {
                hitGrid[y][x] = 10;
                freeGrid[y][x] = 0;
            }
            else if(val == 0)
            {
                hitGrid[y][x] = 0;
                freeGrid[y][x] = 10;
            }
            else
            {
                hitGrid[y][x] = 0;
                freeGrid[y][x] = 0;
            }
        }
    }

    std::cout << "OccupancyGrid mapa nacitana zo suboru: " << filename << std::endl;
    return true;
}

//uloha4
void robot::setUseLoadedMapOnly(bool value)
{
    std::lock_guard<std::mutex> lk(mapMtx);
    useLoadedMapOnly = value;
}


//uloha4
bool robot::mapToWorld(int mx, int my, double &wx_cm, double &wy_cm) const
{
    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return false;

    wx_cm = static_cast<double>(mx - mapOriginCellX) * mapResolutionCm;
    wy_cm = static_cast<double>(mapOriginCellY - my) * mapResolutionCm;

    return true;
}

//uloha4
bool robot::makeInflatedObstacleGrid(std::vector<std::vector<uint8_t>> &blocked) const
{
    blocked.assign(mapHeightCells, std::vector<uint8_t>(mapWidthCells, 0));

    if(occupancyGrid.empty() || occupancyGrid[0].empty())
        return false;

    const int inflateCells = static_cast<int>(
        std::ceil((robotRadiusCm + safetyMarginCm) / mapResolutionCm));

    for(int y0 = 0; y0 < mapHeightCells; ++y0)
    {
        for(int x0 = 0; x0 < mapWidthCells; ++x0)
        {
            const int8_t val = occupancyGrid[y0][x0];

            // obsadená bunka
            const bool occupied = (val >= 50);

            // neznáma bunka - podľa prepínača planUnknownAsFree
            const bool unknownBlocked = (val < 0 && !planUnknownAsFree);

            if(!occupied && !unknownBlocked)
                continue;

            for(int dy = -inflateCells; dy <= inflateCells; ++dy)
            {
                for(int dx = -inflateCells; dx <= inflateCells; ++dx)
                {
                    if(dx*dx + dy*dy > inflateCells*inflateCells)
                        continue;

                    const int x1 = x0 + dx;
                    const int y1 = y0 + dy;

                    if(x1 < 0 || x1 >= mapWidthCells || y1 < 0 || y1 >= mapHeightCells)
                        continue;

                    blocked[y1][x1] = 1;
                }
            }
        }
    }

    return true;
}

//uloha4
bool robot::extractWavePath(const std::vector<std::vector<int>> &wave,
                            const GridPoint &start,
                            const GridPoint &goal,
                            std::vector<GridPoint> &rawPath) const
{
    rawPath.clear();

    if(start.x < 0 || start.x >= mapWidthCells || start.y < 0 || start.y >= mapHeightCells)
        return false;

    if(goal.x < 0 || goal.x >= mapWidthCells || goal.y < 0 || goal.y >= mapHeightCells)
        return false;

    if(wave[start.y][start.x] < 0)
        return false;

    GridPoint cur = start;
    rawPath.push_back(cur);

    GridPoint prevDir{0, 0};

    const int maxSteps = mapWidthCells * mapHeightCells;

    for(int step = 0; step < maxSteps; ++step)
    {
        if(cur.x == goal.x && cur.y == goal.y)
            return true;

        const int curVal = wave[cur.y][cur.x];

        // 4-susednosť - posledný smer potom sa preferuje x
        const GridPoint dirs4[4] =
            {
                { 1,  0},
                {-1,  0},
                { 0,  1},
                { 0, -1}
            };

        GridPoint best = cur;
        int bestValue = curVal;

        // 1. Najprv skús pokračovať predchádzajúcim smerom.
        if(prevDir.x != 0 || prevDir.y != 0)
        {
            const int nx = cur.x + prevDir.x;
            const int ny = cur.y + prevDir.y;

            if(nx >= 0 && nx < mapWidthCells && ny >= 0 && ny < mapHeightCells)
            {
                if(wave[ny][nx] >= 0 && wave[ny][nx] < bestValue)
                {
                    bestValue = wave[ny][nx];
                    best = {nx, ny};
                }
            }
        }

        // 2. Ak posledný smer nevedie bližšie k cieľu, vyber nový smer.
        if(best.x == cur.x && best.y == cur.y)
        {
            for(const auto &d : dirs4)
            {
                const int nx = cur.x + d.x;
                const int ny = cur.y + d.y;

                if(nx < 0 || nx >= mapWidthCells || ny < 0 || ny >= mapHeightCells)
                    continue;

                if(wave[ny][nx] >= 0 && wave[ny][nx] < bestValue)
                {
                    bestValue = wave[ny][nx];
                    best = {nx, ny};
                }
            }
        }

        if(best.x == cur.x && best.y == cur.y)
            return false;

        prevDir = {best.x - cur.x, best.y - cur.y};
        cur = best;
        rawPath.push_back(cur);
    }

    return false;
}

//uloha4
std::vector<robot::GridPoint> robot::simplifyPathToCorners(const std::vector<GridPoint> &rawPath) const
{
    std::vector<GridPoint> corners;

    if(rawPath.empty())
        return corners;

    if(rawPath.size() <= 2)
    {
        corners = rawPath;
        return corners;
    }

    corners.push_back(rawPath.front());

    GridPoint prevDir
        {
            rawPath[1].x - rawPath[0].x,
            rawPath[1].y - rawPath[0].y
        };

    for(size_t i = 2; i < rawPath.size(); ++i)
    {
        GridPoint dir
            {
                rawPath[i].x - rawPath[i - 1].x,
                rawPath[i].y - rawPath[i - 1].y
            };

        // zlomový bod
        if(dir.x != prevDir.x || dir.y != prevDir.y)
        {
            corners.push_back(rawPath[i - 1]);
            prevDir = dir;
        }
    }

    corners.push_back(rawPath.back());

    return corners;
}

//uloha4
bool robot::planPathToGoal(double gx_cm, double gy_cm)
{
    GridPoint start;
    GridPoint goal;

    {
        std::lock_guard<std::mutex> lk(mapMtx);

        if(!worldToMap(x, y, start.x, start.y))
            return false;

        if(!worldToMap(gx_cm, gy_cm, goal.x, goal.y))
            return false;

        std::vector<std::vector<uint8_t>> blocked;
        if(!makeInflatedObstacleGrid(blocked))
            return false;

        // štart a cieľ musia zostať priechodné aj po nafúknutí
        blocked[start.y][start.x] = 0;
        blocked[goal.y][goal.x] = 0;

        std::vector<std::vector<int>> wave(
            mapHeightCells,
            std::vector<int>(mapWidthCells, -1));

        std::queue<GridPoint> q;

        wave[goal.y][goal.x] = 0;
        q.push(goal);

        // 4-susednosť
        const GridPoint dirs4[4] =
            {
                { 1,  0},
                {-1,  0},
                { 0,  1},
                { 0, -1}
            };

        while(!q.empty())
        {
            GridPoint cur = q.front();
            q.pop();

            for(const auto &d : dirs4)
            {
                const int nx = cur.x + d.x;
                const int ny = cur.y + d.y;

                if(nx < 0 || nx >= mapWidthCells || ny < 0 || ny >= mapHeightCells)
                    continue;

                if(blocked[ny][nx])
                    continue;

                if(wave[ny][nx] >= 0)
                    continue;

                wave[ny][nx] = wave[cur.y][cur.x] + 1;
                q.push({nx, ny});
            }
        }

        if(wave[start.y][start.x] < 0)
            return false;

        std::vector<GridPoint> rawPath;
        if(!extractWavePath(wave, start, goal, rawPath))
            return false;

        std::vector<GridPoint> corners = simplifyPathToCorners(rawPath);

        plannedPathCm.clear();

        // prvý bod je aktuálna bunka robota, ten netreba posielať ako cieľ
        for(size_t i = 1; i < corners.size(); ++i)
        {
            double wx, wy;
            if(mapToWorld(corners[i].x, corners[i].y, wx, wy))
                plannedPathCm.push_back({wx, wy});
        }

        if(!plannedPathCm.empty())
        {
            plannedPathCm.back().x_cm = gx_cm;
            plannedPathCm.back().y_cm = gy_cm;
        }
    }

    if(plannedPathCm.empty())
        return false;

    plannedPathIndex = 0;
    followingPlannedPath = true;

    {
        std::lock_guard<std::mutex> lk(controlMtx);
        goalX_cm = plannedPathCm[plannedPathIndex].x_cm;
        goalY_cm = plannedPathCm[plannedPathIndex].y_cm;
        poseControlActive = true;
        useDirectCommands = 0;
    }

    return true;
}

//uloha4
void robot::startNextPlannedWaypoint()
{
    if(!followingPlannedPath)
        return;

    ++plannedPathIndex;

    if(plannedPathIndex >= static_cast<int>(plannedPathCm.size()))
    {
        followingPlannedPath = false;
        return;
    }

    std::lock_guard<std::mutex> lk(controlMtx);
    goalX_cm = plannedPathCm[plannedPathIndex].x_cm;
    goalY_cm = plannedPathCm[plannedPathIndex].y_cm;
    poseControlActive = true;
    useDirectCommands = 0;

    //uloha4
    curForwCmd = 0.0;
    curRotCmd  = 0.0;
    prevChosenDirRad = 0.0;

    prevBlocked.assign(vfhSectorCount, false);
    prevBlockedInitialized = true;

    prevFiRad = fi;
    prevFiInitialized = true;
}

bool robot::worldToMap(double wx_cm, double wy_cm, int &mx, int &my) const
{
    mx = static_cast<int>(std::round(wx_cm / mapResolutionCm)) + mapOriginCellX;
    my = mapOriginCellY - static_cast<int>(std::round(wy_cm / mapResolutionCm));

    return !(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells);
}

void robot::markCellFree(int mx, int my)
{
    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return;

    freeGrid[my][mx] = std::min<uint16_t>(freeGrid[my][mx] + 1, 1000);

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

    int neededHits = 4;

    if(std::fabs(currentOmegaRad) > 0.2)
        neededHits = 8;

    if(std::fabs(currentOmegaRad) > 0.4)
        return;

    if(hitGrid[my][mx] >= neededHits && hitGrid[my][mx] > freeGrid[my][mx] + 4)
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
        const double distCm = static_cast<double>(ld.scanDistance) / 10.0;

        if(distCm < 20.0 || distCm > 200.0)
            continue;

        double rx, ry, rfi;
        if(!interpolatePose(ld.timestamp, rx, ry, rfi))
            continue;

        int robotMx, robotMy;
        if(!worldToMap(rx, ry, robotMx, robotMy))
            continue;

        const double localAngleRad = deg2rad(-static_cast<double>(ld.scanAngle));
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


//uloha5
bool robot::isFreeForParticle(int mx, int my) const
{
    if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
        return false;

    const int8_t v = occupancyGrid[my][mx];
    return v == 0;
}

bool robot::randomFreeParticle(Particle &p)
{
    std::lock_guard<std::mutex> lk(mapMtx);

    if(occupancyGrid.empty() || occupancyGrid[0].empty())
        return false;
    //prepisat na static
    std::uniform_int_distribution<int> distX(0, mapWidthCells - 1);
    std::uniform_int_distribution<int> distY(0, mapHeightCells - 1);
    std::uniform_real_distribution<double> distFi(-kPi, kPi);

    for(int attempt = 0; attempt < 2000; ++attempt)
    {
        const int mx = distX(mclRng);
        const int my = distY(mclRng);

        if(!isFreeForParticle(mx, my))
            continue;

        double wx, wy;
        if(!mapToWorld(mx, my, wx, wy))
            continue;

        p.x_cm = wx;
        p.y_cm = wy;
        p.fi_rad = distFi(mclRng);
        p.weight = 1.0;
        return true;
    }

    return false;
}

void robot::initMonteCarloLocalization(int particleCount)
{
    std::lock_guard<std::mutex> lk(mclMtx);

    mclParticleCount = std::max(500, particleCount);
    particles.clear();
    particles.reserve(mclParticleCount);

    for(int i = 0; i < mclParticleCount; ++i)
    {
        Particle p;
        if(randomFreeParticle(p))
            particles.push_back(p);
    }

    const double w = particles.empty() ? 1.0 : 1.0 / static_cast<double>(particles.size());

    for(auto &p : particles)
        p.weight = w;

    mclInitialized = !particles.empty();
    monteCarloEnabled = mclInitialized;

    //uloha5
    // Vypocitame, ako velmi su vahy sustredene okolo najlepsieho stavu.
    // Cim mensi weighted spread, tym viac si MCL veri.
    mclBestWeight = 0.0;

    for(const auto &p : particles)
    {
        if(p.weight > mclBestWeight)
            mclBestWeight = p.weight;
    }

    // Najprv nastavime odhad ako najlepsiu casticu.
    updateEstimatedPoseFromParticles();

    double spreadSum = 0.0;
    double spreadWeightSum = 0.0;

    for(const auto &p : particles)
    {
        const double dx = p.x_cm - mclX_cm;
        const double dy = p.y_cm - mclY_cm;
        const double dist2 = dx * dx + dy * dy;

        spreadSum += p.weight * dist2;
        spreadWeightSum += p.weight;
    }

    if(spreadWeightSum > 0.0)
        mclWeightedSpreadCm = std::sqrt(spreadSum / spreadWeightSum);
    else
        mclWeightedSpreadCm = 9999.0;

}

void robot::setMonteCarloEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lk(mclMtx);
    monteCarloEnabled = enabled;
}

bool robot::isMonteCarloEnabled() const
{
    return monteCarloEnabled;
}

std::vector<std::pair<double, double>> robot::getParticlesCm()
{
    std::lock_guard<std::mutex> lk(mclMtx);

    std::vector<std::pair<double, double>> out;
    out.reserve(particles.size());

    for(const auto &p : particles)
        out.push_back({p.x_cm, p.y_cm});

    return out;
}

void robot::getMonteCarloPose(double &outX_cm, double &outY_cm, double &outFi_rad)
{
    std::lock_guard<std::mutex> lk(mclMtx);

    outX_cm = mclX_cm;
    outY_cm = mclY_cm;
    outFi_rad = mclFi_rad;
}

//uloha5
bool robot::isMclPoseReliable() const
{
    return monteCarloEnabled &&
           mclInitialized &&
           mclWeightedSpreadCm < mclReliableSpreadThresholdCm &&
           mclBestWeight > mclReliableBestWeightThreshold;
}

//uloha5
void robot::applyMclPoseToOdometry()
{
    std::lock_guard<std::mutex> lk(mclMtx);

    if(!monteCarloEnabled || !mclInitialized)
        return;

    //uloha5
    // Manuálny režim: MCL polohu aplikujeme vždy.
    // Používateľ si správnosť overí pohľadom na mapu.
    x = mclX_cm;
    y = mclY_cm;
    fi = mclFi_rad;

    //uloha5
    // Aby gyro v ďalšom callbacku neprepísalo fi naspäť,
    // nastavíme nový offset voči aktuálnemu gyro uhlu.
    gyroOffsetRad = normalizeAngleRad(latestGyroRadAbs - fi);

    //uloha5
    // Zastavíme rampy/regulátorový zásah po skokovej korekcii polohy.
    curForwCmd = 0.0;
    curRotCmd = 0.0;
}

//uloha5
std::vector<std::pair<int, int>> robot::getParticlesMapCells()
{
    std::lock_guard<std::mutex> lk(mclMtx);

    std::vector<std::pair<int, int>> out;
    out.reserve(particles.size());

    for(const auto &p : particles)
    {
        int mx = 0;
        int my = 0;

        if(worldToMap(p.x_cm, p.y_cm, mx, my))
            out.push_back({mx, my});
    }

    return out;
}

//uloha5
bool robot::getMonteCarloPoseMapCell(int &mx, int &my, double &fi_rad)
{
    std::lock_guard<std::mutex> lk(mclMtx);

    fi_rad = mclFi_rad;

    return worldToMap(mclX_cm, mclY_cm, mx, my);
}


double robot::expectedDistanceToObstacleCm(double x_cm,
                                           double y_cm,
                                           double angle_rad,
                                           double maxDistCm) const
{
    const double stepCm = mapResolutionCm * 0.75;

    for(double d = 0.0; d <= maxDistCm; d += stepCm)
    {
        const double px = x_cm + d * std::cos(angle_rad);
        const double py = y_cm + d * std::sin(angle_rad);

        int mx, my;
        if(!worldToMap(px, py, mx, my))
            return d;

        if(mx < 0 || mx >= mapWidthCells || my < 0 || my >= mapHeightCells)
            return d;

        if(occupancyGrid[my][mx] >= 50)
            return d;
    }

    return maxDistCm;
}

//uloha5
void robot::motionUpdateParticles(double dx_cm, double dy_cm, double dfi_rad)
{
    std::lock_guard<std::mutex> lk(mclMtx);

    if(!monteCarloEnabled || !mclInitialized || particles.empty())
        return;

    const double odomDist = std::sqrt(dx_cm * dx_cm + dy_cm * dy_cm);
    const double odomDir = std::atan2(dy_cm, dx_cm);

    //uloha5
    double transSigma = 0.15;
    double dirSigma   = deg2rad(0.15);
    double rotSigma   = deg2rad(0.15);
    //prepisat polohu amcr, pomocou polohu z odometrie
    if(odomDist >= 0.05 || std::fabs(dfi_rad) >= deg2rad(0.05))
    {
        //uloha5
        // Pri reálnom pohybe rastie šum podľa veľkosti posunu a otočenia.
        transSigma = 0.3 + 0.05 * std::fabs(odomDist) + 0.8 * std::fabs(dfi_rad);
        dirSigma   = deg2rad(0.3) + 0.015 * std::fabs(odomDist);
        rotSigma   = deg2rad(0.3) + 0.02 * std::fabs(odomDist) + 0.10 * std::fabs(dfi_rad);
    }

    std::normal_distribution<double> noiseTrans(0.0, transSigma);
    std::normal_distribution<double> noiseDir(0.0, dirSigma);
    std::normal_distribution<double> noiseRot(0.0, rotSigma);

    for(auto &p : particles)
    {
        const double noisyDist = odomDist + noiseTrans(mclRng);
        const double noisyDir = odomDir + noiseDir(mclRng);
        const double noisyRot = dfi_rad + noiseRot(mclRng);

        p.x_cm += noisyDist * std::cos(noisyDir);
        p.y_cm += noisyDist * std::sin(noisyDir);
        p.fi_rad = normalizeAngleRad(p.fi_rad + noisyRot);

        int mx, my;
        if(!worldToMap(p.x_cm, p.y_cm, mx, my) || !isFreeForParticle(mx, my))
            p.weight *= 0.05;
    }
}

void robot::sensorUpdateParticles(const std::vector<LaserData> &laserData)
{
    std::lock_guard<std::mutex> lk(mclMtx);

    if(!monteCarloEnabled || !mclInitialized || particles.empty())
        return;

    if(laserData.empty())
        return;

    std::lock_guard<std::mutex> mapLock(mapMtx);

    double weightSum = 0.0;

    for(auto &p : particles)
    {
        double err = 0.0;
        int used = 0;

        for(size_t i = 0; i < laserData.size(); i += static_cast<size_t>(mclLaserStep))
        {
            const auto &ld = laserData[i];

            const double measuredCm = static_cast<double>(ld.scanDistance) / 10.0;

            if(measuredCm < 10.0 || measuredCm > mclMaxLaserCm)
                continue;

            const double localAngleRad = deg2rad(-static_cast<double>(ld.scanAngle));
            const double globalAngle = p.fi_rad + localAngleRad;

            const double expectedCm =
                expectedDistanceToObstacleCm(p.x_cm, p.y_cm, globalAngle, mclMaxLaserCm);

            const double diff = measuredCm - expectedCm;
            err += diff * diff;
            ++used;
        }

        if(used == 0)
        {
            p.weight = 1e-9;
            continue;
        }

        const double meanErr = err / static_cast<double>(used);
        const double sigma2 = mclSigmaHitCm * mclSigmaHitCm;

        p.weight = std::exp(-meanErr / (2.0 * sigma2)) + 1e-12;
        weightSum += p.weight;
    }

    if(weightSum <= 0.0)
    {
        const double w = 1.0 / static_cast<double>(particles.size());

        for(auto &p : particles)
            p.weight = w;
    }
    else
    {
        for(auto &p : particles)
            p.weight /= weightSum;
    }

    updateEstimatedPoseFromParticles();
}

void robot::resampleParticles()
{
    std::lock_guard<std::mutex> lk(mclMtx);

    if(!monteCarloEnabled || !mclInitialized || particles.empty())
        return;

    std::vector<Particle> newParticles;
    newParticles.reserve(particles.size());

    std::vector<double> cumulative;
    cumulative.reserve(particles.size());

    double sum = 0.0;

    for(const auto &p : particles)
    {
        sum += p.weight;
        cumulative.push_back(sum);
    }

    if(sum <= 0.0)
        return;

    std::uniform_real_distribution<double> roulette(0.0, sum);

    const int randomCount = std::min(mclRandomParticles, static_cast<int>(particles.size()));

    for(size_t i = 0; i < particles.size(); ++i)
    {
        if(static_cast<int>(i) < randomCount)
        {
            Particle rp;
            if(randomFreeParticle(rp))
            {
                newParticles.push_back(rp);
                continue;
            }
        }

        const double r = roulette(mclRng);
        const auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
        const size_t idx = static_cast<size_t>(std::distance(cumulative.begin(), it));

        Particle selected = particles[std::min(idx, particles.size() - 1)];
        selected.weight = 1.0;
        newParticles.push_back(selected);
    }

    particles = std::move(newParticles);

    const double w = 1.0 / static_cast<double>(particles.size());

    for(auto &p : particles)
        p.weight = w;
}

//uloha5
void robot::updateEstimatedPoseFromParticles()
{
    if(particles.empty())
        return;

    //uloha5
    // Podla zadania berieme ako aktualnu polohu stav s najvyssou vahou.
    // Vazený priemer moze byt zly, ak su castice rozdelene do viacerych zhlukov.
    auto bestIt = std::max_element(
        particles.begin(),
        particles.end(),
        [](const Particle &a, const Particle &b)
        {
            return a.weight < b.weight;
        });

    if(bestIt == particles.end())
        return;

    mclX_cm = bestIt->x_cm;
    mclY_cm = bestIt->y_cm;
    mclFi_rad = bestIt->fi_rad;
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

    //uloha3
    currentOmegaRad = 0.0;

    // DÔLEŽITÉ:
    // mapu inicializuj iba vtedy, keď ešte nie je načítaná/vytvorená
    {
        std::lock_guard<std::mutex> lk(mapMtx);

        if(occupancyGrid.empty())
        {
            occupancyGrid.assign(mapHeightCells, std::vector<int8_t>(mapWidthCells, -1));
            hitGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));
            freeGrid.assign(mapHeightCells, std::vector<uint16_t>(mapWidthCells, 0));
        }
    }

    {
        std::lock_guard<std::mutex> lk(poseHistoryMtx);
        poseHistory.clear();
    }

    lastRobotTimestampUs = 0;
    rampTimestampInitialized = false;
    datacounter = 0;
    useDirectCommands = 0;

    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);

#ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif

#ifndef DISABLE_SKELETON
    robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif

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
    //uloha4
    // pri obyčajnom kliknutí bez plánovania zrušíme starú naplánovanú cestu
    followingPlannedPath = false;
    plannedPathCm.clear();
    plannedPathIndex = 0;

    std::lock_guard<std::mutex> lk(controlMtx);
    goalX_cm = gx_cm;
    goalY_cm = gy_cm;
    poseControlActive = true;
    useDirectCommands = 0;
    prevChosenDirRad = 0.0;

    prevBlocked.assign(vfhSectorCount, false);
    prevBlockedInitialized = true;

    prevFiRad = fi;
    prevFiInitialized = true;
}
//zastavenie
void robot::stopPoseControl()
{
    //uloha4
    followingPlannedPath = false;
    plannedPathCm.clear();
    plannedPathIndex = 0;

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

    //const double corridorHalfWidthCm = (robotRadiusCm + safetyMarginCm)*0.6;
    const double corridorHalfWidthCm = 10.0;

    for(const auto &ld : lidar)
    {
        const double distCm = static_cast<double>(ld.scanDistance) / 10.0; // mm -> cm
        if(distCm < 1.0)
            continue;

        double angRad = deg2rad(normalizeAngleDeg(-static_cast<double>(ld.scanAngle)));

        const double px = distCm * std::cos(angRad);
        const double py = distCm * std::sin(angRad);

        const double gx = std::cos(goalDirRad);
        const double gy = std::sin(goalDirRad);

        const double along = px * gx + py * gy;


        const double perp = std::fabs(-gy * px + gx * py);

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


    //zaciatok logiky pre branie natacania
    double deltaFiRad = 0.0;
    if(prevFiInitialized)
        deltaFiRad = normalizeAngleRad(fi - prevFiRad);
    else
    {
        prevFiRad = fi;
        prevFiInitialized = true;
    }


    const double sectorWidthDeg =
        (vfhMaxAngleDeg - vfhMinAngleDeg) / static_cast<double>(vfhSectorCount);


    //pokracovanie o deltu
    const double deltaFiDeg = rad2deg(deltaFiRad);
    const int sectorShift = static_cast<int>(std::round(deltaFiDeg / sectorWidthDeg));

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

        const double distCm = static_cast<double>(ld.scanDistance) / 10.0; // mm -> cm
        if(distCm < 1.0 || distCm > histogramRangeCm)
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

        const int s0 = angleToSector(angDeg - enlargeDeg);
        const int s1 = angleToSector(angDeg + enlargeDeg);

        const double mag = 0.06 * (histogramRangeCm - distCm);

        const int width = s1 - s0 + 1;
        const double add = mag / static_cast<double>(width);

        for(int s = s0; s <= s1; ++s)
            hist[s] += add;
       /* const double mag = 0.08*(histogramRangeCm - distCm); //1, 0.05

        //interval prekazky z 6 strany
        const int s0 = angleToSector(angDeg - enlargeDeg);
        const int s1 = angleToSector(angDeg + enlargeDeg);
        //zapis do histogramu
        for(int s = s0; s <= s1; ++s)
           //hist[s] = max(hist[s],mag);
            hist[s] += mag;

    */
        const double angRad = deg2rad(angDeg);

        const double px = distCm * std::cos(angRad);   // dopredu +
        const double py = distCm * std::sin(angRad);   // vlavo +

        // stredy minimálnych oblúkov
        // ľavý oblúk:  (0, +R)
        // pravý oblúk: (0, -R)
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



        /*
        if(py >= 0.0 && dLeft <= (Rmask + safeMaskRadius))
        {
            for(int s = vfhSectorCount/2; s < vfhSectorCount; ++s)
                maskBlocked[s] = true;
        }

        if(py <= 0.0 && dRight <= (Rmask + safeMaskRadius))
        {
            for(int s = 0; s < vfhSectorCount/2; ++s)
                maskBlocked[s] = true;
        }
        */

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
            int prevIdx = s + sectorShift;

            while(prevIdx < 0) prevIdx += vfhSectorCount;
            while(prevIdx >= vfhSectorCount) prevIdx -= vfhSectorCount;

            blocked[s] = prevBlocked[prevIdx];
        }
    }

    // aplikácia masky Hm
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
        prevFiRad = fi;
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

        if(widthDeg < wideGapDeg)
        {
            candidatesDeg.push_back(0.5 * (leftDeg + rightDeg)); //stred
        }
        else
        {
            candidatesDeg.push_back(leftDeg  + edgeOffsetDeg);
            candidatesDeg.push_back(rightDeg - edgeOffsetDeg);
        }
    }

    if(candidatesDeg.empty())
        candidatesDeg.push_back(goalDeg);

    const double prevDeg =
        rad2deg(normalizeAngleRad(prevChosenDirRad - deltaFiRad)); //upravene o natacanie

    double bestDeg = candidatesDeg.front();
    double bestCost = std::numeric_limits<double>::infinity();

    for(double cand : candidatesDeg)
    {
        cand = clamp(cand, vfhMinAngleDeg, vfhMaxAngleDeg);

        const double goalErr   = std::fabs(normalizeAngleDeg(cand - goalDeg));
        const double smoothErr = std::fabs(normalizeAngleDeg(cand - prevDeg));
        const double centerErr = std::fabs(cand);

        const double cost = 1.0 * goalErr + 0.3 * centerErr + 0.15 * smoothErr; //rovnica 13.str 1 0.3 0.3

        if(cost < bestCost)
        {
            bestCost = cost;
            bestDeg = cand;
        }
    }
    prevFiRad = fi;
    haveCandidate = true;
    prevChosenDirRad = deg2rad(bestDeg);
    return prevChosenDirRad;
}
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    ///tu mozete robit s datami z robota

    //uloha5
    // Ulozime si polohu pred aktualizaciou odometrie.
    // Z rozdielu starej a novej polohy potom posunieme castice.
    const double oldX = x;
    const double oldY = y;
    const double oldFi = fi;

    //ODOMETRIA
    const double gyroRadAbs = gyroRawToRad(static_cast<double>(robotdata.GyroAngle));

    //uloha5
    latestGyroRadAbs = gyroRadAbs;

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

    //uloha5
    // Pohybovy model Monte Carlo lokalizacie.
    // Po odometrii zistime, aky pohyb robot spravil, a aplikujeme ho na castice.
    {
        const double dxMcl = x - oldX;
        const double dyMcl = y - oldY;
        const double dfiMcl = normalizeAngleRad(fi - oldFi);

        motionUpdateParticles(dxMcl, dyMcl, dfiMcl);
    }

    //uloha3
    {
        static std::uint32_t prevTsOmega = 0;
        static double prevFiOmega = 0.0;

        double px = x;
        double py = y;
        double pfi = fi;

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
            const double dx = gx - x;
            const double dy = gy - y;
            const double rho = std::sqrt(dx*dx + dy*dy); // [cm]

            if(rho <= posDeadbandCm)
            {
                desForw = 0.0;
                desRot  = 0.0;
                curForwCmd = 0.0;
                curRotCmd  = 0.0;

                bool goNextWaypoint = false;

                {
                    std::lock_guard<std::mutex> lk(controlMtx);
                    poseControlActive = false;
                }

                //uloha4
                if(followingPlannedPath)
                    goNextWaypoint = true;

                if(goNextWaypoint)
                    startNextPlannedWaypoint();
            }
            else
            {
                const double heading = std::atan2(dy, dx);
                const double alphaGoal = normalizeAngleRad(heading - fi);

                double frontMinCm = std::numeric_limits<double>::infinity();
                bool haveCandidate = false;

                double alphaAvoid = alphaGoal;

                //uloha4
                if(followingPlannedPath && std::fabs(alphaGoal) > deg2rad(7.0))
                {
                    desForw = 0.0;
                    desRot  = clamp(kpAng * alphaGoal, -wMax * 0.55, wMax * 0.55);
                }
                else
                {
                    if(followingPlannedPath)
                    {
                        alphaAvoid = alphaGoal;
                        frontMinCm = std::numeric_limits<double>::infinity();
                    }
                    else
                    {
                        const double directGoalThresholdCm = 35.0;
                        const bool directToGoal =
                            (rho < directGoalThresholdCm) && canGoDirectlyToGoal(alphaGoal, rho);

                        if(rho < 15.0 || directToGoal)
                        {
                            alphaAvoid = alphaGoal;
                            frontMinCm = std::numeric_limits<double>::infinity();
                        }
                        else if(avoidanceEnabled)
                        {
                            alphaAvoid = computeAvoidanceDirection(alphaGoal, frontMinCm, haveCandidate);
                        }
                    }

                    double alphaCmd = alphaAvoid;

                    if(rho < 18.0)
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

                    const double localVMax = vMax;

                    desForw = clamp(kpDist * rho * steerScale * frontScale * goalScale,
                                    0.0,
                                    localVMax);

                    desRot  = clamp(kpAng * alphaCmd, -wMax * 0.65, wMax * 0.65);

                    if(frontMinCm < frontStopCm)
                        desForw = 0.0;
                }

                /*
                if(rho < 22.0)
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
                desRot  = clamp(kpAng * alphaCmd, -wMax*0.75, wMax*0.75);

                if(frontMinCm < frontStopCm)
                    desForw = 0.0;
                */
            }
        }
        // deadband
        const double forwDeadband = 8.0;
        const double rotDeadband  = 0.05;
        if (std::fabs(desForw) <= forwDeadband) desForw = 0.0;
        if (std::fabs(desRot)  <= rotDeadband)  desRot  = 0.0;

        if(!active && desForw != 0.0 && desRot != 0.0)
            desForw = 0.0;

        // cas pre rampu
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
///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    {
        std::lock_guard<std::mutex> lk(lidarMtx);
        latestLidar = laserData;
    }

    //uloha3
    updateMapFromLidar(laserData);

    //uloha5
    // Lidarove meranie urci vahy castic.
    // Odhad polohy sa vypocita este pred prevzorkovanim, ked maju castice realne vahy.
    // Potom sa spravi resampling pre dalsi krok algoritmu.
    if(monteCarloEnabled)
    {
        sensorUpdateParticles(laserData);
        resampleParticles();
    }

    //uloha5
    // Lidarove meranie urci vahy castic.
    // Odhad polohy sa vypocita este pred prevzorkovanim, ked maju castice realne vahy.
    // Potom sa spravi resampling pre dalsi krok algoritmu.
    if(monteCarloEnabled)
    {
        sensorUpdateParticles(laserData);
        resampleParticles();
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
