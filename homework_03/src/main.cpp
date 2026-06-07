#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
    #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
    #define LOG(msg)
#endif

#if ENABLE_DEBUG
    #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
    #define DEBUG(msg)
#endif

#include <nlohmann/json.hpp>

using json = nlohmann::json;
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

#include <fstream>
#include <iostream>
#include <cstring>

struct Coord {
    float x;
    float y;
    // Adding coordinates
    Coord operator+(const Coord& other) const {
        Coord result;
        result.x = x + other.x;
        result.y = y + other.y;

        return result;
    }
    // Substraction of coordinates
    Coord operator-(const Coord& other) const {
        Coord result;
        result.x = x - other.x;
        result.y = y - other.y;

        return result;
    }
    // Multiplication by a scallar
    Coord operator*(float s) const {
        Coord result;
        result.x = x * s;
        result.y = y * s;

        return result;
    }
    // Division by a scallar
    Coord operator/(float s) const {
        Coord result;
        result.x = x / s;
        result.y = y / s;

        return result;
    }
    // Coordinate comparison
    bool operator==(const Coord& other) const {
        return other.x == x && other.y == y;
    }

    float lenght() {
        return std::hypot(x, y);
    }

    Coord normalize() {
        Coord result;
        float lenght = this->lenght();
        result.x = x / lenght;
        result.y = y / lenght;

        return result;
    }
};

struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int state;
    int targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};
//-- Drone states
enum DroneState
{
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

//-- Consts
const float g_gravity = 9.81f;
const int MAX_STEPS = 10000;


#define STOP_WHEN_TURNING

std::optional<DroneConfig> readInputConfig(const char* ipath)
{
    DroneConfig cfg;
    //-- Read config.json
    std::ifstream fin(ipath);

    if (!fin.is_open())
    {
        std::cerr << "Cannot open config.json" << std::endl;
        return std::nullopt;
    }

    json j;
    fin >> j;
    cfg.startPos = Coord(j["drone"]["position"]["x"], j["drone"]["position"]["y"]);
    cfg.altitude = j["drone"]["altitude"];
    cfg.initialDir = j["drone"]["initialDirection"];
    cfg.attackSpeed = j["drone"]["attackSpeed"];
    cfg.accelPath = j["drone"]["accelerationPath"];
    cfg.angularSpeed = j["drone"]["angularSpeed"];
    cfg.turnThreshold = j["drone"]["turnThreshold"];
    const char* tmp = j["ammo"].get<std::string>().c_str();
    std::strncpy(cfg.ammoName, tmp, 31);
    cfg.simTimeStep = j["simulation"]["timeStep"];
    cfg.hitRadius = j["simulation"]["hitRadius"];
    cfg.arrayTimeStep = j["targetArrayTimeStep"];

    LOG("Config loaded: pos(" << cfg.startPos.x << "," << cfg.startPos.y << ")");
    LOG("               altitude=" << cfg.altitude);
    LOG("               direction=" << cfg.initialDir);
    LOG("               speed=" << cfg.attackSpeed);
    LOG("               accelPath=" << cfg.accelPath);
    LOG("               angularSpeed=" << cfg.angularSpeed);
    LOG("               turnThreshold=" << cfg.turnThreshold);
    LOG("               ammo=" << cfg.ammoName);
    LOG("               timeStep=" << cfg.simTimeStep);
    LOG("               hitRadius=" << cfg.hitRadius);
    LOG("               arrayTimeStep=" << cfg.arrayTimeStep);
    
    return cfg;
}

std::optional<AmmoParams> getAmmoFromInput(const char* ammoName, const char* apath)
{
    AmmoParams bomb;
    //-- Read ammo.json
    std::ifstream fa(apath);

    if (!fa.is_open())
    {
        std::cerr << "Cannot open ammo.json" << std::endl;
        return std::nullopt;;
    }

    json ja; fa >> ja;
    int ammoCount = ja.size();
    AmmoParams* ammo = new AmmoParams[ammoCount];
    
    for (int i = 0; i < ammoCount; i++) {
        std::strncpy(ammo[i].name, ja[i]["name"].get<std::string>().c_str(), 31);
        ammo[i].mass = ja[i]["mass"];
        ammo[i].drag = ja[i]["drag"];
        ammo[i].lift = ja[i]["lift"];
    }

    int ammoIdx = -1;

    for (int k = 0; k < ammoCount; k++) {
        DEBUG("name[" << ammo[k].name << "]; mass[" << ammo[k].mass << "]; drag[" << ammo[k].drag << "]; lift[" << ammo[k].lift << "]");
        if (strcmp(ammoName, ammo[k].name) == 0) {
            bomb = ammo[k];
            ammoIdx = k;
            break;
        }
    }
    
    delete[] ammo;

    if (ammoIdx < 0)
    {
        std::cerr << "Unknown ammo [" << ammoName << "]" << std::endl;
        return std::nullopt;;
    }

    LOG("Ammo found: " << ammoName);

    return bomb;
}

// Normalize angle from -PI to PI
float normalizeAngle(float a)
{
    a = fmod(a + M_PI, 2 * M_PI);

    if (a < 0)
    {
        a += 2 * M_PI;
    }

    a -= M_PI;

    return a;
}

float calcFallTime(AmmoParams ammo, DroneConfig cfg)
{
    float a = ammo.drag * g_gravity * ammo.mass - 2 * (ammo.drag * ammo.drag) * ammo.lift * cfg.attackSpeed;
    float b = -3 * g_gravity * (ammo.mass * ammo.mass) + 3 * ammo.drag * ammo.lift * ammo.mass * cfg.attackSpeed;
    float c = 6 * (ammo.mass * ammo.mass) * cfg.altitude;
    float p = -(b * b) / (3 * (a * a));
    float q = (2 * (b * b * b)) / (27 * (a * a * a)) + c / a;
    float phi = std::acos((3 * q) / (2 * p) * std::sqrt(-3 / p));
    float t = 2 * std::sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    return t;
}

float calcFallDistance(AmmoParams ammo, const float& t, const float& speed)
{
  const float dragP2 = ammo.drag * ammo.drag;
  const float dragP3 = ammo.drag * ammo.drag * ammo.drag;
  const float dragP4 = ammo.drag * ammo.drag * ammo.drag * ammo.drag;
  const float liftP2 = ammo.lift * ammo.lift;
  const float liftP3 = ammo.lift * ammo.lift * ammo.lift;
  const float liftP4 = ammo.lift * ammo.lift * ammo.lift * ammo.lift;
  const float massP2 = ammo.mass * ammo.mass;
  const float massP3 = ammo.mass * ammo.mass * ammo.mass;
  const float massP4 = ammo.mass * ammo.mass * ammo.mass * ammo.mass;

  float h = speed * t - (t * t) * ammo.drag * speed / (2 * ammo.mass) +
            (t * t * t) * (6 * ammo.drag * g_gravity * ammo.lift * ammo.mass - 6 * dragP2 * (liftP2 - 1) * speed) /
              (36 * massP2) +
            pow(t, 4) *
              (-6 * dragP2 * g_gravity * ammo.lift * (1 + liftP2 + liftP4) * ammo.mass +
               3 * dragP3 * liftP2 * (1 + liftP2) * speed +
               6 * dragP3 * liftP4 * (1 + liftP2) * speed) /
              (36 * pow(1 + liftP2, 2) * massP3) +
            pow(t, 5) *
              (3 * dragP3 * g_gravity * liftP3 * ammo.mass -
               3 * dragP4 * liftP2 * (1 + liftP2) * speed) /
              (36 * (1 + liftP2) * massP4);

  return h;
}

Coord extrapolateTarget(Coord**& targets, const int& targetIdx, const float& currentTime,
                        const float& aFallTime, const float& arrayTimeStep, const int& timeSteps)
{
    int idx = (int)std::floor(currentTime / arrayTimeStep);
    int prev = idx;
    //-- Previous idx allowed only after 1+ arrayTimeStep iteration
    if (currentTime > arrayTimeStep) {
        prev = (idx - 1) % timeSteps;
    }

    float frac = (currentTime / arrayTimeStep) - std::floor(currentTime / arrayTimeStep);
    idx %= (int)timeSteps;
    Coord deltaC, deltaV, predicted, interpC, curC;
    predicted = interpC = curC = targets[targetIdx][idx];

    if (idx != prev) {
        deltaC = curC - targets[targetIdx][prev];
        interpC = curC + deltaC * frac;
        deltaV = deltaC / arrayTimeStep;
        predicted = interpC + deltaV * aFallTime;
    }

    return predicted;
}

Coord extrapolateTargetWithFuture(Coord**& targets, const int& targetIdx, const float& currentTime,
                        const float& aFallTime, const float& arrayTimeStep, const int& timeSteps)
{
    int idx = (int)std::floor(currentTime / arrayTimeStep);
    int next = (idx + 1) % timeSteps;
    float frac = (currentTime / arrayTimeStep) - std::floor(currentTime / arrayTimeStep);
    idx %= (int)timeSteps;
    Coord deltaC, deltaV, predicted, interpC, curC;
    predicted = interpC = curC = targets[targetIdx][idx];
    
    deltaC = targets[targetIdx][next] - curC;
    interpC = curC + deltaC * frac;
    deltaV = deltaC / arrayTimeStep;
    predicted = interpC + deltaV * aFallTime;
    

    return predicted;
}

Coord getDropPoint(Coord droneP, const Coord& targetP, const float& hDist, const float& accPath, Coord& intermediateP)
{
    Coord delta = targetP - droneP;
    float dist = delta.lenght();
    float minDist = hDist + accPath;


    if (minDist > dist)
    {
        if (std::fabs(dist) < 1e-6)
        {
            droneP.x = droneP.x - minDist;
            droneP.y = droneP.y;
            delta = targetP - droneP;
            dist = minDist;
        }
        else
        {
            droneP = targetP - delta * minDist / dist;
            delta = targetP - droneP;
            dist = delta.lenght();
        }
        intermediateP = droneP;
    }
    
    Coord firePoint = targetP - delta.normalize() * hDist;
    
    return firePoint;
}

bool needStop(float desiredDir, float currentDir, float angleThreshold)
{
#ifdef STOP_WHEN_TURNING
    float deltaAngle = normalizeAngle(desiredDir - currentDir);

    return std::fabs(deltaAngle) > angleThreshold;
#else
    return false;
#endif
}

float accel(float speed, float accelPath)
{
    return speed * speed / (2.0f * accelPath);
}

float accelTime(float speed, float acc)
{
    return speed / acc;
}

float accelTimeFromDist(float dist, float acc)
{
    return std::sqrt(2.0f * dist / acc);
}

float accelPathFromSpeed (float speed, float acc)
{
    return (speed * speed) / (2.0f * acc);
}

float calcFlightTimeToTarget(Coord droneP, Coord fireP, DroneConfig cfg, float curDir, float curSpeed, bool stopOnTrgt)
{
    float totalTimeToPoint = 0.0f;
    Coord delta = fireP - droneP;
    float desiredDir = std::atan2(delta.y, delta.x);
    float a = accel(cfg.attackSpeed, cfg.accelPath);
    bool ns = needStop(desiredDir, curDir, cfg.turnThreshold);

    if (ns)
    {
        float pathToStop = (curSpeed * curSpeed) / (2.0f * a);
        droneP.x += std::cos(curDir) * pathToStop;
        droneP.y += std::sin(curDir) * pathToStop;
        totalTimeToPoint += accelTime(curSpeed, a);
        totalTimeToPoint += std::fabs(normalizeAngle(desiredDir - curDir)) / cfg.angularSpeed;
        curSpeed = 0;
        curDir = desiredDir;
        delta = fireP - droneP;
    }

    float distToTrgt = delta.lenght();
    float deccPath = std::min(stopOnTrgt ? accelPathFromSpeed(cfg.attackSpeed, a): 0, distToTrgt);
    distToTrgt -= deccPath;
    totalTimeToPoint += accelTimeFromDist(deccPath, a);



    float accelDist = std::min(accelPathFromSpeed(cfg.attackSpeed, a) - accelPathFromSpeed(curSpeed, a), distToTrgt);
    totalTimeToPoint += accelTimeFromDist(accelDist, a);
    distToTrgt -= accelDist;
    totalTimeToPoint += distToTrgt / cfg.attackSpeed;

    return totalTimeToPoint;
}


void processState(DroneState& state, DroneConfig cfg, Coord& dronePosition, float& curDir, float& bestDir, float& curSpeed)
{
    float deltaPath = 0;
    float deltaAngle = normalizeAngle(bestDir - curDir);
    float acceleration = accel(cfg.attackSpeed, cfg.accelPath);

    switch (state)
    {
        case STOPPED:
        {
            if (std::fabs(deltaAngle) > cfg.turnThreshold)
            {
                state = TURNING;
            }
            else
            {
                curDir = bestDir;
                state = ACCELERATING;
            }
            break;
        }

        case ACCELERATING:
        {
            if (std::fabs(deltaAngle) > cfg.turnThreshold && curSpeed > 0.01f)
            {
                //-- Need to turn. Stop first
                state = DECELERATING;
                float prevSpeed = curSpeed;
                curSpeed -= acceleration * cfg.simTimeStep;

                if (curSpeed <= 0)
                {
                    curSpeed = 0;
                    state = STOPPED;
                }
                deltaPath = (prevSpeed + curSpeed) / 2.0f * cfg.simTimeStep;
            }
            else
            {
                //-- Little correct of flight
                if (std:: fabs(deltaAngle) <= cfg.turnThreshold)
                {
                    curDir = bestDir;
                }
                
                float prevSpeed = curSpeed;
                curSpeed += acceleration * cfg.simTimeStep;
                if (curSpeed >= cfg.attackSpeed)
                {
                    curSpeed = cfg.attackSpeed;
                    state = MOVING;
                }
                deltaPath = (prevSpeed + curSpeed) / 2.0f * cfg.simTimeStep;
            }
            break;
        }

        case DECELERATING:
        {
            float prevSpeed = curSpeed;
            curSpeed -= acceleration * cfg.simTimeStep;
            if (curSpeed <= 0)
            {
                curSpeed = 0;
                state = STOPPED;
            }
            
            deltaPath = (prevSpeed + curSpeed) / 2.0f * cfg.simTimeStep;
            break;
        }

        case TURNING:
        {
            float da = normalizeAngle(bestDir - curDir);
            if (std::fabs(da) <= cfg.angularSpeed * cfg.simTimeStep)
            {
                curDir = bestDir;
                state = ACCELERATING;
            }
            else
            {
                curDir += (da > 0 ? 1.0f : -1.0f) * cfg.angularSpeed * cfg.simTimeStep;
                curDir = normalizeAngle(curDir);
            }
            break;
        }

        case MOVING:
        {
            if (std::fabs(deltaAngle) > cfg.turnThreshold)
            {
                state = DECELERATING;
                float prevSpeed = curSpeed;
                curSpeed -= acceleration * cfg.simTimeStep;
                if (curSpeed <= 0)
                {
                    curSpeed = 0;
                    state = STOPPED;
                }
                deltaPath = (prevSpeed + curSpeed) / 2.0f * cfg.simTimeStep;
            }
            else
            {
                //-- Little correct of flight
                if (std::fabs(deltaAngle) <= cfg.turnThreshold)
                {
                    curDir = bestDir;
                }
                
                deltaPath = curSpeed * cfg.simTimeStep;
            }
            break;
        }
    }

    dronePosition.x += std::cos(curDir) * deltaPath;
    dronePosition.y += std::sin(curDir) * deltaPath;
}

void writeSimToJson(const SimStep* s, const int n)
{
    json out;
    out["totalSteps"] = n;
    out["steps"] = json::array();
    for (int i = 0; i < n; i++)
    {

        json step;
        step["position"] = {{"x", s[i].pos.x}, {"y", s[i].pos.y}};
        step["direction"] = s[i].direction;
        step["state"] = s[i].state;
        step["targetIndex"] = s[i].targetIdx;
        step["dropPoint"] = {{"x", s[i].dropPoint.x}, {"y", s[i].dropPoint.y}};
        step["aimPoint"] = {{"x", s[i].aimPoint.x}, {"y", s[i].aimPoint.y}};
        step["predictedTarget"] = {{"x", s[i].predictedTarget.x}, {"y", s[i].predictedTarget.y}};
        out["steps"].push_back(step);
    }

    std::ofstream fout("simulation.json");
    fout << out.dump(2);
}


//-- Main function
int main(int argc, char** argv)
{
    //-- Read config.json
    const char* ipath = "";

    if (argv[1]) {
        ipath = argv[1];
    } else {
        ipath = "config.json";
    }
    const auto cfg = readInputConfig(ipath);
    if (!cfg.has_value()) {
        std::cerr << "error: cannot read input config" << std::endl;
        return 1;
    }

    //-- Get ammo from ammo.json
    const char* apath = "";

    if (argv[2]) {
        apath = argv[2];
    } else {
        apath = "ammo.json";
    }
    const auto bomb = getAmmoFromInput(cfg->ammoName, apath);
    if (!bomb.has_value()) {
        std::cerr << "error: fail to get ammo params" << std::endl;
        return 1;
    }

    //-- Read targets.json
    std::string tpath = "";

    if (argv[3]) {
        tpath = argv[3];
    } else {
        tpath = "targets.json";
    }
    std::ifstream ft(tpath);

    if (!ft.is_open())
    {
        std::cerr << "Cannot open targets.json" << std::endl;
        return 1;
    }

    json jt;
    ft >> jt;
    int trgtCount = jt["targetCount"];
    int timeSteps = jt["timeSteps"];
    Coord** targets = new Coord*[trgtCount];

    for (int i = 0; i < trgtCount; i++) {
        targets[i] = new Coord[timeSteps];
    
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = jt["targets"][i]["positions"][j]["x"];
            targets[i][j].y = jt["targets"][i]["positions"][j]["y"];
        }
    }

    //-- Init simSteps
    SimStep* steps = new SimStep[MAX_STEPS];

    //-- Main cycle initial variables
    DroneState curState{STOPPED};
    float curTime{0.0f};
    float curSpeed{0.0f};
    float curDirection = cfg->initialDir;
    Coord curDronPosition = cfg->startPos;
    int curTarget{-1};
    int step{0};

    //-- Ballistics constants calculation (depends on ammo altitude only)
    float aFallTime = calcFallTime(*bomb, *cfg);
    float hDist = calcFallDistance(*bomb, aFallTime, cfg->attackSpeed);

    while (step < MAX_STEPS)
    {
        float bestTime{1e9f};
        int bestTarget{0};
        float bestDir = curDirection;
        Coord bestPredictC, bestFireC, fireC, predC;
        bestPredictC = bestFireC = curDronPosition;

        DEBUG("Step " << step << " pos=(" << curDronPosition.x << "," << curDronPosition.y << ")");

        for (int targetId = 0; targetId < trgtCount; ++targetId)
        {
            float flightTimeToTarget{0.0f};
            bool hasIntermediate;
            Coord intermPoint{0.0f, 0.0f};

            //Coord predictedTarget = extrapolateTarget(targets, targetId, curTime, aFallTime, cfg->arrayTimeStep, timeSteps);
            Coord predictedTarget = extrapolateTargetWithFuture(targets, targetId, curTime, aFallTime, cfg->arrayTimeStep, timeSteps);
            predC = predictedTarget;
    
            Coord dropPoint = getDropPoint(curDronPosition, predictedTarget, hDist, cfg->accelPath, intermPoint);
            fireC = dropPoint;
    
            hasIntermediate = Coord{} != intermPoint;

            //-- If drone already flight to current target - don't give up!
            if (hasIntermediate && targetId == curTarget && (curState == MOVING || curState == ACCELERATING))
            {
                hasIntermediate = false;
            }

            if (hasIntermediate)
            {
                Coord iDelta = dropPoint - intermPoint;
                flightTimeToTarget += calcFlightTimeToTarget(curDronPosition, dropPoint, *cfg, curDirection, curSpeed, true);
                float directionToFire = std::atan2(iDelta.y, iDelta.x);
                flightTimeToTarget += std::fabs(normalizeAngle(directionToFire - curDirection)) / cfg->angularSpeed;
                flightTimeToTarget += calcFlightTimeToTarget(curDronPosition, dropPoint, *cfg, curDirection, cfg->attackSpeed, false);
            }
            else
            {
                flightTimeToTarget = calcFlightTimeToTarget(curDronPosition, dropPoint, *cfg, curDirection, curSpeed, false);
            }

            if (flightTimeToTarget < bestTime)
            {
                bestTime = flightTimeToTarget;
                bestTarget = targetId;
                bestPredictC = predC;
                bestFireC = fireC;
            }
        }

        curTarget = bestTarget;
        Coord deltaBest = bestFireC - curDronPosition;
        bestDir = std::atan2(deltaBest.y, deltaBest.x);

        Coord aimDir = {std::cos(curDirection), std::sin(curDirection)};
        //-- Write data into simStep
        steps[step].pos = curDronPosition;
        steps[step].direction = curDirection;
        steps[step].state = curState;
        steps[step].targetIdx = curTarget;
        steps[step].dropPoint = bestFireC;
        steps[step].aimPoint = curDronPosition + aimDir * hDist;
        steps[step].predictedTarget = bestPredictC;

        processState(curState, *cfg, curDronPosition, curDirection, bestDir, curSpeed);
        DEBUG(" target=" << curTarget << " state=" << curState);

        // Check hit: drone arrive to fire point
        Coord deltaHit = curDronPosition - bestFireC;
        if (curState == MOVING && deltaHit.lenght() <= cfg->hitRadius * 0.1f)
        {
            break;  //-- drop bomb!
        }

        curTime += cfg->simTimeStep;
        ++step;
    }

    LOG("Simulation complete. Steps: " << step);

    for (int i = 0; i < trgtCount; i++) {
        delete[] targets[i];
    }
    
    writeSimToJson(steps, step);

    delete[] steps;
    delete[] targets;

    return 0;
}