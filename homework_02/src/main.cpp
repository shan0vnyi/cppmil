#define ENABLE_DEBUG  0

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

#include <fstream>
#include <iostream>
#include <cstring>

// Стани дрона (enum)
enum DroneState
{
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

struct Ammo {
  float mass{0.0f};
  float drag{0.0f};
  float lift{0.0f};
  const char* name{};
  bool defined{false};
};
// Константи
const int BOMB_COUNT = 5;
const int TARGET_COUNT = 5;
const int TARGET_ARRAY_SIZE = 60;
const float g_gravity = 9.81f;
const int MAX_STEPS = 10000;

#define STOP_WHEN_TURNING

// Параметри боєприпасів (масиви)
char bombNames[BOMB_COUNT][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
float bombM[BOMB_COUNT] = {0.35f, 0.6f, 1.2f, 6.45f, 1.4f};
float bombD[BOMB_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
float bombL[BOMB_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

// Масиви координат цілей
float targetXInTime[TARGET_COUNT][TARGET_ARRAY_SIZE];
float targetYInTime[TARGET_COUNT][TARGET_ARRAY_SIZE];

// Вихідні масиви
float outX[MAX_STEPS];
float outY[MAX_STEPS];
float outDir[MAX_STEPS];
int outState[MAX_STEPS];
int outTarget[MAX_STEPS];
float outFireX[MAX_STEPS];
float outFireY[MAX_STEPS];
float outPredTgtX[MAX_STEPS];
float outPredTgtY[MAX_STEPS];

// Інтерполяція позиції цілі
void interpolateTarget(int targetIdx, float t, float arrayTimeStep, float& outTx, float& outTy)
{

    int idx= (int)std::floor(t / arrayTimeStep) % TARGET_ARRAY_SIZE;
    int next = (idx + 1) % TARGET_ARRAY_SIZE;
    float frac = (t / arrayTimeStep) - std::floor(t / arrayTimeStep);
    
    outTx = targetXInTime[targetIdx][idx]
          + (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) * frac;
    outTy = targetYInTime[targetIdx][idx] 
          + (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) * frac;
}

// Лінійна екстраполяція: знаємо лише поточний відрізок (idx, idx+1),
// прогнозуємо що ціль продовжить рухатись з тією ж швидкістю і напрямком.
// currentTime - поточний час, dt - на скільки секунд вперед прогнозуєно.
void extrapolateTarget(int targetIdx, float currentTime, float dt,
                       float arrayTimeStep, float& outTx, float& outTy)
{
    int idx = (int)std::floor(currentTime / arrayTimeStep) % TARGET_ARRAY_SIZE;
    int next = (idx + 1) % TARGET_ARRAY_SIZE;

    // Швидкість цілі на поточному відрізку
    float vx = (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) / arrayTimeStep;
    float vy = (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) / arrayTimeStep;

    // Поточна позиція (інтерполяція)
    float curX, curY;
    interpolateTarget(targetIdx, currentTime, arrayTimeStep, curX, curY);
    
    // Екстраполяція
    outTx = curX + vx * dt;
    outTy = curY + vy * dt;
}

// Нормалізація кута до [-PI, PI]
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

Ammo getAmmo(const char* name) {
    int idx = -1;
    
    for (int i = 0; i < BOMB_COUNT; i++) {
        if (strcmp(name, bombNames[i]) == 0) {
            idx = i;
            break;
        }
    }

    if (idx == -1) {
        throw std::invalid_argument(std::string("unknown ammo [") + name + "]");
    }

    return Ammo{bombM[idx], bombD[idx], bombL[idx], name};
}

// Балістика з Д3 1: час польоту (метод Кардано)
float calcFallTime(const float& h, const Ammo& ammo, const float& speed)
{
    float a = ammo.drag * g_gravity * ammo.mass - 2 * (ammo.drag * ammo.drag) * ammo.lift * speed;
    float b = -3 * g_gravity * (ammo.mass * ammo.mass) + 3 * ammo.drag * ammo.lift * ammo.mass * speed;
    float c = 6 * (ammo.mass * ammo.mass) * h;
    float p = -(b * b) / (3 * (a * a));
    float q = (2 * (b * b * b)) / (27 * (a * a * a)) + c / a;
    float phi = std::acos((3 * q) / (2 * p) * std::sqrt(-3 / p));
    float t = 2 * std::sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    return t;
}

float calcFallDistance(const Ammo& ammo, const float& t, const float& speed)
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

void getIntermediateAndDropPoint(float currentX, float currentY, float targetX, float targetY, float hDistance, float accelerationPath, float& outIntermediateX, float& outIntermediateY,
float& outFireX, float& outFireY, bool& outHasIntermediate)
{
    float distanceToTarget = std::sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
    outHasIntermediate = hDistance + accelerationPath > distanceToTarget;
    if (outHasIntermediate)
    {
        if (fabs(distanceToTarget) < 1e-6)
        {
            currentX = targetX - (hDistance + accelerationPath);
            currentY = targetY;
            distanceToTarget = hDistance + accelerationPath;
        }
        else
        {
            currentX = targetX - (targetX - currentX) * (hDistance + accelerationPath) / distanceToTarget;
            currentY = targetY - (targetY - currentY) * (hDistance + accelerationPath) / distanceToTarget;
            distanceToTarget = std::sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
        }
        outIntermediateX = currentX;
        outIntermediateY = currentY;
    }

    float ratio = (distanceToTarget - hDistance) / distanceToTarget;
    outFireX = currentX + (targetX - currentX) * ratio;
    outFireY = currentY + (targetY - currentY) * ratio;
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
float distFromSpeedAndAccel(float speed, float acc)
{
    return (speed * speed) / (2.0f * acc);
}

// Розрахунок часу підльоту до заданої координати
float calcTimeOfFlight(float currentX, float currentY, float targetX, float targetY, float currentDir, float angleThreshold,
                       float angularSpeed, float currentSpeed, float maxSpeed, float accelPath, bool needToStopAtTarget)
{
    float totalTimeToPoint = 0.0f;
    float desiredDir = std::atan2(targetY - currentY, targetX - currentX);
    float a = accel(maxSpeed, accelPath);

    bool ns = needStop(desiredDir, currentDir, angleThreshold);
    if (ns)
    {
        float pathToStop = (currentSpeed * currentSpeed) / (2.0f * a);

        currentX += std::cos(currentDir) * pathToStop;
        currentY += std::sin(currentDir) * pathToStop;

        totalTimeToPoint += accelTime(currentSpeed, a),
        totalTimeToPoint += std::fabs(normalizeAngle(desiredDir - currentDir)) / angularSpeed;

        currentSpeed = 0;
        currentDir = desiredDir;
    }
    
    float distanceToTarget = std::hypot(targetX - currentX, targetY - currentY);
    float deccerelatePath = std::min(needToStopAtTarget ? accelPathFromSpeed(maxSpeed, a): 0, distanceToTarget);
    distanceToTarget -= deccerelatePath;
    totalTimeToPoint += accelTimeFromDist(deccerelatePath, a);

    float accelDist = std::min(accelPathFromSpeed(maxSpeed, a) - accelPathFromSpeed(currentSpeed, a), distanceToTarget);
    totalTimeToPoint += accelTimeFromDist(accelDist, a);
    distanceToTarget -= accelDist;
    totalTimeToPoint += distanceToTarget / maxSpeed;

    return totalTimeToPoint;
}

// MAIN
int main(int argc, char** argv)
{
    // Читання input.txt
    float xd, yd, zd;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    char ammo_name [15] = "";
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;

    {
        std::string ipath = "";

        if (argv[1]) {
            ipath = argv[1];
        } else {
            ipath = "input.txt";
        }
        std::ifstream fin(ipath);
        if (!fin.is_open())
        {
            std::cerr << "Cannot open input.txt" << std::endl;
            return 1;
        }
        fin >> xd >> yd >> zd;
        fin >> initialDir;
        fin >> attackSpeed;
        fin >> accelerationPath;
        fin >> ammo_name;
        fin >> arrayTimeStep;
        fin >> simTimeStep;
        fin >> hitRadius;
        fin >> angularSpeed;
        fin >> turnThreshold;
        fin.close();
    }

    // Пошук боєприпасу за назвою
    Ammo bomb{};
    try {
        bomb = getAmmo(ammo_name);
    } catch (const std::invalid_argument& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }

    //--- Читання targets.txt
    {
        std::string tpath = "";
        
        if (argv[2]) {
            tpath = argv[2];
        } else {
            tpath = "targets.txt";
        }

        std::ifstream ftgt(tpath);

        if (!ftgt.is_open())
        {
            std::cerr << "Cannot open targets.txt" << std::endl;
                return 1;
        }

        for (int i = 0; i < TARGET_COUNT; i++)
        {
            for (int j = 0; j < TARGET_ARRAY_SIZE; j++)
            {
                ftgt >> targetXInTime[i][j];
            }
        }
        for (int i = 0; i < TARGET_COUNT; i++)
        {
            for (int j = 0; j < TARGET_ARRAY_SIZE; j++)
            {
                ftgt >> targetYInTime[i][j];
            }
        }

        ftgt.close();
    }

    // Параметри руху дрона
    float acceleration = accel(attackSpeed, accelerationPath);
    float droneX = xd;
    float droneY = yd;
    float direction = initialDir;
    float speed = 0.0f;
    DroneState state = STOPPED;
    float currentTime = 0.0f;
    int currentTarget = -1;
    /*
    float targetDir = initialDir;
#ifdef STOP_WHEN_TURNING
    float turnRemaining = 0.0f;
#endif
    */
    int step = 0;

    // Попередній розрахунок балістичних констант (залежать лише від висоти та снаряду)
    float flightTime = calcFallTime(zd, bomb, attackSpeed);
    float hDist = calcFallDistance(bomb, flightTime, attackSpeed);

    // Основний цикл симуляції
    while (step < MAX_STEPS)
    {
        float bestTime = 1e9f;
        int bestTarget = 0;
        float bestPredX = droneX, bestPredY = droneY;
        float bestFireX = droneX, bestFireY = droneY;
        float desiredDirectionForBest = direction;
        float fx = 0, fy = 0;

        for (int targetId = 0; targetId < TARGET_COUNT; ++targetId)
        {
            float predX = 0, predY = 0;
            float totalTime = 0;
            bool hasIntermediate;

            extrapolateTarget(targetId, currentTime, totalTime + flightTime, arrayTimeStep, predX, predY);
            DEBUG("pred(" << predX << "," << predY << ")");
            float intermediateX, intermediateY;
            float fireX, fireY;
            getIntermediateAndDropPoint(droneX, droneY, predX, predY, hDist, accelerationPath, intermediateX, intermediateY, fireX, fireY, hasIntermediate);
            
            // Якщо дрон вже летить до поточної цілі - не відступаємо назад
            if (hasIntermediate && targetId == currentTarget && (state == MOVING || state == ACCELERATING))
            {
                hasIntermediate = false;
            }

            totalTime = 0;
            if (hasIntermediate)
            {
                totalTime += calcTimeOfFlight(droneX, droneY, intermediateX, intermediateY, direction,
                                                turnThreshold, angularSpeed, speed, attackSpeed, accelerationPath, true);
                float directionToFire = std::atan2(fireY - intermediateY, fireX - intermediateX);
                totalTime += std::fabs(normalizeAngle(directionToFire - direction)) / angularSpeed;
                totalTime += calcTimeOfFlight(intermediateX, intermediateY, fireX, fireY, directionToFire,
                                                turnThreshold, angularSpeed, attackSpeed, attackSpeed, accelerationPath, false);
                fx = fireX;
                fy = fireY;
            }
            else
            {
                totalTime = calcTimeOfFlight(droneX, droneY, fireX, fireY, direction, turnThreshold, angularSpeed, speed, attackSpeed, accelerationPath, false);
                fx = fireX;
                fy = fireY;
            }


            if (totalTime < bestTime)
            {
                bestTime = totalTime;
                bestTarget = targetId;
                bestPredX = predX;
                bestPredY = predY;
                bestFireX = fx;
                bestFireY = fy;
            }
        }

        currentTarget = bestTarget;

        // Бажаний напрямок = до fire point найкращої цілі
        desiredDirectionForBest = std::atan2(bestFireY - droneY, bestFireX - droneX);

        // 4. Записати дані кроку у вихідні масиви
        outX[step] = droneX;
        outY[step] = droneY;
        outDir[step] = direction;
        outState[step] = (int)state;
        outTarget[step] = currentTarget;
        outFireX[step] = bestFireX;
        outFireY[step] = bestFireY;
        outPredTgtX[step] = bestPredX;
        outPredTgtY[step] = bestPredY;

        float deltaPath = 0;
        float deltaAngle = normalizeAngle(desiredDirectionForBest - direction);
    
        // 6. Автомат станів (самокерований, як у Д33)
        switch (state)
        {
            case STOPPED:
            {
                if (std::fabs(deltaAngle) > turnThreshold)
                {
                    state = TURNING;
                }
                else
                {
                    direction = desiredDirectionForBest;
                    state = ACCELERATING;
                }
                break;
            }

            case ACCELERATING:
            {
                if (std::fabs(deltaAngle) > turnThreshold && speed > 0.01f)
                {
                    // Треба повернути спочатку гальмуємо
                    state = DECELERATING;
                    float prevSpeed = speed;
                    speed -= acceleration * simTimeStep;

                    if (speed <= 0)
                    {
                        speed = 0;
                        state = STOPPED;
                    }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                else
                {
                    // Малі поправки курсу на льоту
                    if (std:: fabs(deltaAngle) <= turnThreshold)
                    {
                        direction = desiredDirectionForBest;
                    }
                    
                    float prevSpeed = speed;
                    speed += acceleration * simTimeStep;
                    if (speed >= attackSpeed)
                    {
                        speed = attackSpeed;
                        state = MOVING;
                    }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                break;
            }

            case DECELERATING:
            {
                float prevSpeed = speed;
                speed -= acceleration * simTimeStep;
                if (speed <= 0)
                {
                    speed = 0;
                    state = STOPPED;
                }
                
                deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                break;
            }

            case TURNING:
            {
                float da = normalizeAngle(desiredDirectionForBest - direction);
                if (std::fabs(da) <= angularSpeed * simTimeStep)
                {
                    direction = desiredDirectionForBest;
                    state = ACCELERATING;
                }
                else
                {
                    direction += (da > 0 ? 1.0f : -1.0f) * angularSpeed * simTimeStep;
                    direction = normalizeAngle(direction);
                }
                break;
            }

            case MOVING:
            {
                if (std::fabs(deltaAngle) > turnThreshold)
                {
                    state = DECELERATING;
                    float prevSpeed = speed;
                    speed -= acceleration * simTimeStep;
                    if (speed <= 0)
                    {
                        speed = 0;
                        state = STOPPED;
                    }
                    deltaPath = (prevSpeed + speed) / 2.0f * simTimeStep;
                }
                else
                {
                    // Малі поправки курсу на льоту
                    if (std::fabs(deltaAngle) <= turnThreshold)
                    {
                        direction = desiredDirectionForBest;
                    }
                    
                    deltaPath = speed * simTimeStep;
                }
                break;
            }
        }

        droneX += std::cos(direction) * deltaPath;
        droneY += std::sin(direction) * deltaPath;

        // Перевірка влучання: дрон долетів до fire point
        if (state == MOVING && std::hypot(droneX - bestFireX, droneY - bestFireY) <= hitRadius * 0.1f)
        {
            break;  // скид боєприпасу!
        }

        currentTime += simTimeStep;
        ++step;
    }

    // Запис simulation.txt
    int N = step; // кількість кроків (індекс 0.. N)
    std::ofstream fout("simulation.txt");

    if (!fout.is_open())
    {
        std::cerr << "Cannot write simulation.txt" << std::endl;
        return 1;
    }

    // Рядок 1: N
    fout << N << std::endl;
    // Рядок 2: x0 y0 x1 y1 ... XN YN
    for (int i = 0; i <= N; i++)
    {
        if (i > 0) fout << " ";
        fout << outX[i] << " " << outY[i];
    }
    fout << std::endl;
    // Рядок 3: d0 d1 ... dN outDir[step]
    for (int j = 0; j <= N; j++)
    {
        if (j > 0) fout << " ";
        fout << outDir[j];
    }
    fout << std::endl;
    // Рядок 4: s0 s1 ... sN outState
    for (int k = 0; k <= N; k++)
    {
        if (k > 0) fout << " ";
        fout << outState[k];
    }
    fout << std::endl;
    // Рядок 5: t0 t1 ... tN
    for (int n = 0; n <= N; n++)
    {
        if (n > 0) fout << " ";
        fout << outTarget[n];
    }
    fout << std::endl;

    fout.close();

    return 0;
}