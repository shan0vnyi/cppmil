#define ENABLE_DEBUG  0

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

#include <iostream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>


struct Tick {
    long ts,fl,fr,bl,br;

    bool operator==(const Tick& other) const {
        return (ts == other.ts &&
                fl == other.fl &&
                fr == other.fr &&
                bl == other.bl &&
                br == other.br);
    }
};

int main(int argc, char** argv) {
    // The program expects exactly one argument: a path to telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    std::ifstream file(argv[1]);

    if (!file.is_open()) {
        std::cerr << "Error: Fail to open input file!" << std::endl;
        return 1;
    }

    const int ticksPerRevolution {1024};
    const float wheelRadiusM {0.3f}, wheelbaseM {1.0f};
    float x {0.0f}, y {0.0f}, theta {0.0f};
    Tick curTick {0,0,0,0,0};
    Tick prevTick {0,0,0,0,0};
    int i = 0;

    while (!file.eof()) {
        DEBUG("++++++++++++++++++ Start iteration [" << i << "] ++++++++++++++++++");

        if (curTick == prevTick && i > 1) {
            std::cerr << "Error: Duplicate values for state!" << std::endl;
        } else {
            prevTick = curTick;
        }

        DEBUG("<< Previous tick [" << prevTick.ts << ";"
                                   << prevTick.fl << ";"
                                   << prevTick.fr << ";"
                                   << prevTick.bl << ";"
                                   << prevTick.br << "]"
        );
        file >> curTick.ts
             >> curTick.fl
             >> curTick.fr
             >> curTick.bl
             >> curTick.br;
        DEBUG(">> Current tick [" << curTick.ts << ";"
                                  << curTick.fl << ";"
                                  << curTick.fr << ";"
                                  << curTick.bl << ";"
                                  << curTick.br << "]"
        );

        if (curTick != prevTick) {
            //-- Calc ticks delta
            long dFl = curTick.fl - prevTick.fl;
            long dFr = curTick.fr - prevTick.fr;
            long dBl = curTick.bl - prevTick.bl;
            long dBr = curTick.br - prevTick.br;

            float dLeft = (dFl + dBl) / 2;
            float dRight = (dFr + dBr) / 2;
            //-- Convert tick to meters
            float distancePerTick = 2 * M_PI * wheelRadiusM / ticksPerRevolution;
            float dLm = dLeft * distancePerTick;
            float dRm = dRight * distancePerTick;

            float centrDist = (dLm + dRm) / 2;
            float dtheta = (dRm - dLm) / wheelbaseM;

            x += centrDist * cos(theta + dtheta / 2);
            y += centrDist * sin(theta + dtheta / 2);
            theta += dtheta;
            DEBUG("timestamp [" << curTick.ts << "]; x [" << x << "]; y [" << y << "]; theta [" << theta << "]");
            std::cout << curTick.ts << " " << x << " " << y << " " << " " << theta << std::endl;
        }

        DEBUG("===================== End iteration [" << i << "] ====================");
        i++;
    }

    return 0;
}
