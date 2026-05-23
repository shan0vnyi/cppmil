#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

#include "ballistics.hpp"

#include <algorithm>
#include <optional>
#include <array>
#include <cmath>
#define _USE_MATH_DEFINES
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <stdexcept>

// Global vars
const float g{9.81f};
const float goesToZero = 1e-6;
//-- Ammo params
const int AMMO_COUNT{5};
//-- Input file params count
const int INPUT_FIELDS_COUNT{8};

const std::array<std::string, AMMO_COUNT> ammoNames = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
float const ammoMass[AMMO_COUNT] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
float const ammoDrag[AMMO_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
float const ammoLift[AMMO_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

Ammo getAmmo(const std::string& name)
{
  Ammo ammo{};

  for (int i = 0; i < AMMO_COUNT; i++) {
    if (name.compare(ammoNames[i]) == 0) {
      ammo = {ammoMass[i], ammoDrag[i], ammoLift[i], name, true};
      break;
    }
  }

  return ammo;
}

float calcFallTime(const Ammo& ammo, const float& h, const float& speed)
{
  float a = ammo.drag * g * ammo.mass - 2 * pow(ammo.drag, 2) * ammo.lift * speed;
  float b = -3 * g * pow(ammo.mass, 2) + 3 * ammo.drag * ammo.lift * ammo.mass * speed;
  float m2 = pow(ammo.mass, 2);
  float c = 6 * m2 * h;
  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;
  float phi = acos((3 * q) / (2 * p) * sqrt(-3 / p));
  float t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

  return t;
}

float calcFallDistance(const Ammo& ammo, const float& t, const float& speed)
{
  float h = speed * t - pow(t, 2) * ammo.drag * speed / (2 * ammo.mass) +
            pow(t, 3) * (6 * ammo.drag * g * ammo.lift * ammo.mass - 6 * pow(ammo.drag, 2) * (pow(ammo.lift, 2) - 1) * speed) /
              (36 * pow(ammo.mass, 2)) +
            pow(t, 4) *
              (-6 * pow(ammo.drag, 2) * g * ammo.lift * (1 + pow(ammo.lift, 2) + pow(ammo.lift, 4)) * ammo.mass +
               3 * pow(ammo.drag, 3) * pow(ammo.lift, 2) * (1 + pow(ammo.lift, 2)) * speed +
               6 * pow(ammo.drag, 3) * pow(ammo.lift, 4) * (1 + pow(ammo.lift, 2)) * speed) /
              (36 * pow(1 + pow(ammo.lift, 2), 2) * pow(ammo.mass, 3)) +
            pow(t, 5) *
              (3 * pow(ammo.drag, 3) * g * pow(ammo.lift, 3) * ammo.mass -
               3 * pow(ammo.drag, 4) * pow(ammo.lift, 2) * (1 + pow(ammo.lift, 2)) * speed) /
              (36 * (1 + pow(ammo.lift, 2)) * pow(ammo.mass, 4));

  return h;
}

double calcDistanceBtwPoints(const float& x1, const float& y1, const float& x2, const float& y2)
{
  return hypot(x2 - x1, y2 - y1);
}

std::optional<BallisticsInput> readBallisticsInput(const std::string& path)
{
  std::ifstream file(path);

  if (!file.is_open()) {
    std::cerr << "error: fail to open input file!\n";
    return std::nullopt;
  }

  BallisticsInput input{};

  for (int i = 0; i < INPUT_FIELDS_COUNT; ++i) {
    switch (i) {
      case 0:
        file >> input.drone_x;
        break;
      case 1:
        file >> input.drone_y;
        break;
      case 2:
        file >> input.drone_z;
        break;
      case 3:
        file >> input.target_x;
        break;
      case 4:
        file >> input.target_y;
        break;
      case 5:
        file >> input.attack_speed_mps;
        break;
      case 6:
        file >> input.acceleration_path_m;
        break;
      case 7:
        file >> input.ammo_name;
        break;
    }

    if (file.eof() && i != (INPUT_FIELDS_COUNT - 1)) {
      std::cerr << "error: missing fields\n";
      return std::nullopt;
    }
    DEBUG("is EOF on iter [" << i << "]: [" << file.eof() << "]\n");
  }

  DEBUG(">> BallisticsInput [" << input.drone_x << ";" << input.drone_y << ";" << input.drone_z << ";" << input.target_x << ";"
                               << input.target_y << ";" << input.attack_speed_mps << ";" << input.acceleration_path_m << ";"
                               << input.ammo_name << "]\n");

  return input;
}

std::optional<DropSolution> computeDropSolution(const BallisticsInput& input)
{
  if (input.drone_z <= 0) {
    std::cerr << "error: incorrect flight altitude [" << input.drone_z << "]\n";
    return std::nullopt;
  }

  Ammo bomb = getAmmo(input.ammo_name);

  if (!bomb.defined) {
    std::cerr << "error: undefined ammo [" << input.ammo_name << "]\n";
    return std::nullopt;
  }

  DropSolution result{};
  float fallTime = calcFallTime(bomb, input.drone_z, input.attack_speed_mps);
  float fallDistance = calcFallDistance(bomb, fallTime, input.attack_speed_mps);
  std::cout << "Fall time is: [" << fallTime << "]; " << " Fall dist is: [" << fallDistance << "];\n";

  if (fallDistance < goesToZero) {
    std::cerr << "error: invalid horizontal fall distance [" << fallDistance << "]\n";
    return std::nullopt;
  }
  else {
    float drone_x{input.drone_x}, drone_y{input.drone_y};
    result.flight_time_s = fallTime;
    result.lead_distance_m = fallDistance;
    float flightDistance = calcDistanceBtwPoints(drone_x, drone_y, input.target_x, input.target_y);
    DEBUG("FD1 is [" << flightDistance << "]\n");
    // Dron and target have same coordinates
    if (flightDistance < goesToZero) {
      DEBUG("Flight distnace goes to zero\n");
      result.requires_maneuver = true;
      double degrees = 180.0;
      double radians = degrees * (M_PI / degrees);
      drone_x = result.intermediate_x = input.target_x + ((fallDistance + input.acceleration_path_m) * cos(radians));
      drone_y = result.intermediate_y = input.target_y + ((fallDistance + input.acceleration_path_m) * sin(radians));
      flightDistance = calcDistanceBtwPoints(drone_x, drone_y, input.target_x, input.target_y);
      DEBUG("FD2 is [" << flightDistance << "]\n");
    }
    // Calculate intermediate point, if needed
    // Comparing rounded to 3 points decimal
    if (std::round((fallDistance + input.acceleration_path_m) * 1000.f) / 1000.0f > std::round(flightDistance * 1000.0f) / 1000.0f) {
      DEBUG("Flight distnace less than fallD and accPath" << fallDistance << "+" << input.acceleration_path_m << "("
                                                          << (fallDistance + input.acceleration_path_m) << ") > " << flightDistance
                                                          << "\n");
      result.requires_maneuver = true;
      drone_x = result.intermediate_x =
        input.target_x - (input.target_x - drone_x) * (fallDistance + input.acceleration_path_m) / flightDistance;
      drone_y = result.intermediate_y =
        input.target_y - (input.target_y - drone_y) * (fallDistance + input.acceleration_path_m) / flightDistance;
      flightDistance = calcDistanceBtwPoints(drone_x, drone_y, input.target_x, input.target_y);
      DEBUG("FD3 is [" << flightDistance << "]\n");
    }

    float ratio = (flightDistance - fallDistance) / flightDistance;
    result.drop_x = drone_x + (input.target_x - drone_x) * ratio;
    result.drop_y = drone_y + (input.target_y - drone_y) * ratio;
  }

  DEBUG(">> DropSolution [ftime: " << result.flight_time_s << "; ldist: " << result.lead_distance_m
                                   << "; req_maneuver: " << result.requires_maneuver << "; interm_x: " << result.intermediate_x
                                   << "; interm_y: " << result.intermediate_y << "; drop_x: " << result.drop_x
                                   << "; drop_y: " << result.drop_y << "]\n");

  return result;
}

void printSolution(const DropSolution& solution)
{
  std::string outPath = "homework_06/data/output.txt";
  std::ofstream file(outPath);

  if (file.is_open()) {
    if (solution.requires_maneuver) {
      file << solution.intermediate_x << " " << solution.intermediate_y << " ";
    }

    file << solution.drop_x << " " << solution.drop_y;
    file.close();
  }
  //-- Also print solution to stdout
  std::cout << "drop x: " << solution.drop_x << '\n';
  std::cout << "drop y: " << solution.drop_y << '\n';
  std::cout << "intermediate x: " << solution.intermediate_x << '\n';
  std::cout << "intermediate y: " << solution.intermediate_y << '\n';
  std::cout << "flight time(seconds): " << solution.flight_time_s << '\n';
  std::cout << "lead distance(meters): " << solution.lead_distance_m << '\n';
  std::cout << "requires maneuver: " << solution.requires_maneuver << '\n';
}