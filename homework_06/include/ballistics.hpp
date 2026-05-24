#pragma once

#include <string>
#include <optional>

struct Ammo {
  float mass{0.0f};
  float drag{0.0f};
  float lift{0.0f};
  std::string name{};
  bool defined{false};
};

struct BallisticsInput {
  double drone_x{0.0};
  double drone_y{0.0};
  double drone_z{0.0};
  double target_x{0.0};
  double target_y{0.0};
  double attack_speed_mps{0.0};
  double acceleration_path_m{0.0};
  std::string ammo_name{};
  bool filled{false};

  bool is_filled()
  {
    BallisticsInput dbi{};
    if (drone_x != dbi.drone_x && drone_y != dbi.drone_y && drone_z != dbi.drone_z && target_x != dbi.target_x &&
        target_y != dbi.target_y && attack_speed_mps > dbi.attack_speed_mps && acceleration_path_m > dbi.acceleration_path_m) {
      filled = true;
    }

    return filled;
  }
};

struct DropSolution {
  double drop_x{0.0f};
  double drop_y{0.0f};
  double intermediate_x{0.0f};
  double intermediate_y{0.0f};
  double flight_time_s{0.0f};
  double lead_distance_m{0.0f};
  bool requires_maneuver{false};
};

std::optional<BallisticsInput> readBallisticsInput(const std::string& path);
std::optional<DropSolution> computeDropSolution(const BallisticsInput& input);
void printSolution(const DropSolution& solution);