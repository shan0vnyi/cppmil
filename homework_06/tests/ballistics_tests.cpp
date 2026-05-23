#include "ballistics.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

TEST(Ballistics, ComputesKnownDropPoint)
{
  const BallisticsInput input{.drone_x = 100.0,
                              .drone_y = 100.0,
                              .drone_z = 100.0,
                              .target_x = 200.0,
                              .target_y = 200.0,
                              .attack_speed_mps = 10.0,
                              .acceleration_path_m = 10.0,
                              .ammo_name = "VOG-17"};

  const auto solution = computeDropSolution(input);
  EXPECT_NEAR(solution->drop_x, 173.759, 0.01);
  EXPECT_NEAR(solution->drop_y, 173.759, 0.01);
}

TEST(Ballistics, ComputesKnownDropPointWithIntermediatePoint)
{
  const BallisticsInput input{.drone_x = 180.0,
                              .drone_y = 180.0,
                              .drone_z = 100.0,
                              .target_x = 200.0,
                              .target_y = 200.0,
                              .attack_speed_mps = 10.0,
                              .acceleration_path_m = 10.0,
                              .ammo_name = "VOG-17"};

  const auto solution = computeDropSolution(input);
  EXPECT_EQ(solution->requires_maneuver, true);
  EXPECT_NEAR(solution->drop_x, 173.759, 0.01);
  EXPECT_NEAR(solution->drop_y, 173.759, 0.01);
  EXPECT_NEAR(solution->intermediate_x, 166.688, 0.01);
  EXPECT_NEAR(solution->intermediate_y, 166.688, 0.01);
}

TEST(Ballistics, UnknownAmmo)
{
  const BallisticsInput input{.drone_x = 100.0,
                              .drone_y = 100.0,
                              .drone_z = 100.0,
                              .target_x = 200.0,
                              .target_y = 200.0,
                              .attack_speed_mps = 10.0,
                              .acceleration_path_m = 10.0,
                              .ammo_name = "TM-72"};

  testing::internal::CaptureStderr();
  const auto solution = computeDropSolution(input);
  std::string cerrOut = testing::internal::GetCapturedStderr();
  EXPECT_EQ(solution, std::nullopt);
  EXPECT_THAT(cerrOut.c_str(), testing::StartsWith("error: undefined ammo"));
}

TEST(Ballistics, IncorrectFlightAltitude)
{
  const BallisticsInput input{.drone_x = 100.0,
                              .drone_y = 100.0,
                              .drone_z = 0.0,
                              .target_x = 200.0,
                              .target_y = 200.0,
                              .attack_speed_mps = 10.0,
                              .acceleration_path_m = 10.0,
                              .ammo_name = "VOG-17"};

  testing::internal::CaptureStderr();
  const auto solution = computeDropSolution(input);
  std::string cerrOut = testing::internal::GetCapturedStderr();
  EXPECT_EQ(solution, std::nullopt);
  EXPECT_THAT(cerrOut.c_str(), testing::StartsWith("error: incorrect flight altitude"));
}

TEST(Ballistics, InputFileDoesNotExists)
{
  std::string doesNotExistsFile = "homework_06/data/does_not_exists.txt";
  testing::internal::CaptureStderr();
  const auto input = readBallisticsInput(doesNotExistsFile);
  std::string cerrOut = testing::internal::GetCapturedStderr();
  EXPECT_EQ(input, std::nullopt);
  EXPECT_EQ(cerrOut, "error: fail to open input file!\n");
}
