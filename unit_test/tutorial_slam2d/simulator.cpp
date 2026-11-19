// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "simulator.h"

#include <functional>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::Each;
using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::Truly;

/**
 * The landmarks a pose observes outlive the simulation, which keeps them as
 * the landmarks it returns.
 */
TEST(TutorialSimulator, PosesReferToTheReturnedLandmarks) {
  constexpr int Poses = 50;
  g2o::tutorial::Simulator simulator;
  simulator.simulate(Poses);

  const auto& landmarks = simulator.landmarks();
  ASSERT_THAT(landmarks, Not(IsEmpty()));
  const auto* first = landmarks.data();
  const auto* last = landmarks.data() + landmarks.size();
  for (const auto& pose : simulator.poses()) {
    EXPECT_THAT(
        pose.landmarks,
        Each(Truly([first, last](const g2o::tutorial::Simulator::Landmark* l) {
          return !std::less<>{}(l, first) && std::less<>{}(l, last);
        })))
        << "pose " << pose.id;
  }
}
