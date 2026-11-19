// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/interrupt.h"

#include <csignal>
#include <cstdlib>

#include "gtest/gtest.h"

//! An interrupt asks the optimization to stop and lets the program go on.
TEST(Interrupt, SetsTheStopFlag) {
  std::atomic<bool>& stop = g2o::stopOnInterrupt();
  stop = false;

  ASSERT_EQ(std::raise(SIGINT), 0);

  EXPECT_TRUE(stop);
  std::signal(SIGINT, SIG_DFL);
}

//! A program that ignores repeated interrupts ends with the third one.
TEST(InterruptDeathTest, ExitsAtTheThirdInterrupt) {
  EXPECT_EXIT(
      {
        g2o::stopOnInterrupt();
        std::raise(SIGINT);
        std::raise(SIGINT);
        std::raise(SIGINT);
        std::exit(EXIT_SUCCESS);
      },
      ::testing::ExitedWithCode(EXIT_FAILURE), "");
}
