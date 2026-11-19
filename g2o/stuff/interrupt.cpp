// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/interrupt.h>
#include <g2o/stuff/logger.h>
#include <ng-log/experimental/signal_safe_log.h>

#include <atomic>
#include <csignal>
#include <cstdlib>

G2O_START_NAMESPACE

namespace {

// A signal handler may only touch atomics that need no lock.
static_assert(std::atomic<bool>::is_always_lock_free,
              "the stop flag needs a lock-free atomic bool, which a signal "
              "handler can set; this platform lacks it");
static_assert(std::atomic<int>::is_always_lock_free,
              "counting interrupts needs a lock-free atomic int, which a "
              "signal handler can update; this platform lacks it");

//! Interrupts after which the program exits instead of waiting.
constexpr int ForcedExitInterrupts = 3;

std::atomic<bool> stopFlag{false};
std::atomic<int> interrupts{0};

extern "C" void handleInterrupt(int /*signal*/) {
  stopFlag = true;
  if (++interrupts == ForcedExitInterrupts) {
    ngx::Log(ngx::SignalSafe | ngx::Warning, "forcing exit");
    // Unlike exit, _Exit is safe to call from a signal handler.
    std::_Exit(EXIT_FAILURE);
  }
}

}  // namespace

std::atomic<bool>& stopOnInterrupt() {
  interrupts = 0;
  std::signal(SIGINT, handleInterrupt);
  return stopFlag;
}

G2O_END_NAMESPACE
