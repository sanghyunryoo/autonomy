#include "autonomy/core/loop_timer.hpp"

#include <algorithm>
#include <cerrno>
#include <stdexcept>

namespace autonomy {
namespace {

constexpr long kNanosecondsPerSecond = 1000000000L;

timespec normalize(timespec value)
{
  while (value.tv_nsec >= kNanosecondsPerSecond) {
    value.tv_nsec -= kNanosecondsPerSecond;
    ++value.tv_sec;
  }
  while (value.tv_nsec < 0) {
    value.tv_nsec += kNanosecondsPerSecond;
    --value.tv_sec;
  }
  return value;
}

}  // namespace

LoopTimer::LoopTimer(
  const std::chrono::nanoseconds period,
  const std::chrono::nanoseconds busy_spin)
: period_(period), busy_spin_(std::max(std::chrono::nanoseconds::zero(), busy_spin))
{
  if (period_ <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("control-loop timer period must be positive");
  }
  init();
}

void LoopTimer::init()
{
  next_tick_ = add(nowMonotonic(), period_);
}

void LoopTimer::rest()
{
  const timespec now = nowMonotonic();
  if (compare(now, next_tick_) >= 0) {
    // The work exceeded the period. Rebase now so delayed historical ticks
    // are never executed back-to-back.
    next_tick_ = add(now, period_);
    return;
  }

  const timespec sleep_until = subtract(next_tick_, busy_spin_);
  if (compare(now, sleep_until) < 0) {
    sleepUntil(sleep_until);
  }

  while (compare(nowMonotonic(), next_tick_) < 0) {
    // Intentional: preserve the reference timer's final 400 us busy-spin.
  }
  next_tick_ = add(next_tick_, period_);
}

timespec LoopTimer::nowMonotonic()
{
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value;
}

timespec LoopTimer::add(const timespec value, const std::chrono::nanoseconds duration)
{
  timespec result = value;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  const auto remainder = duration - seconds;
  result.tv_sec += seconds.count();
  result.tv_nsec += static_cast<long>(remainder.count());
  return normalize(result);
}

timespec LoopTimer::subtract(const timespec value, const std::chrono::nanoseconds duration)
{
  timespec result = value;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  const auto remainder = duration - seconds;
  result.tv_sec -= seconds.count();
  result.tv_nsec -= static_cast<long>(remainder.count());
  return normalize(result);
}

int LoopTimer::compare(const timespec & lhs, const timespec & rhs)
{
  if (lhs.tv_sec != rhs.tv_sec) return lhs.tv_sec < rhs.tv_sec ? -1 : 1;
  if (lhs.tv_nsec != rhs.tv_nsec) return lhs.tv_nsec < rhs.tv_nsec ? -1 : 1;
  return 0;
}

void LoopTimer::sleepUntil(const timespec & deadline)
{
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr) == EINTR) {
    // An absolute deadline does not need a remaining-duration calculation.
  }
}

}  // namespace autonomy
