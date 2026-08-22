#pragma once

#include <chrono>
#include <ctime>

namespace autonomy {

/**
 * Monotonic, absolute-deadline control-loop timer.
 *
 * This deliberately mirrors the reference Runner timer: sleep until shortly
 * before the deadline, busy-spin through the final window, and reset the
 * schedule from the current time after an overrun rather than accumulating
 * stale ticks.
 */
class LoopTimer final {
public:
  explicit LoopTimer(
    std::chrono::nanoseconds period,
    std::chrono::nanoseconds busy_spin = std::chrono::microseconds(400));

  /** Arms the first deadline one control period from now. */
  void init();

  /** Waits for the current deadline and arms the next one. */
  void rest();

private:
  [[nodiscard]] static timespec nowMonotonic();
  [[nodiscard]] static timespec add(
    timespec value, std::chrono::nanoseconds duration);
  [[nodiscard]] static timespec subtract(
    timespec value, std::chrono::nanoseconds duration);
  [[nodiscard]] static int compare(const timespec & lhs, const timespec & rhs);
  static void sleepUntil(const timespec & deadline);

  std::chrono::nanoseconds period_{};
  std::chrono::nanoseconds busy_spin_{};
  timespec next_tick_{};
};

}  // namespace autonomy
