#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <kstone/core/log.hpp>
#include <string_view>

namespace kstone_ros {
/// Fixed-size, null-terminated string that never allocates, for real-time hand-off.
template <std::size_t N> struct Text {
  std::array<char, N> bytes{};
  /// Copy `value`, truncating to N - 1 bytes; returns true when it was truncated.
  bool set(std::string_view value) noexcept {
    const auto size = value.size() < N - 1 ? value.size() : N - 1;
    std::memcpy(bytes.data(), value.data(), size);
    bytes[size] = 0;
    return size != value.size();
  }
  const char *c_str() const noexcept { return bytes.data(); }
};

/// Lock-free triple buffer that hands the latest value from one producer to one consumer.
/// Exactly one serialized producer and one consumer. Each retains its slot
/// until the next exchange. A stalled consumer never pins the producer's slot.
template <class T> class Latest {
public:
  std::array<T, 3> slots;
  /// Producer: slot to fill before publish().
  T &write() noexcept { return slots[back_]; }
  /// Producer: make the written slot the latest value.
  void publish() noexcept {
    back_ = middle_.exchange(back_ | dirty, std::memory_order_acq_rel) & mask;
  }
  /// Consumer: switch read() to the latest value; false when nothing new was published.
  bool acquire() noexcept {
    if (!(middle_.load(std::memory_order_acquire) & dirty)) return false;
    front_ = middle_.exchange(front_, std::memory_order_acq_rel) & mask;
    return true;
  }
  /// Consumer: value taken by the last acquire().
  const T &read() const noexcept { return slots[front_]; }
private:
  static constexpr unsigned dirty = 4, mask = 3;
  unsigned back_{2}, front_{0};
  std::atomic<unsigned> middle_{1};
};

/// Lock-free bounded queue for one producer and one consumer.
template <class T, std::size_t N> class Fifo {
public:
  /// Append `value`; false when the queue is full.
  bool push(const T &value) noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) == N) return false;
    slots_[write % N] = value;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }
  /// Remove the oldest value into `value`; false when empty.
  bool pop(T &value) noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    value = slots_[read % N];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }
  /// Copy the oldest value without removing it; false when empty.
  bool peek(T &value) const noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    value = slots_[read % N];
    return true;
  }
private:
  std::array<T, N> slots_{};
  std::atomic<std::uint64_t> write_{0}, read_{0};
};

/// Log record copied out of the engine, with bounded text.
struct CopiedLog {
  kstone::LogLevel level{};
  std::int64_t time_ns{};
  Text<64> component;
  Text<512> message;
};
/// Log sink buffering up to 256 records for the ROS thread.
/// Multi-producer-safe including driver/plugin logs. One try only: contention
/// drops a record, never spins or waits on the ROS consumer.
class BufferedLog final : public kstone::LogSink {
public:
  void write(const kstone::LogRecord &record) noexcept override {
    if (busy_.test_and_set(std::memory_order_acquire)) { ++dropped; return; }
    if (write_ - read_ == slots_.size()) {
      ++dropped;
    } else {
      auto &slot = slots_[write_++ % slots_.size()];
      slot.level = record.level;
      slot.time_ns = record.time_ns;
      const bool a = slot.component.set(record.component);
      const bool b = slot.message.set(record.message);
      if (a || b) ++truncated;
    }
    busy_.clear(std::memory_order_release);
  }
  /// Remove the oldest record; false when empty or a producer holds the buffer.
  bool pop(CopiedLog &record) noexcept {
    if (busy_.test_and_set(std::memory_order_acquire)) return false;
    const bool available = read_ != write_;
    if (available) record = slots_[read_++ % slots_.size()];
    busy_.clear(std::memory_order_release);
    return available;
  }
  /// Records lost to a full buffer or contention, and records whose text was truncated.
  std::atomic<std::uint64_t> dropped{0}, truncated{0};
private:
  std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
  std::array<CopiedLog, 256> slots_{};
  std::uint64_t write_{0}, read_{0};
};
} // namespace kstone_ros
