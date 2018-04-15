#pragma once
#include <thread>
#include <atomic>
//adot from https://ideone.com/Z7zldb
template <typename Index, typename Callable>
void ParallelFor(Index start, Index end, Callable func) {
  // Estimate number of threads in the pool
  const static unsigned nb_threads_hint = std::thread::hardware_concurrency();
  const static unsigned nb_threads =
      (nb_threads_hint == 0u ? 8u : nb_threads_hint);

  // Size of a slice for the range functions
  Index n = end - start + 1;
  Index slice = (Index)std::round(n / static_cast<double>(nb_threads));
  slice = std::max(slice, Index(1));

  // [Helper] Inner loop
  auto launchRange = [&func](int k1, int k2) {
    for (Index k = k1; k < k2; k++) {
      func(k);
    }
  };

  // Create pool and launch jobs
  std::vector<std::thread> pool;
  pool.reserve(nb_threads);
  Index i1 = start;
  Index i2 = std::min(start + slice, end);
  for (unsigned i = 0; i + 1 < nb_threads && i1 < end; ++i) {
    pool.emplace_back(launchRange, i1, i2);
    i1 = i2;
    i2 = std::min(i2 + slice, end);
  }
  if (i1 < end) {
    pool.emplace_back(launchRange, i1, end);
  }

  // Wait for jobs to finish
  for (std::thread &t : pool) {
    if (t.joinable()) {
      t.join();
    }
  }
}
inline uint32_t FloatToBits(float f) {
  uint32_t ui;
  memcpy(&ui, &f, sizeof(float));
  return ui;
}

inline float BitsToFloat(uint32_t ui) {
  float f;
  memcpy(&f, &ui, sizeof(uint32_t));
  return f;
}
class AtomicFloat {
 public:
  explicit AtomicFloat(float v = 0) { bits = FloatToBits(v); }
  operator float() const { return BitsToFloat(bits); }
  float operator=(float v) {
    bits = FloatToBits(v);
    return v;
  }
  void Add(float v) {
    // all thread wanna add this AtomicFloat with some float number v
    unsigned int oldBits = bits, newBits;
    do {
      newBits = FloatToBits(BitsToFloat(oldBits) + v);
    } while (!bits.compare_exchange_weak(oldBits, newBits));
  }

 private:
  std::atomic<unsigned int> bits;
};
