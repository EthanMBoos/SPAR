#pragma once

namespace worldfile_demo {

template <typename T>
struct Stamped {
  T value{};
  double stamp = -1.0;
};

template <typename T>
bool fresh(const Stamped<T>& stamped, double now, double max_age_sec) {
  const double age = now - stamped.stamp;
  return stamped.stamp >= 0.0 && age >= 0.0 && age <= max_age_sec;
}

}  // namespace worldfile_demo
