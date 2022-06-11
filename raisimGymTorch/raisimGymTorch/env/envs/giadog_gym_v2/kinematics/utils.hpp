#include <algorithm>

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}