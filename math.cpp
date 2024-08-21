module;

#include <cassert>
#include <ostream>
#include <random>
#include <utility>

export module math;

export using scalar = float;
export constexpr auto INF = std::numeric_limits<scalar>::infinity();

export scalar randreal(scalar x, scalar y) {
  static std::random_device rd;
  static std::mt19937 e2(rd());
  assert(x <= y);
  return std::uniform_real_distribution<>(x, y)(e2);
}

export template <typename T> class vector_ {
public:
  scalar x = 0, y = 0, z = 0;

  vector_() = default;
  constexpr vector_(scalar n) : x{n}, y{n}, z{n} {}
  constexpr vector_(scalar x, scalar y, scalar z) : x{x}, y{y}, z{z} {}
  constexpr vector_(const vector_ &v) : x{v.x}, y{v.y}, z{v.z} {};

  vector_ operator-() const { return vector_(-x, -y, -z); }
  vector_ operator+(const vector_ &v) const {
    return vector_(x + v.x, y + v.y, z + v.z);
  }
  vector_ operator-(const vector_ &v) const {
    return vector_(x - v.x, y - v.y, z - v.z);
  }
  vector_ operator*(const vector_ &v) const {
    return vector_(x * v.x, y * v.y, z * v.z);
  }
  vector_ operator/(const vector_ &v) const {
    return vector_(x / v.x, y / v.y, z / v.z);
  }

  void operator+=(const vector_ &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  void operator-=(const vector_ &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }

  void operator*=(const vector_ &v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
  }

  void operator/=(const vector_ &v) {
    x /= v.x;
    y /= v.y;
    z /= v.z;
  }

  void operator*=(scalar n) {
    x *= n;
    y *= n;
    z *= n;
  }

  void operator/=(scalar n) {
    x /= n;
    y /= n;
    z /= n;
  }

  void operator+=(scalar n) {
    x += n;
    y += n;
    z += n;
  }

  void operator-=(scalar n) {
    x -= n;
    y -= n;
    z -= n;
  }

  bool operator==(const vector_ &) const = default;
  bool operator!=(const vector_ &) const = default;
  bool operator<(scalar n) const { return x < n && y < n && z < n; }

  scalar operator[](int n) const {
    switch (n) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    }
    assert(false);
    std::unreachable();
  };

  scalar dot(const vector_ &v) const { return x * v.x + y * v.y + z * v.z; }
  scalar norm() const { return std::sqrt(x * x + y * y + z * z); }

  vector_ normalize() const {
    scalar n = norm();
    return n == 0 ? vector_(x, y, z) : vector_(x / n, y / n, z / n);
  }

  vector_ min(const vector_ &v) const {
    return vector_(std::min(x, v.x), std::min(y, v.y), std::min(z, v.z));
  }

  vector_ max(const vector_ &v) const {
    return vector_(std::max(x, v.x), std::max(y, v.y), std::max(z, v.z));
  }

  vector_ cross(const vector_ &v) const {
    return vector_(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  // Should totally precompute the rotation matrix. Maybe make transformation
  // class? TODO
  vector_ rotate(scalar a, scalar b, scalar y) const {
    return vector_(
        (cos(b) * cos(y) * this->x) +
            ((sin(a) * sin(b) * cos(y) - cos(a) * sin(y)) * this->y) +
            ((cos(a) * sin(b) * cos(y) + sin(a) * sin(y)) * this->z),
        (cos(b) * sin(y) * this->x) +
            ((sin(a) * sin(b) * sin(y) + cos(a) * cos(y)) * this->y) +
            ((cos(a) * sin(b) * sin(y) - sin(a) * cos(y)) * this->z),
        (-sin(b) * this->x) + (sin(a) * cos(b) * this->y) +
            (cos(a) * cos(b) * this->z));
  }
  vector_ rotate(vector_ v) const {
    return vector_(x, y, z).rotate(v.x, v.y, v.z);
  }

  vector_ floor() {
    x = (int)x;
    y = (int)y;
    z = (int)z;
    return *this;
  }

  vector_ pow(scalar n) {
    return vector_(std::pow(x, n), std::pow(y, n), std::pow(z, n));
  }

  vector_ square() { return vector_(x * x, y * y, z * z); }

  scalar sum() { return x + y + z; }

  vector_ clamp(scalar lo, scalar hi) {
    x = std::min(std::max(x, lo), hi);
    y = std::min(std::max(y, lo), hi);
    z = std::min(std::max(z, lo), hi);
    return *this;
  }
};

export template <typename T>
vector_<T> operator*(scalar n, const vector_<T> &v) {
  return vector_<T>(v.x * n, v.y * n, v.z * n);
}
export template <typename T>
vector_<T> operator*(const vector_<T> &v, scalar n) {
  return vector_<T>(v.x * n, v.y * n, v.z * n);
}
export template <typename T>
vector_<T> operator/(scalar n, const vector_<T> &v) {
  return vector_<T>(v.x / n, v.y / n, v.z / n);
}
export template <typename T>
vector_<T> operator/(const vector_<T> &v, scalar n) {
  return vector_<T>(v.x / n, v.y / n, v.z / n);
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const vector_<T> &v) {
  return os << "[" << v.x << "," << v.y << "," << v.z << "]\n";
}

export using vec = vector_<scalar>;
export using veci = vector_<int>;
