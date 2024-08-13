module;

#include <cassert>
#include <ostream>
#include <random>

export module math;

export using scalar = float;
export constexpr auto INF = std::numeric_limits<scalar>::infinity();

export scalar randreal(scalar x, scalar y) {
  static std::random_device rd;
  static std::mt19937 e2(rd());
  assert(x <= y);
  return std::uniform_real_distribution<>(x, y)(e2);
}

export class vec {
public:
  scalar x = 0, y = 0, z = 0;

  vec() = default;
  constexpr vec(scalar n) : x{n}, y{n}, z{n} {}
  constexpr vec(scalar x, scalar y, scalar z) : x{x}, y{y}, z{z} {}
  constexpr vec(const vec &v) : x{v.x}, y{v.y}, z{v.z} {};

  vec operator-() const { return vec(-x, -y, -z); }
  vec operator+(const vec &v) const { return vec(x + v.x, y + v.y, z + v.z); }
  vec operator-(const vec &v) const { return vec(x - v.x, y - v.y, z - v.z); }
  vec operator*(const vec &v) const { return vec(x * v.x, y * v.y, z * v.z); }
  vec operator/(const vec &v) const { return vec(x / v.x, y / v.y, z / v.z); }

  void operator+=(const vec &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  void operator-=(const vec &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }

  void operator*=(const vec &v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
  }

  void operator/=(const vec &v) {
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

  bool operator==(const vec &) const = default;
  bool operator!=(const vec &) const = default;
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
  };

  scalar dot(const vec &v) const { return x * v.x + y * v.y + z * v.z; }
  scalar norm() const { return std::sqrt(x * x + y * y + z * z); }

  vec normalize() const {
    scalar n = norm();
    return n == 0 ? vec(x, y, z) : vec(x / n, y / n, z / n);
  }

  vec min(const vec &v) const {
    return vec(std::min(x, v.x), std::min(y, v.y), std::min(z, v.z));
  }

  vec max(const vec &v) const {
    return vec(std::max(x, v.x), std::max(y, v.y), std::max(z, v.z));
  }

  vec cross(const vec &v) const {
    return vec(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  // Should totally precompute the rotation matrix. Maybe make transformation
  // class? TODO
  vec rotate(scalar a, scalar b, scalar y) const {
    return vec((cos(b) * cos(y) * this->x) +
                   ((sin(a) * sin(b) * cos(y) - cos(a) * sin(y)) * this->y) +
                   ((cos(a) * sin(b) * cos(y) + sin(a) * sin(y)) * this->z),
               (cos(b) * sin(y) * this->x) +
                   ((sin(a) * sin(b) * sin(y) + cos(a) * cos(y)) * this->y) +
                   ((cos(a) * sin(b) * sin(y) - sin(a) * cos(y)) * this->z),
               (-sin(b) * this->x) + (sin(a) * cos(b) * this->y) +
                   (cos(a) * cos(b) * this->z));
  }
  vec rotate(vec v) const { return vec(x, y, z).rotate(v.x, v.y, v.z); }

  vec floor() {
    x = (int)x;
    y = (int)y;
    z = (int)z;
    return *this;
  }

  vec pow(scalar n) {
    return vec(std::pow(x, n), std::pow(y, n), std::pow(z, n));
  }

  vec square() { return vec(x * x, y * y, z * z); }

  scalar sum() { return x + y + z; }

  vec clamp(scalar lo, scalar hi) {
    x = std::min(std::max(x, lo), hi);
    y = std::min(std::max(y, lo), hi);
    z = std::min(std::max(z, lo), hi);
    return *this;
  }

  vec reflect_around(const vec &normal) const;
};

export vec operator*(scalar n, const vec &v) { return vec(v.x * n, v.y * n, v.z * n); }
export vec operator*(const vec &v, scalar n) { return vec(v.x * n, v.y * n, v.z * n); }
export vec operator/(scalar n, const vec &v) { return vec(v.x / n, v.y / n, v.z / n); }
export vec operator/(const vec &v, scalar n) { return vec(v.x / n, v.y / n, v.z / n); }

vec vec::reflect_around(const vec &normal) const {
  return 2 * this->dot(normal) * normal - *this;
}

std::ostream &operator<<(std::ostream &os, const vec &v) {
  return os << "[" << v.x << "," << v.y << "," << v.z << "]\n";
}
