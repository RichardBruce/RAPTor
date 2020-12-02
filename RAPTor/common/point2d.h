#pragma once

template<class T>
class point2d
{
    public:
        point2d(const T x, const T y) : x(x), y(y) {  }

        point2d<T>  operator-() const { return point2d<T>(-x, -y); }

        bool operator!=(const point2d<T> &rhs) const { return (x != rhs.x) || (y != rhs.y); }
        bool operator==(const point2d<T> &rhs) const { return (x == rhs.x) && (y == rhs.y); }

        point2d<T>& operator+=(const point2d<T> &rhs) { x += rhs.x; y += rhs.y; return *this; }
        point2d<T>& operator-=(const point2d<T> &rhs) { x -= rhs.x; y -= rhs.y; return *this; }
        point2d<T>& operator*=(const point2d<T> &rhs) { x *= rhs.x; y *= rhs.y; return *this; }
        point2d<T>& operator/=(const point2d<T> &rhs) { x /= rhs.x; y /= rhs.y; return *this; }

        point2d<T>& operator+=(const T rhs) { x += rhs; y += rhs; return *this; }
        point2d<T>& operator-=(const T rhs) { x -= rhs; y -= rhs; return *this; }
        point2d<T>& operator*=(const T rhs) { x *= rhs; y *= rhs; return *this; }
        point2d<T>& operator/=(const T rhs) { x /= rhs; y /= rhs; return *this; }
    
        T   x;
        T   y;
};

/* Degugging */
template<class T>
inline std::ostream& operator<<(std::ostream &os, const point2d<T> &p)
{
    return os << p.x << ", " << p.y;
}

template<class T>
inline point2d<T> operator+(const point2d<T> &lhs, const point2d<T> &rhs)
{
   return point2d<T>(lhs.x + rhs.x, lhs.y + rhs.y);
}

template<class T>
inline point2d<T> operator-(const point2d<T> &lhs, const point2d<T> &rhs)
{
   return point2d<T>(lhs.x - rhs.x, lhs.y - rhs.y);
}

template<class T>
inline point2d<T> operator*(const point2d<T> &lhs, const point2d<T> &rhs)
{
   return point2d<T>(lhs.x * rhs.x, lhs.y * rhs.y);
}

template<class T>
inline point2d<T> operator/(const point2d<T> &lhs, const point2d<T> &rhs)
{
   return point2d<T>(lhs.x / rhs.x, lhs.y / rhs.y);
}

template<class T>
inline T dot_product(const point2d<T> &rhs, const point2d<T> &lhs)
{
    return (rhs.x * lhs.x) + (rhs.y * lhs.y);
}

template<class T>
inline point2d<T> operator+(const point2d<T> &lhs, const long rhs)
{
   return point2d<T>(lhs.x + rhs, lhs.y + rhs);
}

template<class T>
inline point2d<T> operator-(const point2d<T> &lhs, const long rhs)
{
   return point2d<T>(lhs.x - rhs, lhs.y - rhs);
}

template<class T>
inline point2d<T> operator*(const point2d<T> &lhs, const long rhs)
{
   return point2d<T>(lhs.x * rhs, lhs.y * rhs);
}

template<class T>
inline point2d<T> operator/(const point2d<T> &lhs, const long rhs)
{
   return point2d<T>(lhs.x / rhs, lhs.y / rhs);
}

template<class T>
inline T magnitude_sq(const point2d<T> &lhs)
{
    return (lhs.x * lhs.x) + (lhs.y * lhs.y);
}

inline long winding(const point2d<long> &a, const point2d<long> &b, const point2d<long> &c)
{
    const point2d ab(b - a);
    const point2d bc(c - b);
    return (ab.x * bc.y) - (ab.y * bc.x);
}
