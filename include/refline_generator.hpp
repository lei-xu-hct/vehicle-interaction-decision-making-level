#pragma once 

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// 定义二维点结构
struct Point {
    double x;
    double y;

    Point(double x = 0, double y = 0) : x(x), y(y) {}

    // 计算两点之间的距离
    double distanceTo(const Point& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    // 点与向量相减
    Point operator-(const Point& other) const { return Point(x - other.x, y - other.y); }

    // 点与向量相加
    Point operator+(const Point& other) const { return Point(x + other.x, y + other.y); }

    // 点与标量相乘
    Point operator*(double scalar) const { return Point(x * scalar, y * scalar); }

    // 点积
    double dot(const Point& other) const { return x * other.x + y * other.y; }

    // 叉积
    double cross(const Point& other) const { return x * other.y - y * other.x; }

    // 向量长度
    double length() const { return std::sqrt(x * x + y * y); }

    // 归一化向量
    Point normalized() const {
        double len = length();
        return Point(x / len, y / len);
    }

    // 垂直向量（逆时针旋转90度）
    Point perpendicular() const { return Point(-y, x); }
};

// SL 转换基类
class XYSLConverter {
  public:
    explicit XYSLConverter(const std::vector<Point>& refline) : refline_(refline) {}

    XYSLConverter() = default;

    ~XYSLConverter() = default;

    const std::vector<Point>& refline() const { return refline_; }

    // 将点 (x, y) 转换为 SL 坐标
    void convertToSL(const Point& point, double& s, double& l, Point* refline_dir = nullptr) const {
        double minDistance = std::numeric_limits<double>::max();
        Point closest_point;
        size_t closestSegmentIndex = 0;

        // 遍历参考线的每一段，找到最近的投影点
        for (size_t i = 0; i < refline_.size() - 1; ++i) {
            const Point& p1 = refline_[i];
            const Point& p2 = refline_[i + 1];

            // 计算点到线段的投影
            Point segmentVector = p2 - p1;
            Point pointVector = point - p1;
            double t = pointVector.dot(segmentVector) / segmentVector.dot(segmentVector);

            // 限制 t 在 [0, 1] 范围内
            t = std::max(0.0, std::min(1.0, t));
            Point projection = p1 + segmentVector * t;

            // 计算点到投影点的距离
            double distance = point.distanceTo(projection);
            if (distance < minDistance) {
                minDistance = distance;
                closest_point = projection;
                closestSegmentIndex = i;
            }
        }

        // 计算纵向距离 s
        s = 0.0;
        for (size_t i = 0; i < closestSegmentIndex; ++i) {
            s += refline_[i].distanceTo(refline_[i + 1]);
        }
        s += refline_[closestSegmentIndex].distanceTo(closest_point);

        // 计算横向距离 l
        Point reference_dir = refline_[closestSegmentIndex + 1] - refline_[closestSegmentIndex];
        Point point_dir = point - closest_point;
        // l = point_dir.cross(reference_dir.normalized());
        const auto reference_dir_norm = reference_dir.normalized();
        l = reference_dir_norm.cross(point_dir);

        if (refline_dir != nullptr) {
            *refline_dir = reference_dir;
        }
    }

    // 将 SL 坐标 (s, l) 转换为 (x, y) 坐标
    void convertToXY(const double s, const double l, Point& point) const {
        double accumulatedLength = 0.0;
        size_t segmentIndex = 0;

        // 找到 s 对应的线段
        for (size_t i = 0; i < refline_.size() - 1; ++i) {
            double segmentLength = refline_[i].distanceTo(refline_[i + 1]);
            if (accumulatedLength + segmentLength >= s) {
                segmentIndex = i;
                break;
            }
            accumulatedLength += segmentLength;
        }

        // 计算投影点
        const Point& p1 = refline_[segmentIndex];
        const Point& p2 = refline_[segmentIndex + 1];
        double t = (s - accumulatedLength) / p1.distanceTo(p2);
        Point projection = p1 + (p2 - p1) * t;

        // 计算垂直方向
        Point segmentDir = (p2 - p1).normalized();
        Point perpendicularDir = segmentDir.perpendicular();

        // 计算最终点
        point = projection + perpendicularDir * l;
    }

  protected:
    std::vector<Point> refline_;  // 参考线（由一系列点组成）
};
