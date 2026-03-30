#include <polygon_base/regular_polygon.hpp>
#include <cmath>
#include <pluginlib/class_list_macros.hpp> // 必須包含這個來匯出插件

namespace polygon_plugins {
  // 正方形實作
  class Square : public polygon_base::RegularPolygon {
    public:
      void initialize(double side_length) override { side_length_ = side_length; }
      double area() override { return side_length_ * side_length_; }
    protected:
      double side_length_;
  };

  // 三角形實作
  class Triangle : public polygon_base::RegularPolygon {
    public:
      void initialize(double side_length) override { side_length_ = side_length; }
      double area() override { return 0.5 * side_length_ * (sqrt(3)/2 * side_length_); }
    protected:
      double side_length_;
  };
}

// 註冊插件：讓系統知道這兩個類別可以被動態載入
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)