#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base {
  class RegularPolygon {
    public:
      // pluginlib 要求必須有 initialize 函數，因為插件建構子不能帶參數
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}
    protected:
      RegularPolygon(){}
  };
}
#endif