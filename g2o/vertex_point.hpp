#ifndef VERTEX_POINT_HPP
#define VERTEX_POINT_HPP

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "Node.hpp"

class VertexPointXY : public g2o::BaseVertex<2, Eigen::Vector2d>
{      
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexPointXY(){
    _estimate.setZero();
  }

  virtual void setToOriginImpl() {
    _estimate.setZero();
  }

  virtual void oplusImpl(const double* update)
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
  }

  virtual bool read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1];
    return true;
  }

  virtual bool write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1);
    return os.good();
  }
  
  Eigen::Vector2d operator+(Eigen::Vector2d vec){
    Eigen::Vector2d rst = vec + _estimate;
    return rst;
  }

  double distence_to(VertexPointXY vec){
    Eigen::Vector2d rst = vec.estimate() - _estimate;
    return rst.norm();
  } 

  void set_r(double R) 
  {
    _r = R;
  } 

  double r() const
  {
    return _r;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud() const
  {
    return _cloud;
  }

  void set_cloud(pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    _cloud = cloud;
  }


private:
  double _r;
  pcl::PointCloud<pcl::PointXYZ> _cloud;


};

#endif