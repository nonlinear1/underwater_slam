#ifndef EDGE_POINTXY_HPP
#define EDGE_POINTXY_HPP

#include "vertex_point.hpp"
#include <g2o/core/base_binary_edge.h>

class EdgePointXY : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPointXY, VertexPointXY>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgePointXY(){};

  void computeError(){
    const VertexPointXY* v1 = static_cast<const VertexPointXY*>(_vertices[0]);
    const VertexPointXY* v2 = static_cast<const VertexPointXY*>(_vertices[1]);
    Eigen::Vector2d delta = v2->estimate() - v1->estimate() - _measurement;
    _error = delta.transpose() * _information;
  };
  
  virtual bool read(std::istream& is){
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0,0) >> information()(0,1) >> information()(1,1);
    information()(1,0) = information()(0,1);
    return true;
  };

  virtual bool write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
    return os.good();
  }
};

#endif