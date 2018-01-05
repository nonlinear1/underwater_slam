#ifndef TYPE_SLAM_HPP
#define TYPE_SLAM_HPP

#include "vertex_point.hpp"
#include "edge_pointxy.hpp"
#include "GraphWithTree.hpp"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

G2O_REGISTER_TYPE_GROUP(slam);

G2O_REGISTER_TYPE(VERTEX_POINTXY, VertexPointXY);

G2O_REGISTER_TYPE(EDGE_POINTXY, EdgePointXY);

#endif
