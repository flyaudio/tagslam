/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "tagslam/graph_vertex.h"
#include "tagslam/graph_edge.h"


namespace tagslam {
  typedef  boost::adjacency_list<
    boost::listS, boost::vecS, boost::undirectedS,
    GraphVertex, GraphEdge> BoostGraph;//图的定义. 后面两个参数分别是 节点属性 和 边属性
  typedef BoostGraph::vertex_descriptor BoostGraphVertex;//节点描述符
}
