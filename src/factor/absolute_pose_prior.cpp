/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "tagslam/factor/absolute_pose_prior.h"
#include "tagslam/value/value.h"
#include "tagslam/optimizer.h"
#include "tagslam/graph.h"
#include <sstream>

namespace tagslam {
  namespace factor {
/* 
func
	1.通过idToVertex_,name+time查询 vertex
	2.把Factory_vertex插入boost::graph
  */
    VertexDesc
    AbsolutePosePrior::addToGraph(const VertexPtr &vp, Graph *g) const {
      // NOTE: prior name and pose name must match!
      const VertexDesc cp = g->findPose(getTime(), vp->getName());
      checkIfValid(cp, "no pose for absolute pose prior");
      const VertexDesc fv = g->insertFactor(vp);
      g->addEdge(fv, cp, 0);//Factor_vertex,Value_vertex,edge property
      return (fv);
    }

/*
func
	1.find the Factor_vertex descriptor
	2.通过optimized_ 查询是否已经添加到GTSAM(如果添加了就会有对应的Value_key/Factor_key)
	3.getOptKeysForFactor();通过此Factor_vertex找到相连的Value_vertex
	4.addAbsolutePosePrior();在GTSAM里添加先验prior
	5.此Factor_vertex在optimized_ 里标记(表示已经添加到GTSAM)
*/
    void AbsolutePosePrior::addToOptimizer(Graph *g) const {
      const VertexDesc v = g->find(this);
      checkIfValid(v, "factor not found");
      g->verifyUnoptimized(v);
      const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 1);//得到待优化变量的key
      const FactorKey fk = g->getOptimizer()->addAbsolutePosePrior(
        optKeys[0], getPoseWithNoise());//为此 待优化变量 添加先验约束,=固定此 优化变量
      g->markAsOptimized(v, fk);
    }

/* print name_ & time_*/
    string AbsolutePosePrior::getLabel() const {
      std::stringstream ss;
      ss << "app:" << name_ << ",t:" << format_time(time_);
      return (ss.str());
    }
  } // namespace factor
}  // namespace tagslam
