/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <tagslam/gtsam_utils.h>
#include <cmath>

namespace tagslam {
  namespace gtsam_utils {
    using std::sqrt;
    boost::shared_ptr<gtsam::noiseModel::Gaussian>
    to_gtsam(const PoseNoise &pn) {
      if (pn.getIsDiagonal()) {//if 对角阵
        const PoseNoise::Vector6d cov = pn.getDiagonal();
        PoseNoise::Vector6d sig;
        sig << sqrt(cov(0)), sqrt(cov(1)), sqrt(cov(2)),//开方
          sqrt(cov(3)), sqrt(cov(4)), sqrt(cov(5));
        return (gtsam::noiseModel::Diagonal::Sigmas(sig));//Return standard deviations (sqrt of diagonal) 
      }
      return (gtsam::noiseModel::Gaussian::Covariance(//if not 对角阵
                pn.getCovarianceMatrix()));
    }
  }
}
