/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/gtsam_optimizer.h"
#include "tagslam/graph.h"
#include "tagslam/camera2.h"
#include "tagslam/odometry_processor.h"

#include <flex_sync/sync.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>


#include <string>
#include <unordered_map>


namespace tagslam {
  class TagSlam2 {
    using TagArray = apriltag_msgs::ApriltagArrayStamped;
    using TagArrayPtr = TagArray::Ptr;
    using TagArrayConstPtr = TagArray::ConstPtr;
    using Odometry = nav_msgs::Odometry;
    using OdometryConstPtr = nav_msgs::OdometryConstPtr;
    using Image = sensor_msgs::Image;
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using CompressedImage = sensor_msgs::CompressedImage;
    using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
    using string = std::string;
    using VertexPose = Graph::VertexPose;
  public:
    TagSlam2(const ros::NodeHandle &nh);
    TagSlam2(const TagSlam2&) = delete;
    TagSlam2& operator=(const TagSlam2&) = delete;

    bool initialize();

    template<typename S>
    void processBag(S *sync, rosbag::View *view) {
      for (const rosbag::MessageInstance &m: *view) {
        typename S::Type1::ConstPtr t1 = m.instantiate<typename S::Type1>();
        if (t1) {    sync->process(m.getTopic(), t1); } else {
          typename S::Type2::ConstPtr t2 = m.instantiate<typename S::Type2>();
          if (t2) {  sync->process(m.getTopic(), t2); } else {
            typename S::Type3::ConstPtr t3 =m.instantiate<typename S::Type3>();
            if (t3) { sync->process(m.getTopic(), t3); }
          }
        }
        if (frameNum_ > maxFrameNum_ || !ros::ok()) {
          break;
        }
      }
    }
    void syncCallback(const std::vector<TagArrayConstPtr> &msgvec1,
                      const std::vector<ImageConstPtr> &msgvec2,
                      const std::vector<OdometryConstPtr> &msgvec3);
    void syncCallbackCompressed(
      const std::vector<TagArrayConstPtr> &msgvec1,
      const std::vector<CompressedImageConstPtr> &msgvec2,
      const std::vector<OdometryConstPtr> &msgvec3);

  private:
    typedef std::unordered_map<int, Tag2ConstPtr> TagMap;

    void makeGraph();
    void readBodies();
    void playFromBag(const std::string &fname);
    void processOdom(const std::vector<OdometryConstPtr> &odomMsg,
                     std::vector<BoostGraphVertex> *factors);
    std::vector<std::vector<string>> makeTopics(rosbag::Bag *bag) const;
    void setupOdom(const std::vector<OdometryConstPtr> &odomMsgs);
    void processTagsAndOdom(const std::vector<TagArrayConstPtr> &tagmsgs,
                            const std::vector<OdometryConstPtr> &odommsgs);
    void publishTransforms(const ros::Time &t);
    void publishBodyOdom(const ros::Time &t);
    void sleep(double dt) const;
    void processTags(const std::vector<TagArrayConstPtr> &tagMsgs,
                     std::vector<BoostGraphVertex> *factors);
    Tag2ConstPtr findTag(int tagId);

    // ------ variables --------
    ros::NodeHandle      nh_;
    Graph                graph_;
    GTSAMOptimizer       optimizer_;
    Camera2Vec           cameras_;
    BodyVec              bodies_;
    BodyVec              nonstaticBodies_;
    BodyPtr              defaultBody_;
    ros::Publisher       clockPub_;
    std::vector<ros::Publisher> odomPub_;
    string               fixedFrame_;
    bool                 writeDebugImages_;
    bool                 hasCompressedImages_;
    int                  frameNum_{0};
    int                  maxFrameNum_{1000000};
    std::vector<cv::Mat> images_;
    std::vector<OdometryProcessor> odomProcessors_;
    tf::TransformBroadcaster tfBroadcaster_;
    TagMap               tagMap_;
  };
}
