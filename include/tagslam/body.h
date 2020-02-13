/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/pose_with_noise.h"
#include "tagslam/tag.h"
#include <map>
#include <unordered_map>
#include <memory>
#include <iostream>

namespace tagslam {
  class Body {// 对应yml文件中的bodies
    using string = std::string;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //这个宏在new一个对象时会总是返回一个对齐的指针

    typedef std::shared_ptr<Body>            BodyPtr;
    typedef std::shared_ptr<const Body>      BodyConstPtr;
    typedef std::vector<BodyPtr>             BodyVec;
    typedef std::vector<BodyConstPtr>        BodyConstVec;
    typedef std::unordered_map<int, TagPtr>  IdToTagMap;

    // virtual methods to be implemented by derived classes
    virtual bool printTags() const { return (true); }
    virtual bool write(std::ostream &os, const string &prefix) const = 0;
    virtual bool parse(XmlRpc::XmlRpcValue body, const BodyPtr &bp) = 0;
    //

    // getters
    //
    const string &getName() const        { return (name_); }
    const string &getFrameId() const     { return (frameId_); }
    int           getId() const          { return (id_); }
    const string &getOdomTopic() const   { return (odomTopic_); }
    const string &getOdomFrameId() const { return (odomFrameId_); }
    const PoseWithNoise getPoseWithNoise() const  { return (poseWithNoise_); }
    const Transform &getTransformBodyOdom() const { return (T_body_odom_); }
    double getDefaultTagSize() const              { return (defaultTagSize_); }
    const std::list<TagPtr> &getTags() const     { return (tagList_); }
    
    double getOdomTranslationNoise() const {
      return (odomTranslationNoise_); }
    double getOdomRotationNoise() const {
      return (odomRotationNoise_); }
    double getOdomAccelerationNoiseMin() const {
      return (odomAccelerationNoiseMin_); }
    double getOdomAccelerationNoiseMax() const {
      return (odomAccelerationNoiseMax_); }
    double getOdomAngularAccelerationNoiseMin() const {
      return (odomAngularAccelerationNoiseMin_); }
    double getOdomAngularAccelerationNoiseMax() const {
      return (odomAngularAccelerationNoiseMax_); }
    double getFakeOdomTranslationNoise() const {
      return (fakeOdomTranslationNoise_); }
    double getFakeOdomRotationNoise() const {
      return (fakeOdomRotationNoise_); }
    double getOverrideTagRotationNoise() const {
      return (overrideTagRotationNoise_); }
    double getOverrideTagPositionNoise() const {
      return (overrideTagPositionNoise_); }
    bool   isStatic() const { return (isStatic_); }

    // setters

    void   setType(const string &t)                 { type_ = t; }//'simple' or 'board'
    void   setId(int id)                            { id_   = id; }
    void   setPoseWithNoise(const PoseWithNoise &p) { poseWithNoise_ = p; }

    // helper functions

    bool   ignoreTag(int tagId) const {//查找yml中指定的'ignore_tags'
      return (ignoreTags_.count(tagId) != 0); }//在set中查找
    bool    overrides() const {
      return (overrideTagRotationNoise_ > 0 &&
              overrideTagPositionNoise_ > 0);
    }
    TagPtr  findTag(int tagId, int bits) const;
    void    addTag(const TagPtr &tag);
    void    addTags(const TagVec &tags);

    // static functions

    static BodyVec parse_bodies(XmlRpc::XmlRpcValue config);

  protected:
    Body(const string &n  = string(""), bool iS = false) :
      name_(n), frameId_(n), isStatic_(iS) {};
    virtual ~Body() {};
    bool    parseCommon(XmlRpc::XmlRpcValue body);
    bool    writeCommon(std::ostream &os, const string &prefix) const;
    // -------------------------
    string              name_;
    string              frameId_;
    int                 id_{-1};//如果yml中有多个body,用id来区别
    bool                isStatic_{true};
    string              type_;//'simple' or 'board'
    int                 maxHammingDistance_{2};
    TagMap             tags_; // tags that are hanging off of it
    std::set<int>       ignoreTags_; // reject these tags for this body//yml中的'ignore_tags'
    double              defaultTagSize_{0}; // tag size for discovered tags//yml中的'default_tag_size'
    PoseWithNoise       poseWithNoise_; // initial pose prior if valid
    double              overrideTagRotationNoise_{-1};
    double              overrideTagPositionNoise_{-1};
/*
Sometimes it can help to avoid jump. 
The number should be the maximum distance that you think vehicle moves between two consecutive frames. 
For example if the vehicle moves 1 meter/sec and the camera is 20fps, between two frames vehicle moves 0.05 meters.
But very big odom noise makes the optimisation unstable and the result will be inaccurate. But maybe you can try to see the effect based on the vehicle speed. 
*/
    double              fakeOdomTranslationNoise_{-1.0};
    double              fakeOdomRotationNoise_{-1.0};
    // variables used in case odometry data is available
    string              odomTopic_;
    string              odomFrameId_;
    double              odomTranslationNoise_{-1.0};
    double              odomRotationNoise_{-1.0};
    double              odomAccelerationNoiseMin_{5.0}; // m/s^2
    double              odomAngularAccelerationNoiseMin_{5.0}; // rad/sec^2
    double              odomAccelerationNoiseMax_{50.0}; // m/s^2
    double              odomAngularAccelerationNoiseMax_{50.0}; // rad/sec^2
    Transform           T_body_odom_;
    // -------- static functions
    static BodyPtr parse_body(const string &name, XmlRpc::XmlRpcValue config);
  private:
    std::list<TagPtr>   tagList_;//有了tags_,why还要tagList_ ??
  };
  using BodyPtr      = Body::BodyPtr;
  using BodyConstPtr = Body::BodyConstPtr;
  using BodyVec      = Body::BodyVec;
  using BodyConstVec = Body::BodyConstVec;
}
