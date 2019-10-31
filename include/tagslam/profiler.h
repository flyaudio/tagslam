/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <iostream>
#include <boost/chrono.hpp>
#include <boost/unordered_map.hpp>

namespace tagslam {
  class Profiler {//好像没什么用?? 到底记录什么??
  public:
    typedef boost::chrono::high_resolution_clock::time_point TimePoint;//时刻
    typedef boost::chrono::duration<long long, boost::micro> Duration;//<数值,时间单位>;microseconds
    Profiler() {};
    virtual ~Profiler() {};

    void reset(const char *label) {
      const auto t = boost::chrono::high_resolution_clock::now();
      ProfilerMap::iterator i = map_.find(label);
      if (i == map_.end()) {                 //找不到
        map_[label] = MapEntry(PTimer(), t);//新建
      } else {                              //找到
        i->second.lastTime = t;             //time update
      }
    }
    int record(const char *label, int ncount = 1) {
      ProfilerMap::iterator i = map_.find(label);
      if (i == map_.end()) {   //找不到
        std::cout << "ERROR: invalid timer: " << label << std::endl;
        throw std::runtime_error("invalid timer!");
      }
      auto &me = i->second; //MapEntry 类型
      const TimePoint now =	boost::chrono::high_resolution_clock::now();
      const Duration usec =	boost::chrono::duration_cast<Duration>(now - me.lastTime);
      me.timer = PTimer(usec, me.timer, ncount);
      me.lastTime = now;//time update
      return (usec.count());
    }
	//Profiler类重载运算符<<
    friend std::ostream &operator<<(std::ostream& os, const Profiler &p);
  private:
    struct PTimer {
      PTimer();
      PTimer(const Duration &d, const PTimer&oldTimer, int ncount = 1);
      Duration	duration;		// sum of durations
      int64_t		sqduration;	// sum of squared durations
      Duration	min;				// smallest duration
      Duration	max;				// largest duration
      int64_t		count;			// number of samples
    };
    struct MapEntry {
      MapEntry(const PTimer &p = PTimer(), const TimePoint &t = TimePoint()) :
        timer(p), lastTime(t) {}
      PTimer    timer;
      TimePoint lastTime;
    };
    typedef boost::unordered_map<const char *, MapEntry> ProfilerMap;
    ProfilerMap 	map_;
  };
  std::ostream &operator<<(std::ostream& os, const Profiler &p);//重载操作符<<;不属于类Profiler
}
