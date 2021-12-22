#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <memory>
// #include "ros/ros.h"

#include "rclcpp/rclcpp.hpp"

#define DEBUG_TRAJ 2
using namespace std;

// Father Class

#define AUTOYAW
class TrajectoryGenerator{

protected:    

    std::atomic<float> end_time_ = 0.0f;

public:

    TrajectoryGenerator(){};
    ~TrajectoryGenerator(){};

    virtual bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float medium_speed) = 0;
    virtual bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float medium_speed, const std::vector<float>&){
        std::cout<<"VIRTUAL METHOD CALLED WITHOUT MATCHING FUNCTIONS"<<std::endl;
        return false;
    };
    
    virtual bool evaluateTrajectory(float t ,  std::array<std::array<float,3>,4>& refs, bool for_plot=false) = 0;
    float getEndTime() const {return end_time_;}
    virtual rclcpp::Time getBeginTime(){
        std::cerr << "calling non implemented method"<< std::endl;
        return rclcpp::Clock().now();
    }
    virtual void plot(){
        std::cerr << "calling non implemented method"<< std::endl;
    };

    std::atomic<bool> trajectory_generated_ = false;
    virtual bool getTrajectoryGenerated() {
        return trajectory_generated_;
    }

};

class CircleGenerator:public TrajectoryGenerator{
private :
    std::vector<float> center_;
    float radius_,omega_,height_,yaw_;
    rclcpp::Time beginTime_;
public:
    CircleGenerator(){};
    ~CircleGenerator(){};
    rclcpp::Time getBeginTime(){
        return beginTime_;
    }
    bool generateTrajectory(const vector<vector<float>>& waypoints, float speed);
    bool evaluateTrajectory(float t , std::array<std::array<float,3>,4>& refs_,bool for_plot=false);
};


class LemniscateGenerator:public TrajectoryGenerator{
private :
    std::vector<float> center_;
    float radius_,omega_,height_,yaw_;
    
public:
    LemniscateGenerator(){};
    ~LemniscateGenerator(){};
    bool generateTrajectory(const vector<vector<float>>& waypoints, float speed);
    bool evaluateTrajectory(float time ,  std::array<std::array<float,3>,4>& refs_, bool for_plot=false) ;

};


#endif