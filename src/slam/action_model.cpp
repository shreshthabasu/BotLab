#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
: alpha1_ (0.003f)
, alpha3_ (0.1)
, initialized_ (false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    moved_ = false;
    delrot2_ = 0;
    delrot1_ = 0;
    deltrans_ = 0;
    rot1Std_ = 0.0001f;
    rot2Std_= 0.0001f;
    transStd_= 0.0001f;
    utime_ = 0;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_) {
        prevOdometry_ = odometry;
        initialized_ = true;
    }
    moved_ = false;
    utime_ = odometry.utime;
    float deltaX = odometry.x - prevOdometry_.x;
    float deltaY = odometry.y - prevOdometry_.y;
    float deltaTheta = angle_diff(odometry.theta, prevOdometry_.theta);
    int16_t dir = 1;

    deltrans_ = sqrt(powl(deltaX,2) + powl(deltaY,2));
    delrot1_ = angle_diff(atan2(deltaY, deltaX),prevOdometry_.theta);

    if (deltrans_ < 0.0001f) {
        delrot1_ = 0;
    } else if (delrot1_ >  M_PI / 2) {
        dir *= -1;
        delrot1_ = -angle_diff(M_PI, delrot1_);
    } else if (delrot1_ < -M_PI / 2) {
        dir *= -1;
        delrot1_ = -angle_diff(-M_PI, delrot1_);
    }

    deltrans_ *= dir;
    delrot2_ = angle_diff(deltaTheta, delrot1_);

    if (fabs(deltrans_) > 0.0001f || fabs(delrot2_) > 0.0001f) {
        moved_ = true;
    } 

    // rot1Std_ = alpha1_ * powl(delrot1_, 2) + alpha2_ * powl(deltrans_, 2);
    // transStd_ = alpha3_ * powl(deltrans_, 2) + alpha4_ * (powl(delrot1_, 2) * powl(delrot2_, 2));
    // rot2Std_ = alpha1_ * powl(delrot2_, 2) + alpha2_ * powl(deltrans_, 2);

    rot1Std_ = alpha1_ * fabs(delrot1_);
    transStd_ = alpha3_ * fabs(deltrans_);
    rot2Std_ = alpha1_ * fabs(delrot2_);

    prevOdometry_ = odometry;
    
    if (moved_) {
        return true;
    }
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    
    if (moved_) {
        particle_t newSample = sample;
        double sampledRot1 = std::normal_distribution<>(delrot1_, rot1Std_) (numberGen_);
        double sampledTrans = std::normal_distribution<>(deltrans_, transStd_) (numberGen_);
        double sampledRot2 = std::normal_distribution<>(delrot2_, rot2Std_) (numberGen_);

        newSample.pose.x += sampledTrans * cos(sample.pose.theta + sampledRot1);
        newSample.pose.y += sampledTrans * sin(sample.pose.theta + sampledRot1);
        newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
        newSample.pose.utime = utime_;
        newSample.parent_pose = sample.pose;        
        return newSample;
    } else {
        particle_t newSample = sample;
        newSample.pose.utime = utime_;
        newSample.parent_pose = sample.pose;
        return newSample;
    }
  
}
