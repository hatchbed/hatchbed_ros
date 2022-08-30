/**
 * Copyright (c) 2022, Hatchbed
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nodelet/nodelet.h>
#include <param_util/param_handler.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace hatchbed_localization_util {

class ImuToTwist : public nodelet::Nodelet {

    public:
    ImuToTwist() = default;
    ~ImuToTwist() = default;

    virtual void onInit() override {
        init_timer_ = getNodeHandle().createWallTimer(ros::WallDuration(1.0), &ImuToTwist::initialize, this, true);
    }

    private:
    void initialize(const ros::WallTimerEvent& e) {
        auto nh = getNodeHandle();
        auto pnh = getPrivateNodeHandle();

        params_ = std::make_shared<param_util::ParamHandler>(pnh);

        differential_ = params_->param("differential", false, "Whether to calculate twist from the differential of the orientation").value();

        twist_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 10);
        imu_sub_ = nh.subscribe("imu", 10, &ImuToTwist::callback, this);
    }

    void callback(const sensor_msgs::ImuConstPtr& imu) {
        auto twist = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
        twist->header = imu->header;

        if (differential_) {
            // TODO
        }
        else {
            twist->twist.twist.angular = imu->angular_velocity;
            twist->twist.covariance[21] = imu->angular_velocity_covariance[0];
            twist->twist.covariance[28] = imu->angular_velocity_covariance[4];
            twist->twist.covariance[35] = imu->angular_velocity_covariance[8];
        }

        twist_pub_.publish(twist);
    }

    ros::WallTimer init_timer_;
    ros::Publisher twist_pub_;
    ros::Subscriber imu_sub_;

    param_util::ParamHandler::Ptr params_;

    bool differential_ = false;
};

}  // hatchbed_localization_util

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hatchbed_localization_util::ImuToTwist, nodelet::Nodelet)
