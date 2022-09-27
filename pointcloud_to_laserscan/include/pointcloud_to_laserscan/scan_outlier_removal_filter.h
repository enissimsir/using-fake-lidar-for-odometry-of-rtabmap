
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Fraunhofer IPA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Sofie Nilsson
 */

/* 
 * The scan_outlier_removal_filter removes noise clusters with much smaller range than the surrounding points. 
 * The filter can be configured with the following three parameters cluster_break_distance, max_noise_cluster_size, 
 * max_noise_cluster_distance. 
 */

#ifndef IPA_POINTCLOUD_TO_LASERSCAN_SCAN_OUTLIER_REMOVAL_FILTER
#define IPA_POINTCLOUD_TO_LASERSCAN_SCAN_OUTLIER_REMOVAL_FILTER

#include <sensor_msgs/LaserScan.h>

 namespace scan_outlier_filter
 {
   class ScanOutlierRemovalFilter
   {
   private:
    bool filter_configured_;
    double cluster_break_distance_; 
    int max_noise_cluster_size_; 
    double max_noise_cluster_distance_;

  public:
    ScanOutlierRemovalFilter(): 
    filter_configured_(false), 
    cluster_break_distance_(0.0), 
    max_noise_cluster_size_(0), 
    max_noise_cluster_distance_(0.0)
    {};
    void configure(const double cluster_break_distance, const int max_noise_cluster_size, const double max_noise_cluster_distance);

    void remove_outliers(sensor_msgs::LaserScan &scan);
  };
}
#endif //IPA_POINTCLOUD_TO_LASERSCAN_SCAN_OUTLIER_REMOVAL_FILTER
