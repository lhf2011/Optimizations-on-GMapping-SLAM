/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */
#include <stdio.h>

/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
激光雷达的相关参数

激光雷达的最大距离和最大使用距离
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)

scan-match计算score时的方差(不是标准差)，单位的cell
score是指整个指数项。
likelihood是指高斯分布的指数项的系数，即e的指数系数
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)

进行scan-match的时候寻找激光的匹配点的时候使用。
因为不是用的似然场模型，所以激光击中的点由于噪声的影响会在真实障碍物的附近。
因此需要在一个激光击中的点的邻域内进行查找，这个参数定义邻域的范围
这个数值表示单位表示cell的单位 也就是说这个值应该是整数
- @b "~/kernelSize" @b [double] search window for the scan matching process

scan-matching的过程中的初始的搜索步长和迭代次数
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.

计算在likelihoodandscore()函数中 计算likelihood使用用的方差(不是标准差)
likelihood是指高斯分布的指数项的系数，即e的指数
score是指整个指数项。
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood

对于一帧激光雷达数据来说 只取每第(n+1)个激光束  这个是相对于scan-match来说的。
如果n等于0 则取每第1帧激光束
如果n等于1 则取每第2帧激光束 也就是说使用的激光束变成原来的1/2
如果n等于2 则取每第3帧激光束 也就是说使用的激光束变成原来的1/3
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)

scan-matching结果接受的最小得分
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
机器人的运动模型的相关的噪声参数
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
进行滤波器更新的最小距离
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

粒子滤波器的相关参数
- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
初始时候地图的维度和分辨率
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#define TSDFMAP 0
#define OCCMAP 1
#define COUNTMAP 0

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}

void SlamGMapping::init()
{
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
    
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
}

void SlamGMapping::startLiveSlam()
{
    entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "sick_scan", 5);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
    scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));
    transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
    double transform_publish_period;
    ros::NodeHandle private_nh_("~");
    entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);

    rosbag::Bag bag;
    bag.open(bag_fname, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    topics.push_back(scan_topic);
    rosbag::View viewall(bag, rosbag::TopicQuery(topics));

    std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
    foreach(rosbag::MessageInstance const m, viewall)
    {
        tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
        if (cur_tf != NULL)
        {
            for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
            {
                geometry_msgs::TransformStamped transformStamped;
                tf::StampedTransform stampedTf;
                transformStamped = cur_tf->transforms[i];
                tf::transformStampedMsgToTF(transformStamped, stampedTf);
                tf_.setTransform(stampedTf);
            }
        }

        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        if (s != NULL)
        {
            if (!(ros::Time(s->header.stamp)).is_zero())
            {
                s_queue.push(std::make_pair(s, ""));
            }
            if (s_queue.size() > 5)
            {
                ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
                s_queue.pop();
            }
        }

        while (!s_queue.empty())
        {
            try
            {
                tf::StampedTransform t;
                tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
                this->laserCallback(s_queue.front().first);
                s_queue.pop();
            }
            catch(tf2::TransformException& e)
            {
                s_queue.front().second = std::string(e.what());
                break;
            }
        }
    }

    bag.close();
}

void SlamGMapping::publishLoop(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok())
    {
        publishTransform();
        r.sleep();
    }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  centered_laser_pose_.stamp_ = t;
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw);
  return true;
}

bool SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
    std::cout<<"enter  initMapper"<<std::endl;
    laser_frame_ = scan.header.frame_id;
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = laser_frame_;
    ident.stamp_ = scan.header.stamp;
    try
    {
        tf_.transformPose(base_frame_, ident, laser_pose);//ident: stamped_in, laser_pose: stamped_out
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",e.what());
        return false;
    }

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);
    try
    {
        tf_.transformPoint(laser_frame_, up, up);
        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch(tf::TransformException& e)
    {
        ROS_WARN("Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    if (fabs(fabs(up.z()) - 1) > 0.001)
    {
        ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",up.z());
        return false;
    }

    gsp_laser_beam_count_ = scan.ranges.size();

    double angle_center = (scan.angle_min + scan.angle_max)/2;

    if (up.z() > 0)
    {
        do_reverse_range_ = scan.angle_min > scan.angle_max;
        centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                                   tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
        ROS_INFO("Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan.angle_min < scan.angle_max;
        centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                                   tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
        ROS_INFO("Laser is mounted upside down.");
    }
    laser_angles_.resize(scan.ranges.size());
    double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
    for(unsigned int i=0; i<scan.ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan.angle_increment);
    }

    ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
    ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

    GMapping::OrientedPoint gmap_pose(0, 0, 0);
    ros::NodeHandle private_nh_("~");
    if(!private_nh_.getParam("maxRange", maxRange_))
        maxRange_ = scan.range_max - 0.01;
    if(!private_nh_.getParam("maxUrange", maxUrange_))
        maxUrange_ = maxRange_;
    gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                           gsp_laser_beam_count_,
                                           fabs(scan.angle_increment),
                                           gmap_pose,
                                           0.0,
                                           maxRange_);
    ROS_ASSERT(gsp_laser_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
    ROS_ASSERT(gsp_odom_);

    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan.header.stamp))
    {
        ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, lskip_);

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    GMapping::sampleGaussian(1,seed_);

    ROS_INFO("Initialization complete");

    return true;
}

bool SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
    if(!getOdomPose(gmap_pose, scan.header.stamp))
        return false;
    if(scan.ranges.size() != gsp_laser_beam_count_)
        return false;

    ros::Time startTime, endTime;
    startTime = scan.header.stamp;
    endTime = startTime + ros::Duration(scan.time_increment * scan.ranges.size());
    std::vector<double> angles,ranges;

    double* ranges_double = new double[scan.ranges.size()];

    if (do_reverse_range_)
    {
        ROS_DEBUG("Inverting scan");
        int num_ranges = scan.ranges.size();        

        for(int i=0; i < num_ranges; i++)
        {
            if(scan.ranges[num_ranges - i - 1] < scan.range_min)
            {
                ranges.push_back((double)scan.range_max);
            }
            else
            {
                ranges.push_back((double)scan.ranges[num_ranges - i - 1]);
            }
            angles.push_back(std::max((double)(scan.angle_min + scan.angle_increment * i),(double)scan.angle_min));
        }
    }
    else
    {
        for(unsigned int i=0; i < scan.ranges.size(); i++)
        {
            // Must filter out short readings, because the mapper won't
            if(scan.ranges[i] < scan.range_min)
            {
                ranges.push_back((double)scan.range_max);
            }
            else
            {
                ranges.push_back((double)scan.ranges[i]);
            }
            angles.push_back(std::max((double)(scan.angle_min + scan.angle_increment * i),(double)scan.angle_min));
        }
    }

    // remove the motion distortion
    Lidar_Calibration(ranges,angles,startTime,endTime);

    for(int i = 0; i < ranges.size();i++)
    {
        ranges_double[i]=ranges[i];
    }

    GMapping::RangeReading reading(scan.ranges.size(),ranges_double,gsp_laser_,scan.header.stamp.toSec());
    delete[] ranges_double;
    reading.setPose(gmap_pose);
    return gsp_->processScan(reading);
}

void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;

  if(addScan(*scan, odom_pose))
  {
      ROS_DEBUG("scan processed");

      GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
      ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
      ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
      ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

      tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
      tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

      map_to_odom_mutex_.lock();
      map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
      map_to_odom_mutex_.unlock();

      if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
      {
          updateMap(*scan);
          last_map_update = scan->header.stamp;
          ROS_DEBUG("Updated the map");
      }
   }
  else
      ROS_DEBUG("cannot process scan");
}

double SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
    ROS_DEBUG("Update map");
    boost::mutex::scoped_lock map_lock (map_mutex_);
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

    std_msgs::Float64 entropy;
    entropy.data = computePoseEntropy();
    if(entropy.data > 0.0)
        entropy_publisher_.publish(entropy);

    if(!got_map_)
    {
        map_.map.info.resolution = delta_;
        map_.map.info.origin.position.x = 0.0;
        map_.map.info.origin.position.y = 0.0;
        map_.map.info.origin.position.z = 0.0;
        map_.map.info.origin.orientation.x = 0.0;
        map_.map.info.origin.orientation.y = 0.0;
        map_.map.info.origin.orientation.z = 0.0;
        map_.map.info.origin.orientation.w = 1.0;
    }

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,delta_);

    ROS_DEBUG("Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node;n;n = n->parent)
    {
        ROS_DEBUG("  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            ROS_DEBUG("Reading is NULL");
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY())
    {
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        map_.map.info.width = smap.getMapSizeX();
        map_.map.info.height = smap.getMapSizeY();
        map_.map.info.origin.position.x = xmin_;
        map_.map.info.origin.position.y = ymin_;
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);

        ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
    }

    // generate the map, it can be shown in RVIZ
    // there are 3 kind of mapping method, they are counting method, TSDF method and occupancy grid map
    // you can choose one by setting the macro
    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            GMapping::IntPoint p(x, y);

#if !COUNTMAP && !TSDFMAP && !OCCMAP
            double occ=smap.cell(p);
            assert(occ < 1.0);
            if(occ < 0)//unknown
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
            else if(occ > occ_thresh_)//occupy
            {
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
            }
            else//free
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
#endif

#if COUNTMAP
            double occ=(double)smap.cell(p).pMapHits/(double)(smap.cell(p).pMapHits+smap.cell(p).pMapMisses);

            if(smap.cell(p).pMapHits== 0 && smap.cell(p).pMapMisses==0)//unknown
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
            else if(occ > 0.5)//occupy
            {
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
            }
            else//free
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
#endif

#if TSDFMAP
            double occ=smap.cell(p).pMapTSDF;
            if(occ > occ_thresh_)//occupy
            {
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
            }
            else//free
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
#endif

#if OCCMAP
            double occ =smap.cell(p).pLog;
            if(occ < 0)//unknown
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
            else if(occ > 1)//occupy
            {
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
            }
            else//free
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
#endif
        }
    }
    got_map_ = true;
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = tf_.resolve( map_frame_ );
    sst_.publish(map_.map);
    sstm_.publish(map_.map.info);
}

bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                               nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

void SlamGMapping::Lidar_Calibration(std::vector<double>& ranges,std::vector<double>& angles,ros::Time startTime,ros::Time endTime)
{
    // get lidar beam number in one scan
    int beamNumber = ranges.size();
    if(beamNumber != angles.size())
    {
        ROS_ERROR("Error:ranges not match to the angles");
        return ;
    }

    // take 5ms as a interpolation interval
    int interpolation_time_duration = 5 * 1000;

    tf::Stamped<tf::Pose> frame_start_pose;
    tf::Stamped<tf::Pose> frame_mid_pose;
    tf::Stamped<tf::Pose> frame_base_pose;
    tf::Stamped<tf::Pose> frame_end_pose;

    //start time, us
    double start_time = startTime.toSec() * 1000 * 1000;
    double end_time = endTime.toSec() * 1000 * 1000;
    double time_inc = (end_time - start_time) / beamNumber;

    // start pose in current segment
    int start_index = 0;

    if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0)))
    {
        ROS_WARN("Not Start Pose,Can not Calib");
        return ;
    }

    if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0)))
    {
        ROS_WARN("Not End Pose, Can not Calib");
        return ;
    }
    int cnt = 0;

    // take the 1st start pose as the base pose
    frame_base_pose = frame_start_pose;
    for(int i = 0; i < beamNumber; i++)
    {
        //iterate all segment, each has a time increment: interpolation_time_duration
        double mid_time = start_time + time_inc * (i - start_index);
        if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
        {
            cnt++;

            // get the start pose and end pose
            if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0)))
            {
                ROS_ERROR("Mid %d Pose Error",cnt);
                return ;
            }

            // how many points in interpolation_time_duration
            int interp_count = i - start_index + 1;

            Lidar_MotionCalibration(frame_base_pose,
                                    frame_start_pose,
                                    frame_mid_pose,
                                    ranges,
                                    angles,
                                    start_index,
                                    interp_count);

            // update time
            start_time = mid_time;
            start_index = i;
            frame_start_pose = frame_mid_pose;
        }
    }
}
bool SlamGMapping::getLaserPose(tf::Stamped<tf::Pose> &odom_pose,ros::Time dt)
{
    odom_pose.setIdentity();

    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_laser";
    robot_pose.stamp_ = dt;   // get the lastest transformation

    // get the global pose of the robot
    try
    {
        if(!tf_.waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))
        {
            ROS_ERROR("LidarMotion-Can not Wait Transform()");
            return false;
        }
        tf_.transformPose("/odom", robot_pose, odom_pose);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
        return false;
    }
    return true;
}

// romove distortion, assume robot is the same velocity during a segment
void SlamGMapping::Lidar_MotionCalibration(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double>& ranges,
        std::vector<double>& angles,
        int startIndex,
        int& beam_number)
{
   // get pose from IMU
    double lastYaw = tf::getYaw(frame_start_pose.getRotation());
    double nextYaw = tf::getYaw(frame_end_pose.getRotation());
    double lastX = frame_start_pose.getOrigin().getX();
    double nextX = frame_end_pose.getOrigin().getX();
    double lastY = frame_start_pose.getOrigin().getY();
    double nextY = frame_end_pose.getOrigin().getY();

    // the 1st pose in world coordinate
    Eigen::MatrixXd Tob=Eigen::MatrixXd(3,3);
    Tob<<cos(lastYaw),-sin(lastYaw),lastX,
         sin(lastYaw), cos(lastYaw),lastY,
            0        ,     0       ,  1;

    // for each beam
    for(int i = startIndex+1; i < startIndex+beam_number;i++)
    {
        double currentYaw=lastYaw + i*(nextYaw-lastYaw)/beam_number;
        double currentX=lastX + i*(nextX-lastX)/beam_number;
        double currentY=lastY + i*(nextY-lastY)/beam_number;

        // the pose of this segment in world coordinate
        Eigen::MatrixXd Toa=Eigen::MatrixXd(3,3);
        Toa<<cos(currentYaw),-sin(currentYaw),currentX,
             sin(currentYaw), cos(currentYaw),currentY,
                0        ,     0       ,  1;

        // transform the pose of this segment to the 1st pose, get the transformation matrix
        Eigen::MatrixXd Tba=Tob.inverse()*Toa;

        // the position before calc
        Eigen::Vector3d pointBeforeCalc;
        pointBeforeCalc(0)= ranges[i] * cos(angles[i]),
        pointBeforeCalc(1)= ranges[i] * sin(angles[i]),
        pointBeforeCalc(2)= 1;

        // the position after calc
        Eigen::Vector3d pointAfterCalc;
        pointAfterCalc=Tba*pointBeforeCalc;

        // get the x, y
        double xAfterCalc = pointAfterCalc(0);
        double yAfterCalc = pointAfterCalc(1);

        // transfer the x,y to polar coordinate ranges,angles
        ranges[i]=sqrt(xAfterCalc*xAfterCalc + yAfterCalc*yAfterCalc);
        angles[i]=atan2(yAfterCalc,xAfterCalc);
    }
}
