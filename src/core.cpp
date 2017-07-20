/*
 * core.cpp
 * More sane version of guidance core node.
 *
 */

// cam_select.srv

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <guidance/cam_select.h>

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;
ros::ServiceServer cam_select_srv;

using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

uint8_t         verbosity = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat				g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat				depth8(HEIGHT, WIDTH, CV_8UC1);


int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {        
        image_data* data = (image_data*)content;

		if ( data->m_greyscale_image_left[CAMERA_ID] ){
			memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);

			// publish left greyscale image
			cv_bridge::CvImage left_8;
			g_greyscale_image_left.copyTo(left_8.image);
			left_8.header.frame_id  = "guidance";
			left_8.header.stamp	= ros::Time::now();
			left_8.encoding		= sensor_msgs::image_encodings::MONO8;
			left_image_pub.publish(left_8.toImageMsg());
		}
		if ( data->m_greyscale_image_right[CAMERA_ID] ){
			memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);

			// publish right greyscale image
			cv_bridge::CvImage right_8;
			g_greyscale_image_right.copyTo(right_8.image);
			right_8.header.frame_id  = "guidance";
			right_8.header.stamp	 = ros::Time::now();
			right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;
			right_image_pub.publish(right_8.toImageMsg());
		}
		if ( data->m_depth_image[CAMERA_ID] ){
			memcpy(g_depth.data, data->m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);
			g_depth.convertTo(depth8, CV_8UC1);

			//publish depth image
			cv_bridge::CvImage depth_16;
			g_depth.copyTo(depth_16.image);
			depth_16.header.frame_id  = "guidance";
			depth_16.header.stamp	  = ros::Time::now();
			depth_16.encoding	  = sensor_msgs::image_encodings::MONO16;
			depth_image_pub.publish(depth_16.toImageMsg());
		}
    }

    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
        
    	// publish imu data
		geometry_msgs::TransformStamped g_imu;
		g_imu.header.frame_id = "guidance";
		g_imu.header.stamp    = ros::Time::now();
		g_imu.transform.translation.x = imu_data->acc_x;
		g_imu.transform.translation.y = imu_data->acc_y;
		g_imu.transform.translation.z = imu_data->acc_z;
		g_imu.transform.rotation.w = imu_data->q[0];
		g_imu.transform.rotation.x = imu_data->q[1];
		g_imu.transform.rotation.y = imu_data->q[2];
		g_imu.transform.rotation.z = imu_data->q[3];
		imu_pub.publish(g_imu);
    }

    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
	
		// publish velocity
		geometry_msgs::Vector3Stamped g_vo;
		g_vo.header.frame_id = "guidance";
		g_vo.header.stamp    = ros::Time::now();
		g_vo.vector.x = 0.001f * vo->vx;
		g_vo.vector.y = 0.001f * vo->vy;
		g_vo.vector.z = 0.001f * vo->vz;
		velocity_pub.publish(g_vo);
    }

    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;

		// publish obstacle distance
		sensor_msgs::LaserScan g_oa;
		g_oa.ranges.resize(CAMERA_PAIR_NUM);
		g_oa.header.frame_id = "guidance";
		g_oa.header.stamp    = ros::Time::now();
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
			g_oa.ranges[i] = 0.01f * oa->distance[i];
		obstacle_distance_pub.publish(g_oa);
	}

    /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
	
		// publish ultrasonic data
		sensor_msgs::LaserScan g_ul;
		g_ul.ranges.resize(CAMERA_PAIR_NUM);
		g_ul.intensities.resize(CAMERA_PAIR_NUM);
		g_ul.header.frame_id = "guidance";
		g_ul.header.stamp    = ros::Time::now();
		for ( int d = 0; d < CAMERA_PAIR_NUM; ++d ){
			g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
			g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
		}
		ultrasonic_pub.publish(g_ul);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

bool cam_select_callback(guidance::cam_select::Request &request, guidance::cam_select::Response &response)
{
    ROS_DEBUG("I got called with val %d...", request.camera_index);

    if(request.camera_index > 4)
    {
        ROS_WARN("Requested camera index %d out of range.", request.camera_index);
        response.ok = false;
        return true;
    }

    CAMERA_ID = (e_vbus_index)request.camera_index;

    int err_code = stop_transfer();
    if(err_code)
    {
        ROS_FATAL("guidance stop xfer failed!");
        exit(1);
    }
    reset_config();

    // Err detect?
    select_greyscale_image(CAMERA_ID, true);
    select_greyscale_image(CAMERA_ID, false);
    select_depth_image(CAMERA_ID);

    select_imu();
    select_ultrasonic();
    select_obstacle_distance();
    select_velocity();

    err_code = start_transfer();
    if(err_code)
    {
        ROS_FATAL("guidance start xfer failed!");
        exit(1);
    }

    response.ok = true;
    return true;
}

int main(int argc, char** argv)
{
    /* initialize ros */
    ros::init(argc, argv, "GuidanceCore");
    ros::NodeHandle my_node;
    depth_image_pub			= my_node.advertise<sensor_msgs::Image>("guidance/depth_image",1);
    left_image_pub			= my_node.advertise<sensor_msgs::Image>("guidance/left_image",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("guidance/right_image",1);
    imu_pub  				= my_node.advertise<geometry_msgs::TransformStamped>("guidance/imu",1);
    velocity_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("guidance/velocity",1);
    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("guidance/obstacle_distance",1);
    ultrasonic_pub			= my_node.advertise<sensor_msgs::LaserScan>("guidance/ultrasonic",1);
    cam_select_srv          = my_node.advertiseService("guidance/cam_select", cam_select_callback);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    if(err_code)
    {
        ROS_FATAL("guidance init_transfer failed: %i", err_code);
        exit(1);
    }

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
    if(err_code)
    {
        ROS_FATAL("guidance get_online_status failed: %i", err_code);
        exit(1);
    }

	for(int i=0; i<CAMERA_PAIR_NUM; i++)
    {
        if(!online_status[i]) ROS_WARN("Sensor %d offline!", i);
    }

	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
    if(err_code)
    {
        ROS_FATAL("guidance calibration failed!");
        exit(1);
    }
#if 0
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
	{
        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
	}
#endif
	
    /* select data */
    err_code = select_greyscale_image(CAMERA_ID, true);
    if(err_code)
    {
        ROS_FATAL("guidance select_greyscale_image failed: %i", err_code);
        exit(1);
    }
    err_code = select_greyscale_image(CAMERA_ID, false);
    if(err_code)
    {
        ROS_FATAL("guidance select_greyscale_image failed: %i", err_code);
        exit(1);
    }
    err_code = select_depth_image(CAMERA_ID);
    if(err_code)
    {
        ROS_FATAL("guidance select_greyscale_image failed: %i", err_code);
        exit(1);
    }
    select_imu();
    select_ultrasonic();
    select_obstacle_distance();
    select_velocity();

    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    if(err_code)
    {
        ROS_FATAL("guidance set callback failed!");
        exit(1);
    }

    err_code = start_transfer();
    if(err_code)
    {
        ROS_FATAL("guidance start xfer failed!");
        exit(1);
    }

    ROS_INFO("guidance running...");
	
#if 0
	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
    para.m_camera_pair_index = CAMERA_ID;
#endif
	

	while (ros::ok())
	{
		g_event.wait_event();

        ros::spinOnce();
	}

	/* release data transfer */
	err_code = stop_transfer();
	//make sure the ack packet from GUIDANCE is received
	sleep(1);
    ROS_INFO("guidance releasing...");
	release_transfer();
    ROS_INFO("guidance exit");
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
