/**
 * @file /include/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_drok5_QNODE_HPP_
#define gui_drok5_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
// https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>

#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_drok5 {

    /*****************************************************************************
    ** Class
    *****************************************************************************/

    class QNode : public QThread {
        Q_OBJECT
    public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        bool init();
        bool init(const std::string &master_url, const std::string &host_url);
        void run();

        /*********************
        ** Callback function
        **********************/
        void NUC1_getReady_Callback(const std_msgs::UInt16& ready);
        void NUC2_getReady_Callback(const std_msgs::UInt16& ready);

        void Joystick_state_Callback(const std_msgs::UInt16& state_msg);
        void MD_driver_state_Callback(const std_msgs::UInt16& state_msg);
        void Arm_Ctrl_state_Callback(const std_msgs::UInt16& state_msg);
        void Cam_CtrlDXL_state_Callback(const std_msgs::UInt16& state_msg);
        void Front_RawImage_Callback(const sensor_msgs::CompressedImage::ConstPtr& front_img);
        void Front_YoloImage_Callback(const sensor_msgs::CompressedImage::ConstPtr& yolo_img);
        void SideImage_Callback(const sensor_msgs::CompressedImage::ConstPtr& side_img);
        void Web_vd_server_state_Callback(const std_msgs::UInt16& state_msg);
        //void Velodyne_Lidar_Callback(const sensor_msgs::PointCloud2 velodyne_ptcloud);
        void Velodyne_Lidar_state_Callback(const std_msgs::UInt16& state_msg);
        void Detect_KorOrNor_Callback(const std_msgs::String::ConstPtr& detect_msg);
        void Autodriving_state_Callback(const std_msgs::UInt16& state_msg);

        // Sc
        void Screenshot_Callback(const sensor_msgs::Image& msg);

        // Battery
        void BatteryFloat_Callback(const std_msgs::Float32& battery_msg);

        void blackout(int a);

        /*********************
        ** Logging
        **********************/
        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
        };

        QStringListModel* loggingModel() { return &logging_model; }
        void log(const LogLevel &level, const std::string &msg);

        Q_SIGNALS:
        void loggingUpdated();
        void rosShutdown();

        void statusUpdated();
        void statusUpdated_Sc();
        //void velodyneUpdated();
        void batteryUpdated();

    private:
        int init_argc;
        char** init_argv;
        //ros::Publisher chatter_publisher;
        QStringListModel logging_model;

        /*****************************************************************************
        ** ROS communication node declaration
        *****************************************************************************/
        // Publisher
        ros::Publisher Exec_pkg_publisher;
        ros::Publisher Command_publisher;

        // Subscriber
        ros::Subscriber NUC1_getReady_subscriber;
        ros::Subscriber NUC2_getReady_subscriber;

        ros::Subscriber Joystick_state_subscriber;
        ros::Subscriber MD_driver_state_subscriber;
        ros::Subscriber Arm_Ctrl_state_subscriber;
        ros::Subscriber Cam_CtrlDXL_state_subscriber;
        ros::Subscriber Front_RawImage_subscriber;
        ros::Subscriber Front_YoloImage_subscriber;
        ros::Subscriber SideImage_subscriber;
        ros::Subscriber Web_vd_server_state_subscriber;
        //ros::Subscriber Velodyne_Lidar_subscriber;
        ros::Subscriber Velodyne_Lidar_state_subscriber;
        ros::Subscriber Detect_KorOrNor_subscriber;
        ros::Subscriber Autodriving_state_subscriber;

        // Sc
        ros::Subscriber Screenshot_subscriber;

        // Battery
        ros::Subscriber BatteryFloat_subscriber;
    };

}  // namespace gui_drok5

#endif /* gui_drok5_QNODE_HPP_ */
