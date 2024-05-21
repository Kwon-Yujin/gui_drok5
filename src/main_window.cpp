/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

//#include <QColor>
//#include <QSlider>
//#include <QLabel>
//#include <QGridLayout>
//#include <QVBoxLayout>

#include <QtWidgets>
#include <QMessageBox>
#include <iostream>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "../include/gui_drok5/main_window.hpp"

namespace gui_drok5 {

    int ros_topic_data;
    bool ros_status_flag = 0;
    bool ros_cmd_flag = 0;

    QString q_command_string;

    extern int State[12];
    extern int Arm_State[5];
    extern int Ready[2];
    extern QImage qt_front_image;
    extern QImage qt_side_image;
    extern int nBattery;

    using namespace Qt;

    /*****************************************************************************
    ** Implementation [MainWindow]
    *****************************************************************************/

    MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv) {
        //std::cout << "MainWindow 생성자 함수 시작" << std::endl;
        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

        dialog = new Sc_Dialog;

        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

        ReadSettings();
        setWindowIcon(QIcon(":/images/icon.png"));
        ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
        QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

        /*********************
        ** Logging
        **********************/
        //std::cout << "MainWindow 생성자 함수: Logging 시작" << std::endl;
        ui.view_logging->setModel(qnode.loggingModel());
        QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

        /*********************
        ** Auto Start
        **********************/
        //std::cout << "MainWindow 생성자 함수: remember settings 체크박스 체크되어 있을 시 자동 시작" << std::endl;
        if ( ui.checkbox_remember_settings->isChecked() ) {
            on_button_connect_clicked(true);
        }

        /*******************************
        ** QNode Update event
        ********************************/
        //std::cout << "MainWindow 생성자 함수: qnode 연결" << std::endl;
        QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(updateState()));
        QObject::connect(&qnode, SIGNAL(statusUpdated_Sc()), this, SLOT(updateState_Sc()));
        QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(getReady()));
        //QObject::connect(&qnode, SIGNAL(velodyneUpdated()), this, SLOT(updateVelodyne()));

        /************************************
        ** Execute pkg event - explicit way
        *************************************/
        //QObject::connect(ui.button_left, SIGNAL(clicked()), this, SLOT(moveLeft()));
        // 'getmission' pkg
        //QObject::connect(ui.Button_getmission, SIGNAL(clicked()), this, SLOT(launch_getmission()));
        //std::cout << "MainWindow 생성자 함수: 버튼에 대응되는 시그널-슬롯 연결" << std::endl;

        // Communication
        // 3. Web server available - 양방향 통신 성공 여부 확인
        QObject::connect(ui.Button_Html, SIGNAL(clicked()), this, SLOT(Html()));
        //QObject::connect(ui.Button_Html_Chrome, SIGNAL(clicked()), this, SLOT(Html_Chrome()));
        // 2. Execute rosbridge_websocket.launch
        QObject::connect(ui.Button_Connect_PC_websocket, SIGNAL(clicked()), this, SLOT(Connect_PC_websocket()));
        // 1. Execute .html websocket server file editor
        QObject::connect(ui.Button_Edit_html, SIGNAL(clicked()), this, SLOT(Edit_html()));

        // Start and Stop
        //QObject::connect(ui.Button_Start, SIGNAL(clicked()), this, SLOT(Start()));
        //QObject::connect(ui.Button_All_stop, SIGNAL(clicked()), this, SLOT(All_stop()));

        // #1 Joystick pkg
        QObject::connect(ui.Button_Joystick, SIGNAL(clicked()), this, SLOT(Joystick()));
        QObject::connect(ui.Button_Joystick_OFF, SIGNAL(clicked()), this, SLOT(Joystick_OFF()));

        // #2 MD driver pkg
        QObject::connect(ui.Button_MD_driver, SIGNAL(clicked()), this, SLOT(MD_driver()));
        QObject::connect(ui.Button_MD_driver_OFF, SIGNAL(clicked()), this, SLOT(MD_driver_OFF()));

        // #3 Manipulator pkg
        QObject::connect(ui.Button_Arm_Ctrl, SIGNAL(clicked()), this, SLOT(Arm_Ctrl()));
        QObject::connect(ui.Button_Arm_Ctrl_OFF, SIGNAL(clicked()), this, SLOT(Arm_Ctrl_OFF()));

        // #4 Dynamixel that control front camera pose
        QObject::connect(ui.Button_Cam_CtrlDXL, SIGNAL(clicked()), this, SLOT(Cam_CtrlDXL()));
        QObject::connect(ui.Button_Cam_CtrlDXL_OFF, SIGNAL(clicked()), this, SLOT(Cam_CtrlDXL_OFF()));

        // #5 Front camera drive
        QObject::connect(ui.Button_Front_RawCam, SIGNAL(clicked()), this, SLOT(Front_RawCam()));
        QObject::connect(ui.Button_Front_RawCam_OFF, SIGNAL(clicked()), this, SLOT(Front_RawCam_OFF()));

        // #6 Front Yolo camera drive
        QObject::connect(ui.Button_Front_YoloCam, SIGNAL(clicked()), this, SLOT(Front_YoloCam()));
        QObject::connect(ui.Button_Front_YoloCam_OFF, SIGNAL(clicked()), this, SLOT(Front_YoloCam_OFF()));

        // #7 Side camera drive
        QObject::connect(ui.Button_SideCam_drive, SIGNAL(clicked()), this, SLOT(SideCam_drive()));
        QObject::connect(ui.Button_SideCam_drive_OFF, SIGNAL(clicked()), this, SLOT(SideCam_drive_OFF()));

        // #8 Web_video_server: NUC에서 web_video_server pkg 시작
        QObject::connect(ui.Button_web_vd_server, SIGNAL(clicked()), this, SLOT(Web_vd_server()));
        QObject::connect(ui.Button_web_vd_server_OFF, SIGNAL(clicked()), this, SLOT(Web_vd_server_OFF()));

        /*********************
        ** Embed RVIZ
        **********************/
        // #9 Run Velodyne
        // Create a render panel in ui.QVBoxLayout_Main
        render_panel_ = new rviz::RenderPanel;  // rviz 화면 게시할 디스플레이 세팅
        ui.QVBoxLayout_Main->addWidget(render_panel_);
        QObject::connect(ui.Button_Velodyne_Lidar, SIGNAL(clicked()), this, SLOT(Velodyne_Lidar()));
        QObject::connect(ui.Button_Velodyne_Lidar_OFF, SIGNAL(clicked()), this, SLOT(Velodyne_Lidar_OFF()));

        // #11 Autodriving pkg
        QObject::connect(ui.Button_Autodrive, SIGNAL(clicked()), this, SLOT(AutoDrive()));
        QObject::connect(ui.Button_Autodrive_OFF, SIGNAL(clicked()), this, SLOT(AutoDrive_OFF()));

        /*********************
        ** Label
        **********************/
        m_lightimg[0].load(":/images/led-off.png");     // 0: OFF
        m_lightimg[1].load(":/images/led-on.png");      // 1: ON
        m_lightimg[2].load(":/images/led-green-on.png");

        m_readyimg[0].load(":/images/switch2.jpg");     // 0: OFF
        m_readyimg[1].load(":/images/switch1.jpg");     // 1: ON (Green-colored switch)

        // Banner image below the window-KUDOS-
        //KUDOS_img.load(":/images/KUDOS2.png");
        cat1_img.load(":/images/cat.jpg");
        //cat2_img.load(":/images/cat2.jpg");

        // 각 사진을 UI 레이블에 표시
        //ui.label->setPixmap(cat1_img);
        //ui.Label_FrontImage->setPixmap(cat1_img);
        //ui.label_2->setPixmap(KUDOS_img);

        //QPixmap cat1_img(":/images/cat.jpg");
        ui.Label_FrontImage->setPixmap(cat1_img);
        ui.Label_SideImage->setPixmap(cat1_img);

        /*********************
        ** Progress bar
        **********************/
        QObject::connect(ui.BatteryBar, SIGNAL(valueChanged(nBattery)), this, SLOT(setValue(nBattery)));
    }

    MainWindow::~MainWindow() { delete manager_; }

    /*****************************************************************************
    ** Implementation [Slots]
    *****************************************************************************/

    void MainWindow::showNoMasterMessage() {
        QMessageBox msgBox;
        msgBox.setText("Couldn't find the ros master.");
        msgBox.exec();
        close();
    }

    /*
     * These triggers whenever the button is clicked, regardless of whether it
     * is already checked or not.
     */

    void MainWindow::on_button_connect_clicked(bool check) {
            if ( ui.checkbox_use_environment->isChecked() ) {
                if ( !qnode.init() ) {
                    showNoMasterMessage();
                } else {
                    ui.button_connect->setEnabled(false);
                }
            } else {
                if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                    ui.line_edit_host->text().toStdString()) ) {
                    showNoMasterMessage();
            } else {
                ui.button_connect->setEnabled(false);
                ui.line_edit_master->setReadOnly(true);
                ui.line_edit_host->setReadOnly(true);
                ui.line_edit_topic->setReadOnly(true);
            }
        }
    }

    /*
    void MainWindow::on_button_test_clicked(bool check ) {
        showButtonTestMessage();
    }
    */

    void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
        bool enabled;
        if ( state == 0 ) {
            enabled = true;
        } else {
            enabled = false;
        }
        ui.line_edit_master->setEnabled(enabled);
        ui.line_edit_host->setEnabled(enabled);
        //ui.line_edit_topic->setEnabled(enabled);
    }

    /*****************************************************************************
    ** Implemenation [Slots][manually connected]
    *****************************************************************************/
    /**
     * This function is signalled by the underlying model. When the model changes,
     * this will drop the cursor down to the last line in the QListview to ensure
     * the user can always see the latest log message.
     */
    void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
    }

    // 모든 패키지의 status 항목을 취합해 스위치 상태를 업데이트하는 함수가 updateState() 함수이다.
    // get_guicmd에서 GUI 상 발행된 ros_topic_data 메시지를 받아 응답한 경우, LED-on 상태로 돌아감.
    void MainWindow::updateState()
    {
        // Joystick pkg enabled or disabled status
        if (State[1] == 1)
            ui.Label_State_1->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_1->setPixmap(m_lightimg[0]);

        // MD_driver pkg status
        if (State[2] == 1)
            ui.Label_State_2->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_2->setPixmap(m_lightimg[0]);

        // Manipulator control pkg status
        if (State[3] == 1)
            ui.Label_State_3->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_3->setPixmap(m_lightimg[0]);

        // Cam dynamixel control pkg status
        if (State[4] == 1)
            ui.Label_State_4->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_4->setPixmap(m_lightimg[0]);

        // #5 Front webcam drive pkg status
        if (State[5] == 1) {
            ui.Label_State_5->setPixmap(m_lightimg[1]);
            // Sending front camera's video out
            ui.Label_FrontImage->setPixmap(QPixmap::fromImage(qt_front_image));
            ui.Label_FrontImage->resize(ui.Label_FrontImage->pixmap()->size());
        }
        else {
            ui.Label_State_5->setPixmap(m_lightimg[0]);
        }

        // #6 Front Yolo cam drive pkg status
        if (State[6] == 1) {
            ui.Label_State_6->setPixmap(m_lightimg[1]);
            if (State[10] == 1)
                ui.Label_State_9->setPixmap(m_lightimg[2]);
            // Sending front camera's video-ver Yolo- out
            ui.Label_FrontImage->setPixmap(QPixmap::fromImage(qt_front_image));
            ui.Label_FrontImage->resize(ui.Label_FrontImage->pixmap()->size());
        }
        else {
            ui.Label_State_6->setPixmap(m_lightimg[0]);
        }

        // #7 Side RealSense camera drive pkg status
        if (State[7] == 1) {
            ui.Label_State_7->setPixmap(m_lightimg[1]);
            // Sending side camera's video-ver Yolo- out
            ui.Label_SideImage->setPixmap(QPixmap::fromImage(qt_side_image));
            ui.Label_SideImage->resize(ui.Label_FrontImage->pixmap()->size());
        }
        else {
            ui.Label_State_7->setPixmap(m_lightimg[0]);
        }

        // web_video_server pkg status
        if (State[8] == 1)
            ui.Label_State_8->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_8->setPixmap(m_lightimg[0]);

        // Velodyne lidar pkgs status
        //if (State[7] == 1)
        //    ui.Label_State_7->setPixmap(m_lightimg[1]);
        //else
        //    ui.Label_State_7->setPixmap(m_lightimg[0]);

        // Velodyne Lidar pkg status
        if (State[9] == 1)
            ui.Label_State_9->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_9->setPixmap(m_lightimg[0]);

        // Detected NOR=true, else=false
        //if (State[10] == 1)
        //    ui.Label_State_10->setPixmap(m_lightimg[1]);
        //else
        //    ui.Label_State_10->setPixmap(m_lightimg[1]);

        if (State[11] == 1)
            ui.Label_State_11->setPixmap(m_lightimg[1]);
        else
            ui.Label_State_11->setPixmap(m_lightimg[0]);
    }

    void MainWindow::updateState_Sc() {
        dialog->setWindowTitle("NUC Screen");
        dialog->show(); // add
        dialog->show_screenshot(); // add
    }

    // GUI signal(graphical/visible) update function
    void MainWindow::getReady() {
        if (Ready[1] == 1)
            ui.Label_Get_ready->setPixmap(m_readyimg[1]);
        else
            ui.Label_Get_ready->setPixmap(m_readyimg[0]);
    }

    /*
    void MainWindow::updateBattery() {
        ui.BatteryBar->setValue(nBattery);
    }
    */

    /*****************************************************************************
    ** Implemenation for topic publishment [Slots][manually connected]
    *****************************************************************************/
    // Communication
    void MainWindow::Html()
    {
        //ROS_INFO("Html");
        //std::string command_html = "gnome-terminal -- firefox ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html --new-window --width=1080 --height=720";
        //std::string command_html = "gnome-terminal -- google-chrome --new-window --width=1080 --height=720 ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html";
        std::string command_html = "gnome-terminal -- google-chrome --new-window --width=1080 --height=720 ~/catkin_ws/src/roslibjs/examples/HW_test_server.html";
        const char *c_html = command_html.c_str();
        system(c_html);
    }

    void MainWindow::Connect_PC_websocket()
    {
        std::string command_web = "gnome-terminal -- roslaunch rosbridge_server rosbridge_websocket.launch";
        const char *c_web = command_web.c_str();
        system(c_web);
    }

    void MainWindow::Edit_html()
    {
        //std::string command_edit = "gedit ~/catkin_ws/src/roslibjs/examples/HW_test_server_0927_1.html";
        std::string command_edit = "gedit ~/catkin_ws/src/roslibjs/examples/HW_test_server.html";
        const char *c_edit = command_edit.c_str();
        system(c_edit);
    }

    // Start and Stop
    void MainWindow::Start()
    {
        ros_topic_data = 0;
        ros_status_flag = true;
    }
    void MainWindow::All_stop() {
        ros_topic_data = 100;
        ros_status_flag = true;
    }

    // Joystick
    void MainWindow::Joystick()
    {
        ros_topic_data = 1;
        ros_status_flag = true;
    }
    void MainWindow::Joystick_OFF()
    {
        ros_topic_data = 101;
        ros_status_flag = true;
    }

    // MD driver
    void MainWindow::MD_driver()
    {
        ros_topic_data = 2;
        ros_status_flag = true;
    }
    void MainWindow::MD_driver_OFF()
    {
        ros_topic_data = 102;
        ros_status_flag = true;
    }

    // Manipulator control
    void MainWindow::Arm_Ctrl()
    {
        ros_topic_data = 3;
        ros_status_flag = true;
    }
    void MainWindow::Arm_Ctrl_OFF()
    {
        ros_topic_data = 103;
        ros_status_flag = true;
    }

    // Dynamixel control of camera pose
    void MainWindow::Cam_CtrlDXL()
    {
        ros_topic_data = 4;
        ros_status_flag = true;
    }
    void MainWindow::Cam_CtrlDXL_OFF()
    {
        ros_topic_data = 104;
        ros_status_flag = true;
    }

    // Front cam drive
    void MainWindow::Front_RawCam()
    {
        ros_topic_data = 5;
        ros_status_flag = true;
    }
    void MainWindow::Front_RawCam_OFF()
    {
        ros_topic_data = 105;
        ros_status_flag = true;
    }

    // Front cam drive
    void MainWindow::Front_YoloCam()
    {
        ros_topic_data = 6;
        ros_status_flag = true;
    }
    void MainWindow::Front_YoloCam_OFF()
    {
        ros_topic_data = 106;
        ros_status_flag = true;
    }

    // Side RealSense cam drive
    void MainWindow::SideCam_drive()
    {
        ros_topic_data = 7;
        ros_status_flag = true;
    }
    void MainWindow::SideCam_drive_OFF()
    {
        ros_topic_data = 107;
        ros_status_flag = true;
    }

    // web_video_server
    void MainWindow::Web_vd_server()
    {
        ros_topic_data = 8;
        ros_status_flag = true;
        //std::string command_html = "gnome-terminal -- firefox \"http://223.171.62.1:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed\" --new-window --width=640 --height=480";
        std::string command_html = "gnome-terminal -- google-chrome --new-window --window-size=640,480 \"http://223.171.62.1:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed\"";
        const char *c_html = command_html.c_str();
        system(c_html);
    }
    void MainWindow::Web_vd_server_OFF()
    {
        ros_topic_data = 108;
        ros_status_flag = true;
    }

    void MainWindow::Velodyne_Lidar()
    {
        ros_topic_data = 9;
        ros_status_flag = true;

        // Create a VisualizationManager
        manager_ = new rviz::VisualizationManager(render_panel_);
        render_panel_->initialize(manager_->getSceneManager(), manager_);

        // Create a Display for PointCloud2
        ptcloud2_display_ = manager_->createDisplay("rviz/PointCloud2", "Velodyne PointCloud", true);
        if (ptcloud2_display_)
        {
            // Set the topic name to '/velodyne_points'
            ptcloud2_display_->subProp("Topic")->setValue("/velodyne_points");
        }
        else
        {
            ROS_ERROR("Failed to create PointCloud2 display.");
        }

        manager_->setFixedFrame("velodyne");

        // Initialize the manager
        manager_->initialize();
        manager_->startUpdate();
    }
    void MainWindow::Velodyne_Lidar_OFF()
    {
        ros_topic_data = 109;
        ros_status_flag = true;

        delete manager_;
    }

    void MainWindow::AutoDrive()
    {
        ros_topic_data = 11;
        ros_status_flag = true;
    }
    void MainWindow::AutoDrive_OFF()
    {
        ros_topic_data = 111;
        ros_status_flag = true;
    }

    // Sceenshot
    void MainWindow::NUC1_screenshot_clicked(bool checked)
    {
        if(checked == true) {
            ros_topic_data = 1000;
            ros_status_flag = true;
            //dialog->show();   //add
        }
        else {
            dialog->close();    //add
        }
    }

    void MainWindow::NUC2_screenshot_clicked(bool checked)
    {
        if(checked == true){
            ros_topic_data = 2000;
            ros_status_flag = true;
            //dialog->show();   //add
        }
        else{
            dialog->close();    //add
        }
    }

    /*****************************************************************************
    ** Implementation [Menu]
    *****************************************************************************/

    void MainWindow::on_actionAbout_triggered() {
        QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
    }

    /*****************************************************************************
    ** Implementation [Configuration]
    *****************************************************************************/

    void MainWindow::ReadSettings() {
        QSettings settings("Qt-Ros Package", "gui_drok5");
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());
        QString master_url = settings.value("master_url", QString("http://10.60.3.9:11311/")).toString();
        QString host_url = settings.value("host_url", QString("10.60.3.9")).toString();
        ////QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
        ui.line_edit_master->setText(master_url);
        ui.line_edit_host->setText(host_url);
        //ui.line_edit_topic->setText(topic_name);
        bool remember = settings.value("remember_settings", false).toBool();
        ui.checkbox_remember_settings->setChecked(remember);
        bool checked = settings.value("use_environment_variables", false).toBool();
        ui.checkbox_use_environment->setChecked(checked);
        if ( checked ) {
            ui.line_edit_master->setEnabled(false);
            ui.line_edit_host->setEnabled(false);
            //ui.line_edit_topic->setEnabled(false);
        }
    }

    void MainWindow::WriteSettings() {
        QSettings settings("Qt-Ros Package", "gui_drok5");
        settings.setValue("master_url",ui.line_edit_master->text());
        settings.setValue("host_url",ui.line_edit_host->text());
        //settings.setValue("topic_name",ui.line_edit_topic->text());
        settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
        settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    }

    void MainWindow::closeEvent(QCloseEvent *event) {
        WriteSettings();
        QMainWindow::closeEvent(event);
    }
}
