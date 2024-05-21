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
#ifndef gui_drok5_MAIN_WINDOW_H
#define gui_drok5_MAIN_WINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <QPixmap>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "sc_dialog.hpp"

#include <stdlib.h>
#include <string>

namespace rviz
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

namespace gui_drok5 {

    /*
    // BEGIN_TUTORIAL
    // Class "MyViz" implements the top level widget for this example.
    class MyViz : public QWidget
    {
        Q_OBJECT    // Q_OBJECT macro

    public:
        MyViz( QWidget* parent = nullptr );
        virtual ~MyViz();

    private Q_SLOTS:
        void setThickness( int thickness_percent );
        void setCellSize( int cell_size_percent );

    private:
        rviz::VisualizationManager* manager_;
        rviz::RenderPanel* render_panel_;
        rviz::Display* grid_;
    };
    // END_TUTORIAL
    */

    class MainWindow : public QMainWindow
    {
        Q_OBJECT

    public:
        MainWindow( int argc, char** argv, QWidget *parent = nullptr ); //Combination with MainWindow and MyViz
        virtual ~MainWindow();

        void ReadSettings();    // Load up qt program settings at startup
        void WriteSettings();   // Save qt program settings when closing

        void closeEvent(QCloseEvent *event);    // Overloaded function
        void showNoMasterMessage();
        //void showButtonTestMessage();

    public Q_SLOTS:
        /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
        void on_actionAbout_triggered();
        void on_button_connect_clicked(bool check);
        //void on_button_test_clicked(bool check);
        void on_checkbox_use_environment_stateChanged(int state);

        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView();   // no idea why this can't connect automatically
        //void moveLeft();
        //void moveRight();

        /******************************************
        ** QNode Update
        *******************************************/
        void updateState();     // GUI signal update function
        void updateState_Sc();
        void getReady();
        //void updateVelodyne();
        //void updateBattery();

        /******************************************
        ** SLOTS of DroK5
        *******************************************/
        // Communication with Robot PC
        void Html();
        //void Html_Chrome();
        void Connect_PC_websocket();
        void Edit_html();

        // Start and Stop
        void Start();
        void All_stop();

        // Package execute
        void Joystick();
        void Joystick_OFF();
        void MD_driver();
        void MD_driver_OFF();
        void Arm_Ctrl();
        void Arm_Ctrl_OFF();
        void Cam_CtrlDXL();
        void Cam_CtrlDXL_OFF();
        void Front_RawCam();
        void Front_RawCam_OFF();
        void Front_YoloCam();
        void Front_YoloCam_OFF();
        void SideCam_drive();
        void SideCam_drive_OFF();
        void Web_vd_server();
        void Web_vd_server_OFF();
        void Velodyne_Lidar();
        void Velodyne_Lidar_OFF();
        //void Detect_KorOrNor();
        //void Detect_KorOrNor_OFF();
        void AutoDrive();
        void AutoDrive_OFF();

        // Sc
        void NUC1_screenshot_clicked(bool checked);
        void NUC2_screenshot_clicked(bool checked);

    private:
        Ui::MainWindowDesign ui;
        Sc_Dialog *dialog;
        QNode qnode;

        QPixmap m_lightimg[3];
        QPixmap m_readyimg[2];

        QPixmap KUDOS_img;
        QPixmap cat1_img;
        QPixmap cat2_img;

        QLineEdit qline;

        // For rviz visualization
        rviz::VisualizationManager* manager_;
        rviz::RenderPanel* render_panel_;
        rviz::Display* ptcloud2_display_;

  //QStringListModel* logging_model;
    };
}

#endif // gui_drok5_MAIN_WINDOW_H
