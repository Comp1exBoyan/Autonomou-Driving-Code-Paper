/**
 * @file /include/can_gui/main_window.hpp
 *
 * @brief Qt based gui for can_gui.
 *
 * @date November 2010
 **/
#ifndef can_gui_MAIN_WINDOW_H
#define can_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtWidgets/QMainWindow>
//#include <QtGui/QMainWindow>
//#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "canthread1_move.h"
#include <QTimer>
#include <QVBoxLayout>

//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>

#include "qrviz.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace can_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    //void on_button_connect_clicked(bool check );
    //void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void updateControl();//Send control msg
    void updateShow();//Send control msg

private Q_SLOTS:
    void on_pushButton_clicked();

    //void on_pushButton_2_clicked();

   // void displayText(VCI_CAN_OBJ *vci,DWORD dwRel);

    void on_pushButton_can_2_clicked();

    void on_pushButton_front_pressed();

    void on_pushButton_front_released();

    void on_pushButton_stop_clicked();

    void on_pushButton_stop_pressed();

    void on_pushButton_stop_released();

    void on_pushButton_left_pressed();

    void on_pushButton_left_released();

    void on_pushButton_right_pressed();

    void on_pushButton_right_released();

    void on_pushButton_Reset_clicked();

    void on_pushButton_Reset_pressed();

    void on_pushButton_Reset_released();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
   // void paintEvent ( QPaintEvent * event );

    void ros_sys_init();


public:
    canthread1 *canthread;
    QTimer *m_pTimer_sendcontrol;
    int control_MSG_MOD;//0: no msg; 1: front; 2: left: 3: right; 4 break.
    bool m_canisopen;
     int m_run0;

    QTimer *m_pTimer_updateShow;

    int m_val_bat;
    int m_val_spd;
    int m_val_ang;
    int m_val_tmp;


    //CANThread *canthread;

    can_gui::QRviz *mp_maprviz;
};





}  // namespace can_gui

namespace can_gui {

class RvizPlugin: public QObject
{
Q_OBJECT
public:
    RvizPlugin(QVBoxLayout* ui)
    {

//创建rviz的容器，并将该容器作为组建加入到ui内，其中关键class为VisualizationManager，是个管理类，起到控制创建rviz图层和设置坐标系的作用。
  /*   render_panel_=new rviz::RenderPanel;
      ui->addWidget(render_panel_);
      manager_=new rviz::VisualizationManager(render_panel_);
      render_panel_->initialize(manager_->getSceneManager(),manager_);
      manager_->initialize();
      manager_->startUpdate();
      manager_->setFixedFrame("/world");
//创建一个类型为rviz/PointCloud2的图层
 /*     rviz::Display *map_ = manager_->createDisplay("rviz/PointCloud2","pointCloud2",true);
      ROS_ASSERT(map_!=NULL);
      map_->subProp("Topic")->setValue("/points_map");
      map_->subProp("Invert Rainbow")->setValue("true");
      map_->subProp("Style")->setValue("Points");
      map_->subProp("Size (Pixels)")->setValue("2");
      //map_->subProp("Decay Time")->setValue("0");
      map_->subProp("Color Transformer")->setValue("FlatColor");
      map_->subProp("Queue Size")->setValue("10");
      map_->subProp("Alpha")->setValue("0.05");

//创建一个类型为rviz/PointCloud2的图层，用于接收topic为points_raw的点云数据，就是雷达实时扫描数据的展示

      rviz::Display *robot_=manager_->createDisplay("rviz/PointCloud2","pointCloud2",true);
      ROS_ASSERT(robot_!=NULL);
      robot_->subProp("Topic")->setValue("/points_raw");
      //robot_->subProp("Use Rainbow")->setValue(true);
      robot_->subProp("Size (Pixels)")->setValue("2");
      robot_->subProp("Style")->setValue("Points");
       //robot_->subProp("Autocompute Intensity Bounds")->setValue(true);
       //robot_->subProp("Color Transformer")->setValue("Intensity");
      robot_->subProp("Queue Size")->setValue("10");
      robot_->subProp("Alpha")->setValue("0.3");

       //robot_->subProp("Channel Name")->setValue("Intensity");

  //     robot_->subProp("Color Transformer")->setValue("Intensity");

  //     robot_->subProp("Color Transformer")->setValue("Intensity");

//这个是创建小车模型的图层，由urdf文件控制

       rviz::Display *car=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
       ROS_ASSERT(car!=NULL);
       car->subProp("Robot Description")->setValue("robot_description");

       manager_->startUpdate();
//设置整个地图的展示方式，如视角、距离、偏航等
      viewManager = manager_->getViewManager();
      viewManager->setRenderPanel(render_panel_);
      viewManager->setCurrentViewControllerType("rviz/ThirdPersonFollower");
      viewManager->getCurrent()->subProp("Target Frame")->setValue("/base_link");
      viewManager->getCurrent()->subProp("Near Clip Distance")->setValue("0.01");
      viewManager->getCurrent()->subProp("Focal Point")->setValue("1.90735e-06;-7.62939e-06;0");
      viewManager->getCurrent()->subProp("Focal Shape Size")->setValue("0.05");
      viewManager->getCurrent()->subProp("Focal Shape Fixed Size")->setValue("true");
*/
      //juli

  /*    viewManager->getCurrent()->subProp("Distance")->setValue("204.168");
      //chetou jiaodu
      viewManager->getCurrent()->subProp("Yaw")->setValue("1.7004");
      //fujiao
      viewManager->getCurrent()->subProp("Pitch")->setValue("0.770398");
*/
    }

//public slots:

public:

    rviz::RenderPanel* render_panel;// = new rviz::RenderPanel;
    rviz::VisualizationManager *manager;// =  new rviz::VisualizationManager(pointCloud_panel);
    rviz::ViewManager* viewManager;
    rviz::Display *map;
};



}



#endif // can_gui_MAIN_WINDOW_H
