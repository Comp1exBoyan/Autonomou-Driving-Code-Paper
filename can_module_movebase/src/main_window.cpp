/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/can_module/main_window.hpp"
#include "canthread1_move.h"
#include "gl_data.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace can_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
//	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


    canthread = new canthread1;
   // connect(canthread,&CANThread::getProtocolData,this,&MainWindow::displayText);

	/*********************
	** Logging
	**********************/
//	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
 //   if ( ui.checkbox_remember_settings->isChecked() ) {
 //       on_button_connect_clicked(true);
//    }
    m_canisopen = false;

    m_pTimer_sendcontrol = new QTimer();
    m_pTimer_sendcontrol->setInterval(30);
    connect(m_pTimer_sendcontrol, SIGNAL(timeout()), this, SLOT(updateControl()));
    m_pTimer_sendcontrol->start();
    control_MSG_MOD = 0;
    //m_pTimer_sendcontrol->start();

    m_pTimer_updateShow = new QTimer();
    m_pTimer_updateShow->setInterval(30);
    connect(m_pTimer_updateShow, SIGNAL(timeout()), this, SLOT(updateShow()));
    m_pTimer_updateShow->start();

    m_val_bat=0;
    m_val_spd=0;
    m_val_ang=0;
    m_val_tmp=0;

    m_run0 = 1;

    qnode.init();// init the ros nodes

   // RvizPlugin rvi(this->ui.verticalLayout);

   // rviz::RenderPanel *render_panel_=new rviz::RenderPanel;

  //  rvi.render_panel = new rviz::RenderPanel;
  //  this->ui.verticalLayout->addWidget(rvi.render_panel);

    //mp_maprviz = new QRviz();


    ui.setupUi(this);
}

MainWindow::~MainWindow() {}

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





/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
     //   ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}
 int m_run0=1;
/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
 /*   QSettings settings("Qt-Ros Package", "can_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  //  ui.line_edit_master->setText(master_url);
  //  ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
 //   ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
*/}

void MainWindow::WriteSettings() {
/*    QSettings settings("Qt-Ros Package", "can_gui");
  //  settings.setValue("master_url",ui.line_edit_master->text());
 //   settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
 //   settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
*/
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace can_gui


void can_gui::MainWindow::on_pushButton_clicked()//send 1
{
    if( m_canisopen == false)//open can
    {
        if(canthread->initial_openCAN())
        {
            ui.pushButton->setText(tr("CAN opened"));
           // canthread->
            ui.pushButton->setDown(true);

            int ret;
            usleep(1);
            m_run0 = 1;
            ret=pthread_create(&(canthread->can_rec_th),NULL,receive_func,&m_run0);
            if(ret==0)
            {
                m_canisopen = true;
            }
            else
            {
                m_canisopen = false;
            }

        }
        else
        {
             ui.pushButton->setText(tr("CAN failed"));
             m_canisopen = false;
        }
    }
    else
    {


        //canthread->close_CAN();
        m_run0=0;//线程关闭指令。
        pthread_join(canthread->can_rec_th,NULL);
       // pthread_join(threadid,NULL);//等待线程关闭。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
        VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

        m_canisopen = false;
        ui.pushButton->setDown(false);
        ui.pushButton->setText(tr("openCAN"));
    }

}



/*
void can_gui::MainWindow::displayText(VCI_CAN_OBJ *vci,DWORD dwRel)
{
//    qDebug()<<"from thread slot:";
    QByteArray data;
    for(int i = 0;i<dwRel;i++)
    {
        for(int j = 0;j<8;j++)
            data.append(vci[i].Data[j]);
    }
    QString str(data);
    ui->textEdit_2->setText(str);
}
*/

void can_gui::MainWindow::ros_sys_init()
{


}



void can_gui::MainWindow::on_pushButton_can_2_clicked()
{

    if(m_canisopen)
    {
        m_run0=0;//线程关闭指令。
        pthread_join(canthread->can_rec_th,NULL);
       // pthread_join(threadid,NULL);//等待线程关闭。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
        VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。

    }
    this->close();
}

void can_gui::MainWindow::on_pushButton_front_pressed()
{
    control_MSG_MOD = 1;
   // m_pTimer_sendcontrol->start();
  //  this->ui.pushButton_front->setStyleSheet("background-color: rgb(0, 255, 0)");
}

void can_gui::MainWindow::on_pushButton_front_released()
{
    control_MSG_MOD = 0;
  //  m_pTimer_sendcontrol->start();
   // this->ui.pushButton_front->setStyleSheet("background-color: rgb(100, 155, 100)");
}

void can_gui::MainWindow::updateControl()
{
    if(m_canisopen == false) return;

    switch (control_MSG_MOD) {
    case 0:
        this->ui.label_controlcmd->setText(tr("NULL"));
        break;
    case 1:
        this->ui.label_controlcmd->setText(tr("forward"));
        canthread->goforward(1,5);
       // m_val_bat++;
       // m_val_spd++;
       // m_val_ang++;
       // m_val_tmp++;
        break;
    case 2:
         this->ui.label_controlcmd->setText(tr("Left"));
         canthread->controlfunc(1,1,20,100);
        break;
    case 3:
         this->ui.label_controlcmd->setText(tr("Right"));
         canthread->controlfunc(1,1,20,-100);
        break;
    case 4:
         this->ui.label_controlcmd->setText(tr("Stop"));
         canthread->brake();
        break;
    case 5:
         this->ui.label_controlcmd->setText(tr("Reset"));
         canthread->controlfunc(1,1,0,0);
        break;
    default:
        this->ui.label_controlcmd->setText(tr("NULL"));
        break;
    }

}

void can_gui::MainWindow::updateShow()
{
    gl_datalock1.lock();
    m_val_bat = gl_bat;
    m_val_tmp = gl_tmp;
    m_val_ang = gl_ang;
    m_val_spd = gl_spd/3.6*10;// spd  dm/s
    gl_datalock1.unlock();

    this->ui.lcdNumber_bat->display(m_val_bat);
    this->ui.lcdNumber_tmp->display(m_val_tmp);
    this->ui.progressBar_angle->setValue(m_val_ang);
    this->ui.progressBar_spd->setValue(m_val_spd);

    this->update();

}


void can_gui::MainWindow::on_pushButton_stop_clicked()
{
   //  control_MSG_MOD = 4;
}

void can_gui::MainWindow::on_pushButton_stop_pressed()
{
    control_MSG_MOD = 4;
}

void can_gui::MainWindow::on_pushButton_stop_released()
{
    control_MSG_MOD = 0;
}

void can_gui::MainWindow::on_pushButton_left_pressed()
{
    control_MSG_MOD = 2;
}

void can_gui::MainWindow::on_pushButton_left_released()
{
    control_MSG_MOD = 0;
}

void can_gui::MainWindow::on_pushButton_right_pressed()
{
    control_MSG_MOD = 3;
}

void can_gui::MainWindow::on_pushButton_right_released()
{
    control_MSG_MOD = 0;
}

void can_gui::MainWindow::on_pushButton_Reset_clicked()
{
    //control_MSG_MOD = 4;
}

void can_gui::MainWindow::on_pushButton_Reset_pressed()
{
    control_MSG_MOD = 5;
}

void can_gui::MainWindow::on_pushButton_Reset_released()
{
     control_MSG_MOD = 0;
}
