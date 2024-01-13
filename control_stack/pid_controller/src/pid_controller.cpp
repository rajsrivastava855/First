#include <pid_controller/pid_control.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh;
    QApplication a(argc, argv);
    PIDController w(nh);
    w.show();

    ros::Rate rate(30);
    while(ros::ok() && w.isVisible())
    {
        rate.sleep();
        a.processEvents();
    }

    //return a.exec();
}
