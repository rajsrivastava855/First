#include <thruster_controller/thruster_control.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "thruster_controller");
    ros::NodeHandle nh;
    QApplication a(argc, argv);
    ThrusterControl w(nh);
    w.show();
    ros::Rate rate(30);
    while(ros::ok() && w.isVisible())
    {
        rate.sleep();
        a.processEvents();
    }
}
