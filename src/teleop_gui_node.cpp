#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>  // <-- Include for QPushButton
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TeleopWidget : public QWidget {
    Q_OBJECT

public:
    TeleopWidget(ros::NodeHandle& nh) {
        pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        QVBoxLayout* layout = new QVBoxLayout;

        QLabel* linearXLabel = new QLabel("Linear Velocity x");
        linearXSlider_ = new QSlider(Qt::Vertical);
        linearXSlider_->setRange(-100, 100);
        linearXSlider_->setValue(0);

        QLabel* linearYLabel = new QLabel("Linear Velocity y");
        linearYSlider_ = new QSlider(Qt::Horizontal);
        linearYSlider_->setRange(-100, 100);
        linearYSlider_->setValue(0);        

        QLabel* angularLabel = new QLabel("Angular Velocity");
        angularSlider_ = new QSlider(Qt::Horizontal);
        angularSlider_->setRange(-100, 100);
        angularSlider_->setValue(0);

        QPushButton* resetButton = new QPushButton("Reset");  // <-- Add reset button

        layout->addWidget(linearXLabel);
        layout->addWidget(linearXSlider_);
        layout->addWidget(linearYLabel);
        layout->addWidget(linearYSlider_);
        layout->addWidget(angularLabel);
        layout->addWidget(angularSlider_);
        layout->addWidget(resetButton);  // <-- Add to layout

        setLayout(layout);
        setWindowTitle("ROS Teleop GUI");

        connect(linearXSlider_, &QSlider::valueChanged, this, &TeleopWidget::sendCommand);
        connect(linearYSlider_, &QSlider::valueChanged, this, &TeleopWidget::sendCommand);
        connect(angularSlider_, &QSlider::valueChanged, this, &TeleopWidget::sendCommand);
        connect(resetButton, &QPushButton::clicked, this, &TeleopWidget::resetSliders);  // <-- Connect slot

        // Start timer to periodically send the last command (for sustained motion)
        startTimer(100); // 10 Hz
    }

protected:
    void timerEvent(QTimerEvent*) override {
        sendCommand();  // Keep publishing while slider is held
    }

private slots:
    void sendCommand() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linearXSlider_->value() / 100.0;
        cmd.linear.y = -linearYSlider_->value() / 100.0;
        cmd.angular.z = angularSlider_->value() / 100.0;
        pub_.publish(cmd);
    }

    void resetSliders() {
        linearXSlider_->setValue(0);
        linearYSlider_->setValue(0);
        angularSlider_->setValue(0);
        sendCommand();  // Publish zero command immediately
    }

private:
    QSlider* linearXSlider_;
    QSlider* linearYSlider_;
    QSlider* angularSlider_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_gui_node");
    ros::NodeHandle nh;

    QApplication app(argc, argv);
    TeleopWidget widget(nh);
    widget.show();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return app.exec();
}

#include "teleop_gui_node.moc"
