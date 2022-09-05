#include"opencv2/opencv.hpp"
#include"rclcpp/rclcpp.hpp"
#include"cv_bridge/cv_bridge.h"
#include"sensor_msgs/msg/image.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"sensor_msgs/image_encodings.hpp"
#include<chrono>
#include<string>
#include<memory>

using namespace cv;
using namespace std;
// 占位符
using std::placeholders::_1;

class follow_red_car : public rclcpp::Node
{
public:
    follow_red_car() : Node("follow_red_car")
    {
        // 读取camera画面
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("camera", 10, 
                                                                           std::bind(&follow_red_car::callBack, this, _1));
        // 创建发布
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
private:
    // 处理图像
    Mat tackle_img(Mat img){
        Mat hsv_img;
        Mat red_1;
        Mat red_2;
        Mat all_red;
        //Mat tackled;
        // 高斯降噪
        GaussianBlur(img, img, Size(7, 7), 0, 0);
        cvtColor(img, hsv_img, COLOR_BGR2HSV);
        vector<Mat> hsv;
        /*
            原本想用开运算和闭运算让图像内矩形更加完整清晰，
            但是会产生process has died, exit code -11的报错
            所以采用直方图均衡的方法让图像更加明显，然后用merge合并图像通道
        */
        split(hsv_img, hsv);
        equalizeHist(hsv[2], hsv[2]);
        merge(hsv, hsv_img);
        // 红色的h通道有两个范围
        inRange(hsv_img, Scalar(156, 43, 46), Scalar(180, 255, 255), red_1);
        inRange(hsv_img, Scalar(0, 43, 46), Scalar(10, 255, 255), red_2);
        add(red_1, red_2, all_red, Mat());

        //Mat element_1 = getStructuringElement(MORPH_RECT, Size(5, 5));
        //Mat element_2 = getStructuringElement(MORPH_RECT, Size(10, 10));
        //morphologyEx(all_red, tackled, MORPH_OPEN, element_1);
        //morphologyEx(tackled, tackled, MORPH_CLOSE, element_2);
        return all_red;
    }
    // 寻找矩形中心点
    void findCenter(Mat& img, double& center_x, double& center_y, double& width, bool& isFound){
        vector<Vec4i> hierarchy;
        vector<vector<Point>> contours;
        findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        // 判断有没有找到红色轮式机器人
        if(hierarchy.size()){
            isFound = true;
            RotatedRect rect = minAreaRect(contours.back());
            Rect r = rect.boundingRect();
            // 得到中心点以及矩形宽度
            center_x = rect.center.x;
            center_y = rect.center.y;
            width = r.width;
        }
        else{
            isFound = false;
        }
    }
    void callBack(const sensor_msgs::msg::Image & msg){
        cv_bridge::CvImagePtr CVPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat img = CVPtr->image;
        Mat redImg = tackle_img(img);
        double center_x, center_y, width;
        bool isFound;
        findCenter(redImg, center_x, center_y, width, isFound);
        geometry_msgs::msg::Twist pub;
        if(isFound){
            // 尽量让红色机器人在中心
            pub.angular.z = (redImg.cols / 2 - center_x) * 0.02;
            // 蓝色机器人速度，保持与红色机器人距离，调整300和0.01
            pub.linear.x = (300 - width) * 0.01;
        }
        // 没找到红色机器人就让蓝色机器人转圈
        else{
            pub.angular.z = 0.5;
            pub.linear.x = 0.5;
        }
        // 发布信息
        publisher_->publish(pub);
        waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<follow_red_car>());
    rclcpp::shutdown();
    return 0;
}