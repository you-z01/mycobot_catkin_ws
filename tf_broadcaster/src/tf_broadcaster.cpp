#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TfBroadcaster {
public:
    TfBroadcaster();
    ~TfBroadcaster();
    // Broadcast
    void BroadcastStaticTfFromCameraToMyCobot();

private:
    ros::NodeHandle nh_;
    // TF Broadcaster
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

TfBroadcaster::TfBroadcaster(){}
TfBroadcaster::~TfBroadcaster(){}

void TfBroadcaster::BroadcastStaticTfFromCameraToMyCobot() {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "joint6_flange"; // Ä©¶ËÖ´ÐÐÆ÷µÄ×ø±êÏµ
    transformStamped.child_frame_id = "camera_color_frame"; // Ïà»úµÄ×ø±êÏµ

    // ÊÖÑÛ±ê¶¨½á¹ûµÄÆ½ÒÆ²¿·Ö
    transformStamped.transform.translation.x = -0.0291947;
    transformStamped.transform.translation.y = -0.0481932;
    transformStamped.transform.translation.z = 0.0133379;

    // ÊÖÑÛ±ê¶¨½á¹ûµÄÐý×ª²¿·Ö£¬Ê¹ÓÃËÄÔªÊý
    transformStamped.transform.rotation.w = 0.999574831685977;
    transformStamped.transform.rotation.x = 0.016861922075053517;
    transformStamped.transform.rotation.y = -0.0012176168900470685;
    transformStamped.transform.rotation.z = 0.023756027719189287;

    // ·¢ËÍ×ª»»
    static_tf_broadcaster_.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster");
    TfBroadcaster tf_broadcaster;
    tf_broadcaster.BroadcastStaticTfFromCameraToMyCobot();

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
