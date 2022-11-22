#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "crawler_ros_api_interface/srv/crawler_request_command.hpp"

typedef crawler_ros_api_interface::srv::CrawlerRequestCommand CrawlerCommandReq;
typedef const std::shared_ptr<CrawlerCommandReq::Request> CmdReqPtr;
typedef const std::shared_ptr<CrawlerCommandReq::Response> CmdRespPtr;

class CrawlerRosApi : public rclcpp::Node
{
private:
};

void setEngineForward(CmdReqPtr request, CmdRespPtr response)
{
    response->ack = request->val1 == 10  //
        && request->val2 == 11 //
        && request->val3 == 12;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto apiNode = rclcpp::Node::make_shared("crawler_ros_api");
    auto apiService = apiNode->create_service<CrawlerCommandReq>("setEngineForward", &setEngineForward);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crawler API service is up");

    rclcpp::spin(apiNode);
    rclcpp::shutdown();
    return 0;
}