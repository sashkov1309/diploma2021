//
// Created by alex on  26.11.20.
//

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <diploma_2021/logic/core/state_machine/main.hh>
#include <bcr_core/logic/world/world_handler.hh>
#include <thread>
#include <diploma_2021/logic/camera_processor/camera_processor.hh>
#include <diploma_2021/logic/image_check/image_check.hh>
#include <diploma_2021/transport/transport.hh>
#include <diploma_2021/logic/status/status_aggregator.hh>
#include <diploma_2021/logic/perception/map_perceptor_node.hh>

int main(int argc, char **argv) {
    diploma_2021::tools::logging::Logger(argv[0]).ExecutableLogLevel();
    rclcpp::init(argc, argv);
    auto worldHandler = std::make_unique<diploma_2021::logic::world::WorldHandler>(50);
    bcr::transport::visualization::VisualLogger::Instance();
    diploma_2021::transport::Transport transport;

    worldHandler->Proxy().UpdateAndWait(new diploma_2021::logic::world::RobotLoadAction);

    std::thread([&]{
        rclcpp::executors::SingleThreadedExecutor e;
        auto perceptor = std::make_shared<diploma_2021::logic::perception::MapPerceptorNode>(worldHandler.get(), transport);
        RCLCPP_INFO_STREAM(perceptor->get_logger(), "Map Perceptor Node Constructed");
        e.add_node(perceptor);
        e.spin();
        RCLCPP_INFO(perceptor->get_logger(), "Exiting Map Perceptor thread");
    }).detach();

    rclcpp::executors::SingleThreadedExecutor executor;//looks like this one can create more than one thread per node
    auto main = std::make_shared<diploma_2021::logic::core::Main>(worldHandler.get(), transport);
    RCLCPP_INFO(main->get_logger(), "Main Node Contructed");
    executor.add_node(main);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}