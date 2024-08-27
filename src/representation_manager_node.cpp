#include <rclcpp/rclcpp.hpp>

#include "representation_manager/representation_manager.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<representation_manager::RepresentationManager>());
	return 0;
}
