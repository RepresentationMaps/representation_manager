#include <rclcpp/rclcpp.hpp>

#include "representation_manager/semantic_representation_manager.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	// Testing
	bool threaded = false;
	float voxel_size = 0.1;
	bool vertex_centered = false;

	auto node = std::make_shared<representation_manager::SemanticRepresentationManager>(
			threaded, voxel_size, vertex_centered);

	node->initialize();

	rclcpp::spin(node);
	return 0;
}
