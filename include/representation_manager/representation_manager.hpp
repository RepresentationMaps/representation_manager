#include <vector>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include "representation_plugin_base/plugin_base.hpp"
#include "representation_plugins/semantic_plugin.hpp"

#include <pluginlib/class_loader.hpp>

#include "representation_manager/srv/add_plugin.hpp"
#include "representation_manager/srv/remove_plugin.hpp"

typedef std::shared_ptr<representation_plugins::PluginBase> RepPluginPtr;

namespace representation_manager{
	class RepresentationManager : public rclcpp::Node{
		protected:
			rclcpp::Service<representation_manager::srv::AddPlugin>::SharedPtr add_plugin_service_;
			rclcpp::Service<representation_manager::srv::RemovePlugin>::SharedPtr remove_plugin_service_;

			pluginlib::ClassLoader<representation_plugins::PluginBase> plugin_loader_;
			std::map<std::string, RepPluginPtr> plugins_;

			virtual void addPlugin(
				const std::shared_ptr<representation_manager::srv::AddPlugin::Request> request,
				std::shared_ptr<representation_manager::srv::AddPlugin::Response> response);
			virtual void removePlugin(
				const std::shared_ptr<representation_manager::srv::RemovePlugin::Request> request,
				std::shared_ptr<representation_manager::srv::RemovePlugin::Response> response);
		public:
			RepresentationManager();
	};
}