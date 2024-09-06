#include "representation_manager/representation_manager.hpp"

namespace representation_manager{
	RepresentationManager::RepresentationManager():
		Node("representation_manager"),
		plugin_loader_("representation_plugins", "representation_plugins::PluginBase")
	{
		add_plugin_service_ = create_service<representation_manager::srv::AddPlugin>("add_plugin",
			std::bind(&RepresentationManager::addPlugin, this, std::placeholders::_1, std::placeholders::_2));
		remove_plugin_service_ = create_service<representation_manager::srv::RemovePlugin>("remove_plugin",
			std::bind(&RepresentationManager::removePlugin, this, std::placeholders::_1, std::placeholders::_2));
	}

	void RepresentationManager::addPlugin(
		const std::shared_ptr<representation_manager::srv::AddPlugin::Request> request,
		std::shared_ptr<representation_manager::srv::AddPlugin::Response> response){
		if(plugins_.find(request->plugin_name) == plugins_.end()){
			RepPluginPtr plugin = plugin_loader_.createSharedInstance(std::string("")+(request->plugin_name.c_str()));
			plugin->setup(shared_from_this(), request->plugin_name, request->threaded);
			plugin->initialize();
			plugins_[request->plugin_name] = plugin;
			response->success = true;
		}
		else{
			response->success = false;
		}
	}

	void RepresentationManager::removePlugin(
		const std::shared_ptr<representation_manager::srv::RemovePlugin::Request> request,
		std::shared_ptr<representation_manager::srv::RemovePlugin::Response> response){
		if(plugins_.find(request->plugin_name) != plugins_.end()){
			plugins_.erase(request->plugin_name);
			response->success = true;
		}
		else{
			response->success = false;
		}
	}
}