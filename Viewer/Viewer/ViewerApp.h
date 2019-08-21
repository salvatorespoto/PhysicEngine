#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/from_stream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/filter_parser.hpp>


#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <optional>

namespace logging = boost::log;

class ViewerApp {

private:
	boost::property_tree::ptree appProperties;

public:

	void run() {
		init();
	}

	void init() {

		// Init logs 
		boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
		boost::log::add_file_log
		(
			boost::log::keywords::file_name = "viewer.log",
			boost::log::keywords::format = "[%TimeStamp%] [%Severity%] %Message%"
		);
		boost::log::add_common_attributes();
		BOOST_LOG_TRIVIAL(info) << "Logs initialized.";

		// Load configuration file
		boost::property_tree::ini_parser::read_ini("config.ini", appProperties);
		BOOST_LOG_TRIVIAL(info) << "Configuration file 'config.ini' loaded.";

		// Set logs level
		BOOST_LOG_TRIVIAL(info) << "Setting logs level to " << appProperties.get<std::string>("Logs.Level") << ".";
		boost::log::core::get()->set_filter
		(
			boost::log::trivial::severity >= boost::lexical_cast<boost::log::trivial::severity_level>(appProperties.get<std::string>("Logs.Level"))
		);
	}
};