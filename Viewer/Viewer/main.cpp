// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"


/** 
 * Viewer application entry point 
 */
int main() 
{
	try 
	{
		ViewerApp::GetInstance().Run();
	}
	catch (const std::exception& e) 
	{
		BOOST_LOG_TRIVIAL(fatal) << e.what() << ".";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}