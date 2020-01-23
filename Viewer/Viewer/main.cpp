#include "Viewer.h"


/** 
 * Viewer application entry point 
 */
int main() 
{
	try 
	{
		Viewer::GetInstance().Run();
	}
	catch (const std::exception& e) 
	{
		BOOST_LOG_TRIVIAL(fatal) << e.what() << ".";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}