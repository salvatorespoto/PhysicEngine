// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"
#include "include/PhysicEngine/Functions.h"

/** Application entry point */
int main() 
{

	glm::vec3 p0 = { 3.43f, 7.09f, 0.98f };
	glm::vec3 p1 = { 1.81f, 6.59f, 2.26f };
	glm::vec3 q0 = { 2.75f, 6.68f, 7.73f };
	glm::vec3 q1 = { 2.75f, 6.68f, -12.35f };

	PhysicEngine::Object3D out;
	PhysicEngine::GetEdgeEdgeIntersection(p0, p1, q0, q1, out);

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