// Copyright 2019 Salvatore Spoto

#include "ViewerApp.h"


ViewerApp::ViewerApp() : Window(NULL), WindowWidth(0), WindowHeight(0), ViewportWidth(0), ViewportHeight(0)
{}


ViewerApp& ViewerApp::GetInstance() 
{
	static ViewerApp instance;
	return instance;
}


void ViewerApp::Run() 
{
	InitUtils();
	InitGLFW();
	InitOpenGL();
	SetupScene();
	//LoadMeshes();
	RenderLoop();
	ShutDown();
}


void ViewerApp::InitUtils() 
{
	// Load configuration file
	boost::property_tree::ini_parser::read_ini("config.ini", ViewerProperties);
	BOOST_LOG_TRIVIAL(info) << "Loaded config.ini";
	
	// Init logs 
	boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
	try 
	{
		std::string logFileName = ViewerProperties.get<std::string>("Logs.FileName");
		// Log to file instead of stdout if Logs.File properties existsi in config.ini
		boost::log::add_file_log
		(
			boost::log::keywords::file_name = logFileName,
			boost::log::keywords::format = "[%TimeStamp%] [%Severity%] %Message%"
		);
	}
	catch (const boost::property_tree::ptree_bad_path& e) 
	{
		BOOST_LOG_TRIVIAL(info) << e.what() << " " << "No logs file name specified, logging to stdout";
	}
	boost::log::add_common_attributes();
	BOOST_LOG_TRIVIAL(info) << "Logs initialized";

	// Set logs level
	BOOST_LOG_TRIVIAL(info) << "Setting logs level to " << ViewerProperties.get<std::string>("Logs.Level");
	boost::log::core::get()->set_filter
	(
		boost::log::trivial::severity 
			>= boost::lexical_cast<boost::log::trivial::severity_level>(ViewerProperties.get<std::string>("Logs.Level"))
	);

	CurrentPath = boost::filesystem::current_path().string();
}


void ViewerApp::InitGLFW() 
{
	glfwInit();
	glfwSetErrorCallback(glfwErrorCallback);
	
	WindowWidth = boost::lexical_cast<int>(ViewerProperties.get<std::string>("Application.Width"));
	WindowHeight = boost::lexical_cast<int>(ViewerProperties.get<std::string>("Application.Height"));
	
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	Window = glfwCreateWindow
	(
		WindowWidth,
		WindowWidth,
		"Physics engine viewer",
		nullptr,
		nullptr
	);
	
	if (!Window) 
	{
		glfwTerminate();
		throw std::runtime_error("Error initializing GLFW window");
	}

	ViewportWidth = WindowWidth;
	ViewportHeight = WindowHeight;
	
	glfwSetFramebufferSizeCallback(Window, FramebufferSizeCallback);
	
	BOOST_LOG_TRIVIAL(info) << "Application windows initialized with GLFW";
}


void ViewerApp::InitOpenGL() 
{
	glfwMakeContextCurrent(Window);
	glfwGetFramebufferSize(Window, &ViewportWidth, &ViewportHeight);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		BOOST_LOG_TRIVIAL(error) << "Failed to initialize extension loader library GLAD";
		return;
	}

	// Load and compile shaders 
	boost::filesystem::path shaderPath(CurrentPath);
	shaderPath.append(ViewerProperties.get<std::string>("Shader.Directory"));

	std::vector<std::string> vertexShaderList;
	Utils::ListFilesInDirectory(shaderPath, Shader::VERTEX_SHADER_EXTENSION, vertexShaderList);
	for (std::string vertexShaderName : vertexShaderList)
	{
		ShadersMap[vertexShaderName] = new Shader(boost::filesystem::path(shaderPath).append(vertexShaderName));
	}

	std::vector<std::string> fragmentShaderList;
	Utils::ListFilesInDirectory(shaderPath, Shader::FRAGMENT_SHADER_EXTENSION, fragmentShaderList);
	for (std::string fragmentShaderName : fragmentShaderList)
	{
		ShadersMap[fragmentShaderName] = new Shader(boost::filesystem::path(shaderPath).append(fragmentShaderName));
	}

	// TODO evaluate if move this code in a separate function / class that handle shader programs
	// Build shader programs 
	ShaderProgram = glCreateProgram();
	glAttachShader(ShaderProgram, ShadersMap["shader.vs"]->Id);
	glAttachShader(ShaderProgram, ShadersMap["shader.fs"]->Id);
	glLinkProgram(ShaderProgram);
	
	int success;
	char infoLog[1024];
	glGetProgramiv(ShaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(ShaderProgram, 512, NULL, infoLog);
		BOOST_LOG_TRIVIAL(error) << "Failed to build shader program";
	}

	BOOST_LOG_TRIVIAL(info) << "OpenGL initialized";

	// Init camera values
	CameraFPS.TranslationSpeed = 0.01f;
	CameraFPS.RotationSpeed = 0.001f;
	CameraFPS.ZoomSpeed = 0.01f;
	TrackBall.IsTrackBall = true;
}


void ViewerApp::FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}


void ViewerApp::LoadMeshes()
{
	boost::filesystem::path meshDirectoryPath(CurrentPath);
	meshDirectoryPath.append(ViewerProperties.get<std::string>("Mesh.Directory"));
	
	std::vector<std::string> objFilesList;
	Utils::ListFilesInDirectory(meshDirectoryPath, ".obj", objFilesList);
	for (std::string objFileName : objFilesList)
	{
		MeshMap[objFileName] = new Mesh3D(boost::filesystem::path(meshDirectoryPath).append(objFileName));
	}
}


void ViewerApp::ProcessInput()
{
	HandleKeyboardInput();
	HandleMouseInput();
}


void ViewerApp::UpdateCamera()
{
	// Compute the projection matrix
	glm::mat4 projection = glm::perspective(glm::radians(CameraFPS.FieldOfViewValue),
		(float) ViewportWidth / (float) ViewportHeight, 0.10f, 100.0f);

	// Compute the view matrix
	glm::mat4 view = CameraFPS.GetViewMatrix();
}


void ViewerApp::HandleKeyboardInput() 
{
	// ESC: exit application
	if (glfwGetKey(Window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(Window, true);

	if (glfwGetKey(Window, GLFW_KEY_1) == GLFW_PRESS)
		RenderModel = !RenderModel;
	if (glfwGetKey(Window, GLFW_KEY_2) == GLFW_PRESS)
		RenderBoundingBox = !RenderBoundingBox;
	if (glfwGetKey(Window, GLFW_KEY_3) == GLFW_PRESS)
		RenderConvexHull = !RenderConvexHull;

	// Handle camera movements
	if (glfwGetKey(Window, GLFW_KEY_W) == GLFW_PRESS)
		CameraFPS.Translate(Camera::FORWARD, 1.0f);
	if (glfwGetKey(Window, GLFW_KEY_S) == GLFW_PRESS)
		CameraFPS.Translate(Camera::BACKWARD, 1.0f);
	if (glfwGetKey(Window, GLFW_KEY_A) == GLFW_PRESS)
		CameraFPS.Translate(Camera::LEFT, 1.0f);
	if (glfwGetKey(Window, GLFW_KEY_D) == GLFW_PRESS)
		CameraFPS.Translate(Camera::RIGHT, 1.0f);
	if (glfwGetKey(Window, GLFW_KEY_R) == GLFW_PRESS)
		CameraFPS.Translate(Camera::UP, 1.0f);
	if (glfwGetKey(Window, GLFW_KEY_F) == GLFW_PRESS)
		CameraFPS.Translate(Camera::DOWN, 1.0f);

	// Handle selected object rotation
	if(draggingMesh) 
	{
		if (SelectedMesh != nullptr)
		{
			if (glfwGetKey(Window, GLFW_KEY_I) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, 0.01f, glm::vec3(1.0f, 0.0f, 0.0f));
			if (glfwGetKey(Window, GLFW_KEY_K) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, -0.01f, glm::vec3(1.0f, 0.0f, 0.0f));
			if (glfwGetKey(Window, GLFW_KEY_J) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, 0.01f, glm::vec3(0.0f, 1.0f, 0.0f));
			if (glfwGetKey(Window, GLFW_KEY_L) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, -0.01f, glm::vec3(0.0f, 1.0f, 0.0f));
			if (glfwGetKey(Window, GLFW_KEY_U) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, 0.01f, glm::vec3(0.0f, 0.0f, 1.0f));
			if (glfwGetKey(Window, GLFW_KEY_O) == GLFW_PRESS)
				SelectedMesh->PhysicRigidBody->OrientationQuaternion =
					glm::rotate(SelectedMesh->PhysicRigidBody->OrientationQuaternion, -0.01f, glm::vec3(0.0f, 0.0f, 1.0f));
		}	
	}
}


void ViewerApp::HandleMouseInput()
{
	double xCoordinate, yCoordinate;
	glfwGetCursorPos(Window, &xCoordinate, &yCoordinate);

	// Ignore first input coordinates because we do not have a previous valid value
	if (MouseXCoordinate == 99999.0f || MouseYCoordinate == 99999.0f) {
		MouseXCoordinate = xCoordinate;
		MouseYCoordinate = yCoordinate;
		return;
	}
	
	// Compute offsets of the mouse movement
	// xOffset and yOffset will be the the yaw and pitch of the camera movement
 	double xOffset = xCoordinate - MouseXCoordinate;
	double yOffset = yCoordinate - MouseYCoordinate;

	MouseXCoordinate = xCoordinate;
	MouseYCoordinate = yCoordinate;

	if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS) 
	{
		
		// Left mouse click is the First Person Shooter camera
		// yaw and pitch are counterclockwise movement of the camera around Y ans X axis 
		// so for example a positive xOffset (mouse move to the right) have to be inverted to get
		// a rotation to the right (clockwise). The same for pitch.
		CameraFPS.Rotate(-xOffset, -yOffset, 0.0f, 1.0f);
	}
	else if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) 
	{
		// Right mouse button rotate the trackball 
		TrackBall.Rotate(-xOffset, -yOffset, 0.0f, 1.0f);
	}
	if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) 
	{
		draggingMesh = true;
	}
	if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_1) == GLFW_RELEASE)
	{
		draggingMesh = false;
	}
	
	if(draggingMesh) 
	{
		// Get the arriving point of the mouse onto the 
		if (SelectedMesh != nullptr) 
		{
			// Traslate the mesh on the perpendi plane to the camera front vector
			glm::vec3 traslation = (TrackBall.UpVector * (float)(-yOffset / 5.0f)) + (TrackBall.RightVector * (float)(xOffset / 5.0f));
			SelectedMesh->PhysicRigidBody->Position += traslation;
		}
	}
}


void ViewerApp::SetupScene() 
{
	// Load meshes and put them into the 3D world
	boost::filesystem::path meshDirectoryPath(CurrentPath);
	meshDirectoryPath.append(ViewerProperties.get<std::string>("Mesh.Directory"));

	std::vector<std::string> objFilesList;
	Utils::ListFilesInDirectory(meshDirectoryPath, ".obj", objFilesList);
	
	MeshMap["floor"] = new Mesh3D(boost::filesystem::path(meshDirectoryPath).append("floor.obj"));
	MeshMap["cube"] = new Mesh3D(boost::filesystem::path(meshDirectoryPath).append("cube.obj"));	
	//MeshMap["suzanne"] = new Mesh3D(boost::filesystem::path(meshDirectoryPath).append("suzanne.obj"));
	//MeshMap["tetra"] = new Mesh3D(boost::filesystem::path(meshDirectoryPath).append("tetra.obj"));

	
	// Set up phisics 
	physicEngine.SetStartingTime(0);
	physicEngine.SetDeltaTime(1.0f / 30.0f);
	
	/*MeshMap["suzanne"]->PhysicRigidBody->SetState(
		glm::vec3(0.0f, 7.0f, 0.0f),
		glm::identity<glm::quat>(),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.0f)
	);

	MeshMap["cube"]->PhysicRigidBody->SetState(
		glm::vec3(-3.0f, 7.0f, 0.0f),
		glm::identity<glm::quat>(),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.0f)
	);

	MeshMap["tetra"]->PhysicRigidBody->SetState(
		glm::vec3(3.0f, 7.0f, 0.0f),
		glm::identity<glm::quat>(),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 0.0f)
	);
	*/
	//MeshMap["cube"]->PhysicRigidBody->SetForceFunction(GravityForce);
	
	MeshMap["suzanne"]->PhysicRigidBody->LinearMomentum = MeshMap["suzanne"]->PhysicRigidBody->Mass * glm::vec3(0.0f, -0.3f, 0.0f);
	MeshMap["suzanne"]->PhysicRigidBody->AngularMomentum = MeshMap["suzanne"]->PhysicRigidBody->Mass * glm::vec3(0, 0.1f, 0);

	MeshMap["cube"]->PhysicRigidBody->LinearMomentum = MeshMap["cube"]->PhysicRigidBody->Mass * glm::vec3(0.0f, -0.5f, 0.0f);
	MeshMap["cube"]->PhysicRigidBody->AngularMomentum = MeshMap["cube"]->PhysicRigidBody->Mass * glm::vec3(0.1f, 0.1f, 0);

	MeshMap["tetra"]->PhysicRigidBody->LinearMomentum = MeshMap["tetra"]->PhysicRigidBody->Mass * glm::vec3(0.0f, -0.1f, 0.0f);
	MeshMap["tetra"]->PhysicRigidBody->AngularMomentum = MeshMap["tetra"]->PhysicRigidBody->Mass * glm::vec3(0, 0.1f, 0.1f);


	for (std::pair<std::string, Mesh3D*> p : MeshMap)
	{
		physicEngine.AddRigidBody(p.second->PhysicRigidBody);
	}

	// Position camera
	CameraFPS.SetPosition(glm::vec3(0.0f, 4.0f, 30.0f));
//	TrackBall.Rotate(-200.0f, -120.0f, 0.0f, 1.0f);
}

void ViewerApp::RenderLoop() 
{
	BOOST_LOG_TRIVIAL(debug) << "Entering mainloop";

	while (!glfwWindowShouldClose(Window)) 
	{
		glfwGetFramebufferSize(Window, &ViewportWidth, &ViewportHeight);
		ProcessInput();

		// Update physic matricies
		/*
		for (std::pair<std::string, Mesh3D*> p : MeshMap) 
		{
			p.second->PhysicRigidBody->ModelMatrix = p.second->GetModelMatrix();
		}*/
		
		physicEngine.Tick();
		for (std::pair<std::string, Mesh3D*> p : MeshMap)
		{
			p.second->SetModelMatrix(p.second->PhysicRigidBody->ModelMatrix);
		}

		DrawWorld();
		glfwSwapBuffers(Window);
		glfwPollEvents();
	}

	BOOST_LOG_TRIVIAL(debug) << "Exiting mainloop";
}


void ViewerApp::DrawWorld()
{
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(ShaderProgram);

	// Setup light
	glm::vec3 lightPosition(1.2f, 1.0f, 2.0f);
	glUniform3f(glGetUniformLocation(ShaderProgram, "lightPosition"), lightPosition.x, lightPosition.y, lightPosition.z);
	float ambientStrength = 1.0f;
	glUniform1f(glGetUniformLocation(ShaderProgram, "ambientStrength"), ambientStrength);
	glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
	glUniform3f(glGetUniformLocation(ShaderProgram, "lightColor"), lightColor.x, lightColor.y, lightColor.z);
	glm::vec3 meshColor(1.0f, 0.5f, 0.31f);
	glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);

	// Setup View Matrix
	GLuint viewLocation = glGetUniformLocation(ShaderProgram, "view");
	glm::mat4 view = CameraFPS.GetViewMatrix();
	glm::mat4 viewTrackball = TrackBall.GetViewMatrix();
	glUniformMatrix4fv(viewLocation, 1, GL_FALSE, glm::value_ptr(view * viewTrackball));
	
	// Setup Projection matrix
	ProjectionMatrix = glm::perspective (glm::radians(45.0f), (float)ViewportWidth / (float)ViewportHeight, 0.1f, 100.0f);
	GLuint projectionLocation = glGetUniformLocation(ShaderProgram, "projection");
	glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, glm::value_ptr(ProjectionMatrix));
	
	float minDistance = 100000.0f;
	if (!draggingMesh)
	{
		SelectedMesh = nullptr;
		for(std::pair<std::string, Mesh3D*> p : MeshMap)
		{
			Mesh3D* mesh = p.second;
			float distance;
			if (CheckMouseObjectsIntersection(*mesh, distance))
			{
				if (distance < minDistance) 
				{
					minDistance = distance;
					SelectedMesh = mesh;
				}
			}
		}
	}

	// Render meshes
	std::for_each(MeshMap.begin(), MeshMap.end(),
		[this](std::pair<std::string, Mesh3D*> p)
		{
			Mesh3D* mesh = p.second;
			if(mesh == SelectedMesh) 
			{
				glm::vec3 meshColor(0.31f, 0.5f, 1.0f);
				glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);
			}
			else 
			{
				glm::vec3 meshColor(1.0f, 0.5f, 0.31f);
				glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);
			}

			// Colliding mesh
			if (mesh->PhysicRigidBody->IsColliding)
			{
				glm::vec3 meshColor(1.0f, 0.0f, 0.0f);
				glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);
			}

			mesh->RenderModel = RenderModel;
			mesh->RenderBoundingBox = RenderBoundingBox;
			mesh->RenderConvexHull = RenderConvexHull;
			mesh->Draw(ShaderProgram);
		});

	// Draw colliding sets
	std::for_each(physicEngine.GetContacts().begin(), physicEngine.GetContacts().end(),
		[this](PhysicEngine::Contact p)
		{
				glm::vec3 meshColor(0.0f, 1.0f, 0.0f);
				glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);
				
				std::vector<Vertex3D> vertices;
				vertices.push_back(Vertex3D{ p.Point } );
				std::vector<unsigned int> elements;
				elements.push_back(0);

				GLuint vaoId, vboId, eboId;
				glGenVertexArrays(1, &vaoId);
				glGenBuffers(1, &vboId);
				glGenBuffers(1, &eboId);

				glBindVertexArray(vaoId);

				glBindBuffer(GL_ARRAY_BUFFER, vboId);
				glBufferData(GL_ARRAY_BUFFER, 1 * sizeof(Vertex3D), &vertices[0], GL_STATIC_DRAW);

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboId);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, 1 * sizeof(unsigned int), &elements[0], GL_STATIC_DRAW);

				glEnableVertexAttribArray(0);

				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
				glBindVertexArray(0);			

				// Set up model transform
				GLuint modelLocation = glGetUniformLocation(ShaderProgram, "model");
				glm::mat4 identity = glm::mat4(1.0);
				glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(identity));

				glBindVertexArray(vaoId);
				glPointSize(5.0f);
				glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, 0);
				glBindVertexArray(0);



		});


	// Draw edgi
	std::for_each(physicEngine.GetContacts().begin(), physicEngine.GetContacts().end(),
		[this](PhysicEngine::Contact p)
		{
			glm::vec3 meshColor(0.0f, 0.0f, 1.0f);
			glUniform3f(glGetUniformLocation(ShaderProgram, "meshColor"), meshColor.x, meshColor.y, meshColor.z);

			std::vector<Vertex3D> vertices;
			vertices.push_back(Vertex3D{ p.E00});
			vertices.push_back(Vertex3D{ p.E01 });
			vertices.push_back(Vertex3D{ p.E10 });
			vertices.push_back(Vertex3D{ p.E11 });
			std::vector<unsigned int> elements;
			elements.push_back(0);
			elements.push_back(1);
			elements.push_back(2);
			elements.push_back(3);

			GLuint vaoId, vboId, eboId;
			glGenVertexArrays(1, &vaoId);
			glGenBuffers(1, &vboId);
			glGenBuffers(1, &eboId);

			glBindVertexArray(vaoId);

			glBindBuffer(GL_ARRAY_BUFFER, vboId);
			glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(Vertex3D), &vertices[0], GL_STATIC_DRAW);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboId);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, 4 * sizeof(unsigned int), &elements[0], GL_STATIC_DRAW);

			glEnableVertexAttribArray(0);

			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);
			glBindVertexArray(0);

			// Set up model transform
			GLuint modelLocation = glGetUniformLocation(ShaderProgram, "model");
			glm::mat4 identity = glm::mat4(1.0);
			glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(identity));

			glBindVertexArray(vaoId);
			glDrawElements(GL_LINES, 4, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);

		});

	glUseProgram(0);
}


bool ViewerApp::CheckMouseObjectsIntersection(Mesh3D& mesh, float& outDistance) {

	// The start and end ray positions, in Normalized Device Coordinates (NDC)
	float rayXNDC = ((float)MouseXCoordinate / (float)ViewportWidth - 0.5f) * 2.0f;
	float rayYNDC = ((float)MouseYCoordinate / (float)ViewportHeight - 0.5f) * 2.0f;
	glm::vec4 rayStartNDC(rayXNDC , -rayYNDC, -1.0, 1.0f);
	glm::vec4 rayEndNDC(rayXNDC, -rayYNDC, 0.0, 1.0f);

	// Get the inverse transofrmation from NDC -> camera space -> wold space
	// So inverse(ProjectionMatrix) goes from NDC to Camera Space.
	glm::mat4 InverseProjectionMatrix = glm::inverse(ProjectionMatrix * CameraFPS.GetViewMatrix() * TrackBall.GetViewMatrix());
	glm::vec4 rayStartWorld = InverseProjectionMatrix * rayStartNDC; 
	rayStartWorld /= rayStartWorld.w;
	glm::vec4 rayEndWorld = InverseProjectionMatrix * rayEndNDC; 
	rayEndWorld /= rayEndWorld.w;
	glm::vec3 rayDirectionWorld = glm::normalize(rayEndWorld - rayStartWorld);
	
	return PhysicEngine::TestRayBoundingBoxIntersection(rayStartWorld, rayDirectionWorld, 
		mesh.PhysicRigidBody->boundingBox, mesh.GetModelMatrix(), outDistance);
}


void ViewerApp::ShutDown() 
{

	glfwDestroyWindow(Window);
	BOOST_LOG_TRIVIAL(debug) << "GLFW Windows destroyed";
	
	glfwTerminate();
	BOOST_LOG_TRIVIAL(debug) << "GLFW terminated";

	BOOST_LOG_TRIVIAL(info) << "All resources freed";
}


void glfwErrorCallback(int error, const char* description) 
{
	BOOST_LOG_TRIVIAL(error) << "GLFW Error: " << description;
}