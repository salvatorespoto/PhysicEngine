#include "ViewerApp.h"

int main() {

	ViewerApp viewer;

	try {
		viewer.run();
	}
	catch (const std::exception& e) {
		BOOST_LOG_TRIVIAL(fatal) << e.what() << ".";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}