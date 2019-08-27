#include "Utils.h"

using std::vector;
using std::string;

vector<char> Utils::readFile(const string& filename) {
	
	std::ifstream file(filename, std::ios::ate | std::ios::binary);
	if (!file.is_open()) throw std::runtime_error("failed to open file!");
	
	size_t fileSize = (size_t)file.tellg();
	vector<char> buffer(fileSize);

	file.seekg(0);
	file.read(buffer.data(), fileSize);

	file.close();

	return buffer;
}