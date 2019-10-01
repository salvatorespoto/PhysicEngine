// Copyright 2019 Salvatore Spoto

#include "Utils.h"


void Utils::ListFilesInDirectory(const boost::filesystem::path& root, std::string_view extension, std::vector<std::string>& outFileList)
{
	namespace fs = boost::filesystem;

	if (!fs::exists(root) || !fs::is_directory(root)) return;

	fs::recursive_directory_iterator it(root);
	fs::recursive_directory_iterator endit;
	while (it != endit)
	{
		std::string name = it->path().filename().string();
		if (fs::is_regular_file(*it) && it->path().extension() == extension.data()) 
			outFileList.push_back(it->path().filename().string());
		++it;
	}
}
