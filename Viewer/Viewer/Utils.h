// Copyright 2019 Salvatore Spoto

#ifndef UTILS_H
#define UTILS_H

#include <vector>

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 
#include <boost/filesystem.hpp>



/** This class is a collection of utility functions */
class Utils
{

public:

	/** 
	 * Lists all files in a directory with a given extension
     *
	 * @param directory the directory to look into
	 * @param extension the extension to look for
	 * @param outFileList will contains the list of all file names
	 */
	static void ListFilesInDirectory(const boost::filesystem::path& root, std::string_view extension, std::vector<std::string>& outFileList);
};

#endif
