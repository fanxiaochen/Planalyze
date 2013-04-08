#ifndef FileSystem_H
#define FileSystem_H

#include <string>

namespace FileSystem
{
bool exists(const std::string& filename);
bool isRelative(const std::string& filename);
std::string base(const std::string& filename);
std::string extension(const std::string& filename);
std::string path(const std::string& filename);
std::string absolute(const std::string& filename);
std::string relativePath(const std::string& filename, const std::string& path);
};

#endif // FileSystem_H
