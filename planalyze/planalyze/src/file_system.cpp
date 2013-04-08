#include <QDir>
#include <QFileInfo>

#include "file_system.h"

namespace FileSystem
{
    bool exists(const std::string& filename){
        return QFileInfo(filename.c_str()).exists();
    }

    bool isRelative(const std::string& filename) {
        return QFileInfo(filename.c_str()).isRelative();
    }

    std::string base(const std::string& filename)
    {
        return QFileInfo(filename.c_str()).completeBaseName().toStdString();
    }

    std::string extension(const std::string& filename)
    {
        return QFileInfo(filename.c_str()).suffix().toStdString();
    }

    std::string path(const std::string& filename)
    {
        return QFileInfo(filename.c_str()).path().toStdString();
    }

    std::string absolute(const std::string& filename)
    {
        return QFileInfo(filename.c_str()).absoluteFilePath().toStdString();
    }

    std::string relativePath(const std::string& filename, const std::string& path)
    {
        return QDir(path.c_str()).relativeFilePath(filename.c_str()).toStdString();
    }
};