#ifndef LOG_MANAGER_LOGMANAGER_H
#define LOG_MANAGER_LOGMANAGER_H

#include <fstream>
#include <mutex>
#include <string>

namespace log_manager {

enum class LogLevel {
    INFO,
    WARN,
    ERROR
};

class LogManager {
public:
    static LogManager& Instance();

    bool InitFromConfig(const std::string& config_file_path);
    bool Init(const std::string& log_dir_path);

    void Log(LogLevel level, const char* file, int line, const std::string& message);

private:
    LogManager() = default;
    ~LogManager();
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

    std::string BuildLogFilePath(const std::string& log_dir_path) const;
    std::string GetCurrentTimeForLogLine() const;
    std::string GetCurrentTimeForFileName() const;
    std::string LevelToString(LogLevel level) const;
    std::string ExtractFileName(const char* file_path) const;
    std::string ReadLogDirFromConfig(const std::string& config_file_path) const;

private:
    mutable std::mutex mutex_;
    std::ofstream log_file_;
    bool initialized_ = false;
};

} // namespace log_manager

#endif // LOG_MANAGER_LOGMANAGER_H
