#include "log_manager/LogManager.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace log_manager {

LogManager::~LogManager()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
    }
}

LogManager& LogManager::Instance()
{
    static LogManager instance;
    return instance;
}

bool LogManager::InitFromConfig(const std::string& config_file_path)
{
    const std::string log_dir = ReadLogDirFromConfig(config_file_path);
    if (log_dir.empty()) {
        return Init("./logs");
    }
    return Init(log_dir);
}

bool LogManager::Init(const std::string& log_dir_path)
{
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        std::filesystem::create_directories(log_dir_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] [LogManager] create_directories failed: " << e.what() << std::endl;
        return false;
    }

    const std::string log_file_path = BuildLogFilePath(log_dir_path);
    log_file_.open(log_file_path, std::ios::out | std::ios::app);
    if (!log_file_.is_open()) {
        std::cerr << "[ERROR] [LogManager] failed to open log file: " << log_file_path << std::endl;
        return false;
    }

    initialized_ = true;
    return true;
}

void LogManager::Log(LogLevel level, const char* file, int line, const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string line_text = "[" + GetCurrentTimeForLogLine() + "] [" +
                                  LevelToString(level) + "] [" +
                                  ExtractFileName(file) + ":" + std::to_string(line) + "] " +
                                  message;

    if (level == LogLevel::ERROR) {
        std::cerr << line_text << std::endl;
    } else {
        std::cout << line_text << std::endl;
    }

    if (initialized_ && log_file_.is_open()) {
        log_file_ << line_text << std::endl;
        log_file_.flush();
    }
}

std::string LogManager::BuildLogFilePath(const std::string& log_dir_path) const
{
    return (std::filesystem::path(log_dir_path) / (GetCurrentTimeForFileName() + ".txt")).string();
}

std::string LogManager::GetCurrentTimeForLogLine() const
{
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto now_time_t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm local_tm {};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time_t);
#else
    localtime_r(&now_time_t, &local_tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string LogManager::GetCurrentTimeForFileName() const
{
    const auto now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local_tm {};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time_t);
#else
    localtime_r(&now_time_t, &local_tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d-%H_%M_%S");
    return oss.str();
}

std::string LogManager::LevelToString(LogLevel level) const
{
    switch (level) {
    case LogLevel::INFO:
        return "INFO";
    case LogLevel::WARN:
        return "WARN";
    case LogLevel::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

std::string LogManager::ExtractFileName(const char* file_path) const
{
    if (file_path == nullptr) return "unknown";
    return std::filesystem::path(file_path).filename().string();
}

std::string LogManager::ReadLogDirFromConfig(const std::string& config_file_path) const
{
    std::ifstream in(config_file_path);
    if (!in.is_open()) {
        std::cerr << "[WARN] [LogManager] config file open failed, fallback to ./logs: "
                  << config_file_path << std::endl;
        return "";
    }

    std::string line;
    const std::string key = "log_folder_path";
    while (std::getline(in, line)) {
        const auto pos = line.find(key);
        if (pos == std::string::npos) continue;

        const auto colon_pos = line.find(':', pos + key.size());
        if (colon_pos == std::string::npos) continue;

        std::string value = line.substr(colon_pos + 1);

        auto trim = [](std::string& s) {
            const size_t start = s.find_first_not_of(" \t\r\n");
            const size_t end = s.find_last_not_of(" \t\r\n");
            if (start == std::string::npos || end == std::string::npos) {
                s.clear();
                return;
            }
            s = s.substr(start, end - start + 1);
        };

        trim(value);
        if (!value.empty() && value.front() == '"' && value.back() == '"' && value.size() >= 2) {
            value = value.substr(1, value.size() - 2);
        }
        trim(value);
        return value;
    }

    std::cerr << "[WARN] [LogManager] key log_folder_path not found, fallback to ./logs" << std::endl;
    return "";
}

} // namespace log_manager
