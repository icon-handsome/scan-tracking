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
    /**
     * 获取单例实例。
     * @return LogManager 的全局唯一实例的引用。
     */
    static LogManager& Instance();

    /**
     * 从配置文件初始化日志管理器。
     * 该函数会从指定的配置文件中读取日志目录并尝试初始化日志文件。
     * @param config_file_path 配置文件的路径。
     * @return 成功返回 true，失败返回 false。
     */
    bool InitFromConfig(const std::string& config_file_path);

    /**
     * 使用指定的日志目录初始化日志管理器。
     * 如果目录有效并且能够创建/打开日志文件则初始化成功。
     * @param log_dir_path 日志目录的路径。
     * @return 成功返回 true，失败返回 false。
     */
    bool Init(const std::string& log_dir_path);

    /**
     * 写入一条日志记录。
     * 该方法会根据日志级别、源文件与行号以及消息内容格式化日志并写入到日志文件中。
     * 线程安全：内部使用互斥锁保护文件写入。
     * @param level 日志级别（INFO/WARN/ERROR）。
     * @param file 调用日志接口的源文件路径（通常使用 __FILE__）。
     * @param line 调用日志接口的行号（通常使用 __LINE__）。
     * @param message 日志消息内容。
     */
    void Log(LogLevel level, const char* file, int line, const std::string& message);

private:
    LogManager() = default;
    ~LogManager();
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

    /**
     * 根据给定的日志目录构建日志文件的完整路径（包含文件名）。
     * @param log_dir_path 日志目录路径。
     * @return 日志文件的完整路径字符串。
     */
    std::string BuildLogFilePath(const std::string& log_dir_path) const;

    /**
     * 获取用于日志行的当前时间字符串（用于每条日志的时间戳）。
     * @return 格式化后的时间字符串。
     */
    std::string GetCurrentTimeForLogLine() const;

    /**
     * 获取用于日志文件名的当前时间字符串（例如按日期或时间生成文件名）。
     * @return 格式化后的时间字符串，适合用于文件名。
     */
    std::string GetCurrentTimeForFileName() const;

    /**
     * 将日志级别枚举转换为可读的字符串表示。
     * @param level 日志级别枚举值。
     * @return 对应的字符串（例如 "INFO"/"WARN"/"ERROR"）。
     */
    std::string LevelToString(LogLevel level) const;

    /**
     * 从完整的文件路径中提取出文件名部分（去除目录路径）。
     * @param file_path 源文件的完整路径。
     * @return 仅包含文件名的字符串。
     */
    std::string ExtractFileName(const char* file_path) const;

    /**
     * 从配置文件中读取日志目录设置。
     * 如果配置文件不存在或无法解析，可能返回空字符串。
     * @param config_file_path 配置文件路径。
     * @return 配置中指定的日志目录，或空字符串表示未找到/错误。
     */
    std::string ReadLogDirFromConfig(const std::string& config_file_path) const;

private:
    mutable std::mutex mutex_;
    std::ofstream log_file_;
    bool initialized_ = false;
};

} // namespace log_manager

#endif // LOG_MANAGER_LOGMANAGER_H
