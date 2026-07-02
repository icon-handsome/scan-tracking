#include "scan_tracking/common/logger.h"

#include <QtCore/QDateTime>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

namespace scan_tracking::common {

namespace {

constexpr char kUtf8Bom[] = "\xEF\xBB\xBF";

thread_local int g_log_handler_depth = 0;

const char* safeCategoryName(const QMessageLogContext& context)
{
    if (context.category != nullptr && context.category[0] != '\0') {
        return context.category;
    }
    return "default";
}

bool isMutedHmiLogCategory(const char* category)
{
    if (category == nullptr || category[0] == '\0') {
        return false;
    }
    return std::strcmp(category, "hmi.server") == 0
        || std::strcmp(category, "hmi.session") == 0;
}

std::string dailyLogFilePath(const std::string& log_dir, const QDate& date)
{
    return log_dir + "/scan_tracking_"
        + date.toString(QStringLiteral("yyyy-MM-dd")).toStdString() + ".txt";
}

std::string runLogFileBaseName(const QDateTime& start_time)
{
    return (QStringLiteral("scan_tracking_run_")
            + start_time.toString(QStringLiteral("yyyy-MM-dd_HH-mm-ss"))
            + QStringLiteral(".txt"))
        .toStdString();
}

std::string runLogFilePath(const std::string& log_dir, const QDateTime& start_time)
{
    return log_dir + '/' + runLogFileBaseName(start_time);
}

std::string resolveUniqueRunLogFilePath(const std::string& log_dir, const QDateTime& start_time)
{
    std::string candidate = runLogFilePath(log_dir, start_time);
    if (!std::filesystem::exists(candidate)) {
        return candidate;
    }

    const std::string base_name = runLogFileBaseName(start_time);
    const std::size_t dot_pos = base_name.rfind('.');
    const std::string stem = base_name.substr(0, dot_pos);
    const std::string extension = base_name.substr(dot_pos);

    for (int suffix = 2; suffix < 1000; ++suffix) {
        candidate = log_dir + '/' + stem + '_' + std::to_string(suffix) + extension;
        if (!std::filesystem::exists(candidate)) {
            return candidate;
        }
    }

    return runLogFilePath(log_dir, start_time);
}

bool ensureLogDirectory(const std::string& log_dir)
{
    if (log_dir.empty()) {
        return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return false;
    }
    return std::filesystem::is_directory(log_dir);
}

void writeConsoleLine(QtMsgType type, const char* data, std::size_t size)
{
    FILE* out = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) ? stderr : stdout;
    std::fwrite(data, 1, size, out);
    std::fwrite("\n", 1, 1, out);
    std::fflush(out);
}

std::string buildLogLine(
    const char* time_stamp,
    const char* severity,
    const char* category,
    const char* msg_utf8,
    const char* source_suffix)
{
    std::string line;
    line.reserve(
        std::strlen(time_stamp) + std::strlen(severity) + std::strlen(category)
        + std::strlen(msg_utf8) + (source_suffix != nullptr ? std::strlen(source_suffix) : 0) + 32);
    line += '[';
    line += time_stamp;
    line += "] [";
    line += severity;
    line += "] [";
    line += category;
    line += "] ";
    line += msg_utf8;
    if (source_suffix != nullptr && source_suffix[0] != '\0') {
        line += source_suffix;
    }
    return line;
}

std::string sourceLocationSuffix(const QMessageLogContext& context)
{
    if (context.file == nullptr || context.line <= 0) {
        return {};
    }

    std::string suffix = " (";
    suffix += context.file;
    suffix += ':';
    suffix += std::to_string(context.line);
    suffix += ')';
    return suffix;
}

void emitMinimalFallback(QtMsgType type, const QMessageLogContext& context, const QByteArray& msg_utf8)
{
    const char* severity = "UNK";
    switch (type) {
        case QtDebugMsg: severity = "DBG"; break;
        case QtInfoMsg: severity = "INF"; break;
        case QtWarningMsg: severity = "WRN"; break;
        case QtCriticalMsg: severity = "CRT"; break;
        case QtFatalMsg: severity = "FTL"; break;
        default: break;
    }
    const std::string suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : std::string{};
    const std::string line = buildLogLine(
        "reentrant",
        severity,
        safeCategoryName(context),
        msg_utf8.constData(),
        suffix.empty() ? nullptr : suffix.c_str());
    writeConsoleLine(type, line.c_str(), line.size());
}

}  // namespace

Logger* Logger::instance_ = nullptr;
QtMessageHandler Logger::previous_handler_ = nullptr;

Logger::Logger(const QString& log_dir)
    : log_dir_(log_dir.toStdString()),
      min_level_(QtDebugMsg)
{
    if (!ensureLogDirectory(log_dir_)) {
        std::cerr << "严重错误：Logger 无法创建日志目录：" << log_dir_ << "\n";
    }
    const QDateTime start_time = QDateTime::currentDateTime();
    openDailyLogFile(start_time.date());
    openRunLogFile(start_time);
}

Logger::~Logger()
{
    if (run_log_file_ != nullptr) {
        const std::string end_stamp =
            QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toStdString();
        const std::string footer = buildLogLine(
            end_stamp.c_str(),
            "INF",
            "logger",
            "===== 进程会话结束 =====",
            nullptr);
        writeLogLineToFile(run_log_file_, footer);
        std::fclose(run_log_file_);
        run_log_file_ = nullptr;
    }

    if (daily_log_file_ != nullptr) {
        std::fclose(daily_log_file_);
        daily_log_file_ = nullptr;
    }
}

void Logger::initialize(const QString& log_dir)
{
    if (instance_ != nullptr) {
        return;
    }

    instance_ = new Logger(log_dir);
    previous_handler_ = qInstallMessageHandler(Logger::messageHandler);
}

void Logger::cleanup()
{
    if (instance_ == nullptr) {
        return;
    }

    QtMessageHandler upstream = previous_handler_;
    qInstallMessageHandler(upstream);
    previous_handler_ = nullptr;

    Logger* doomed = instance_;
    instance_ = nullptr;

    {
        std::lock_guard<std::mutex> lock(doomed->mutex_);
    }
    delete doomed;
}

Logger* Logger::instance()
{
    return instance_;
}

void Logger::setMinLevel(QtMsgType level)
{
    std::lock_guard<std::mutex> lock(mutex_);
    min_level_ = level;
}

int Logger::getSeverityLevel(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return 0;
        case QtInfoMsg: return 1;
        case QtWarningMsg: return 2;
        case QtCriticalMsg: return 3;
        case QtFatalMsg: return 4;
        default: return 0;
    }
}

const char* Logger::getLogSeverity(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return "DBG";
        case QtInfoMsg: return "INF";
        case QtWarningMsg: return "WRN";
        case QtCriticalMsg: return "CRT";
        case QtFatalMsg: return "FTL";
        default: return "UNK";
    }
}

void Logger::writeLogLineToFile(FILE* file, const std::string& line)
{
    if (file == nullptr) {
        return;
    }

    std::fwrite(line.c_str(), 1, line.size(), file);
    std::fwrite("\r\n", 1, 2, file);
    std::fflush(file);
}

void Logger::openDailyLogFile(const QDate& target_date)
{
    if (!target_date.isValid()) {
        return;
    }

    const std::string file_path = dailyLogFilePath(log_dir_, target_date);

    if (daily_log_file_ != nullptr) {
        std::fclose(daily_log_file_);
        daily_log_file_ = nullptr;
    }

    daily_log_file_ = std::fopen(file_path.c_str(), "ab");
    if (daily_log_file_ == nullptr) {
        std::cerr << "严重错误：Logger 无法打开日日志文件：" << file_path << "\n";
        return;
    }

    std::fseek(daily_log_file_, 0, SEEK_END);
    if (std::ftell(daily_log_file_) == 0) {
        std::fwrite(kUtf8Bom, 1, 3, daily_log_file_);
        std::fflush(daily_log_file_);
    }

    current_date_ = target_date;
}

void Logger::openRunLogFile(const QDateTime& start_time)
{
    if (!start_time.isValid()) {
        return;
    }

    if (run_log_file_ != nullptr) {
        std::fclose(run_log_file_);
        run_log_file_ = nullptr;
    }

    const std::string file_path = resolveUniqueRunLogFilePath(log_dir_, start_time);
    run_log_file_ = std::fopen(file_path.c_str(), "wb");
    if (run_log_file_ == nullptr) {
        std::cerr << "严重错误：Logger 无法打开运行日志文件：" << file_path << "\n";
        return;
    }

    std::fwrite(kUtf8Bom, 1, 3, run_log_file_);
    std::fflush(run_log_file_);

    const std::string start_stamp = start_time.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toStdString();
    const std::string header = buildLogLine(
        start_stamp.c_str(),
        "INF",
        "logger",
        ("===== 进程会话开始 | 日志文件: " + file_path + " =====").c_str(),
        nullptr);
    writeLogLineToFile(run_log_file_, header);
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    Logger* logger = instance_;
    if (logger != nullptr) {
        logger->log(type, context, msg);
        return;
    }
    if (previous_handler_ != nullptr) {
        previous_handler_(type, context, msg);
    }
}

void Logger::log(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    const QByteArray msg_utf8 = msg.toUtf8();
    struct DepthGuard {
        DepthGuard() { ++g_log_handler_depth; }
        ~DepthGuard() { --g_log_handler_depth; }
    } depth_guard;

    if (g_log_handler_depth > 1) {
        emitMinimalFallback(type, context, msg_utf8);
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (getSeverityLevel(type) < getSeverityLevel(min_level_)) {
        return;
    }

    if (isMutedHmiLogCategory(safeCategoryName(context))) {
        return;
    }

    const QDateTime now = QDateTime::currentDateTime();
    if (now.date() != current_date_) {
        openDailyLogFile(now.date());
    }

    const std::string time_stamp = now.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toStdString();
    const std::string suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : std::string{};
    const std::string line = buildLogLine(
        time_stamp.c_str(),
        getLogSeverity(type),
        safeCategoryName(context),
        msg_utf8.constData(),
        suffix.empty() ? nullptr : suffix.c_str());

    writeLogLineToFile(daily_log_file_, line);
    writeLogLineToFile(run_log_file_, line);

    writeConsoleLine(type, line.c_str(), line.size());
}

}  // namespace scan_tracking::common
