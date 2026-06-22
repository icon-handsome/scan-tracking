#pragma once

#include <QtCore/QString>
#include <QtCore/QtMessageHandler>
#include <QtCore/QDate>

#include <cstdio>
#include <mutex>

namespace scan_tracking::common {

// 双路落盘：
// 1) 按自然日：logs/scan_tracking_yyyy-MM-dd.txt（跨日自动切换，仅追加）
// 2) 按启动会话：logs/scan_tracking_run_yyyy-MM-dd_HH-mm-ss.txt（进程启动时创建，至退出关闭）
class Logger {
public:
    static void initialize(const QString& log_dir = QStringLiteral("logs"));
    static void cleanup();

    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static Logger* instance();

    void setMinLevel(QtMsgType level);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    explicit Logger(const QString& log_dir);
    ~Logger();

    void openDailyLogFile(const QDate& target_date);
    void openRunLogFile(const QDateTime& start_time);
    void writeLogLineToFile(FILE* file, const std::string& line);
    void log(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static const char* getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    std::string log_dir_;
    FILE* daily_log_file_ = nullptr;
    FILE* run_log_file_ = nullptr;
    std::mutex mutex_;
    QDate current_date_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
};

}  // namespace scan_tracking::common
