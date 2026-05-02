#pragma once

#include <QtCore/QString>
#include <QtCore/QtMessageHandler>
#include <QtCore/QDate>
#include <QtCore/QMutex>

class QFile;

namespace scan_tracking::common {

class Logger {
public:
    static void initialize(const QString& log_dir = QStringLiteral("logs"));
    static void cleanup();
    
    // Qt 全局捕莽回调转发接口
    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    // 可以在任何地方动态获取列厗前侵
    static Logger* instance();

    // 动态调整最低输出级别
    void setMinLevel(QtMsgType level);

    // 禁用拷贝于赋值，强制单例约束
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    explicit Logger(const QString& log_dir);
    ~Logger();

    void openLogFile(const QDate& target_date);
    void log(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static QString getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    QString log_dir_;
    QFile* log_file_ = nullptr;
    QMutex mutex_;
    QDate current_date_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
};

} // namespace scan_tracking::common
