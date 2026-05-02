#include "scan_tracking/common/logger.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QMutexLocker>
#include <QtCore/QTextStream>
#include <iostream>

namespace scan_tracking::common {

Logger* Logger::instance_ = nullptr;
QtMessageHandler Logger::previous_handler_ = nullptr;

Logger::Logger(const QString& log_dir)
    : log_dir_(log_dir),
      log_file_(new QFile()),
      min_level_(QtDebugMsg) {
    QDir dir(log_dir_);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    openLogFile(QDate::currentDate());
}

Logger::~Logger() {
    if (log_file_) {
        if (log_file_->isOpen()) {
            log_file_->close();
        }
        delete log_file_;
        log_file_ = nullptr;
    }
}

void Logger::initialize(const QString& log_dir) {
    if (!instance_) {
        instance_ = new Logger(log_dir);
        previous_handler_ = qInstallMessageHandler(Logger::messageHandler);
    }
}

void Logger::cleanup() {
    if (instance_) {
        qInstallMessageHandler(previous_handler_);
        delete instance_;
        instance_ = nullptr;
    }
}

Logger* Logger::instance() {
    return instance_;
}

void Logger::setMinLevel(QtMsgType level) {
    QMutexLocker locker(&mutex_);
    min_level_ = level;
}

int Logger::getSeverityLevel(QtMsgType type) {
    switch (type) {
        case QtDebugMsg:    return 0;
        case QtInfoMsg:     return 1;
        case QtWarningMsg:  return 2;
        case QtCriticalMsg: return 3;
        case QtFatalMsg:    return 4;
        default:            return 0;
    }
}

QString Logger::getLogSeverity(QtMsgType type) {
    switch (type) {
        case QtDebugMsg:    return QStringLiteral("DBG");
        case QtInfoMsg:     return QStringLiteral("INF");
        case QtWarningMsg:  return QStringLiteral("WRN");
        case QtCriticalMsg: return QStringLiteral("CRT");
        case QtFatalMsg:    return QStringLiteral("FTL");
        default:            return QStringLiteral("UNK");
    }
}

void Logger::openLogFile(const QDate& target_date) {
    if (log_file_->isOpen()) {
        log_file_->close();
    }

    current_date_ = target_date;
    QString date_str = current_date_.toString("yyyy-MM-dd");
    QString file_path = QDir(log_dir_).filePath(QString("scan_tracking_%1.log").arg(date_str));
    
    log_file_->setFileName(file_path);
    if (!log_file_->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        std::cerr << "CRITICAL: Logger failed to open target file: " << file_path.toStdString() << "\n";
    }
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    if (instance_) {
        instance_->log(type, context, msg);
    } else if (previous_handler_) {
        previous_handler_(type, context, msg);
    }
}

void Logger::log(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    if (getSeverityLevel(type) < getSeverityLevel(min_level_)) {
        return;
    }

    QMutexLocker locker(&mutex_);

    QDateTime now = QDateTime::currentDateTime();
    
    if (now.date() != current_date_) {
        openLogFile(now.date());
    }

    QString time_stamp = now.toString("yyyy-MM-dd HH:mm:ss.zzz");
    QString severity = getLogSeverity(type);
    QString category = QString::fromLatin1(context.category ? context.category : "default");
    
    QString formatted_message = QStringLiteral("[%1] [%2] [%3] %4").arg(time_stamp, severity, category, msg);

    if (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) {
        if (context.file && context.line > 0) {
            formatted_message.append(QStringLiteral(" (%1:%2)").arg(QString::fromLatin1(context.file)).arg(context.line));
        }
    }

    if (log_file_ && log_file_->isOpen()) {
        QTextStream stream(log_file_);
        stream << formatted_message << "\n";
        stream.flush();
    }

    if (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) {
        std::cerr << formatted_message.toStdString() << std::endl;
    } else {
        std::cout << formatted_message.toStdString() << std::endl;
    }
}

} // namespace scan_tracking::common
