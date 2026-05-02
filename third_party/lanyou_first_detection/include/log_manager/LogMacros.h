#ifndef LOG_MANAGER_LOGMACROS_H
#define LOG_MANAGER_LOGMACROS_H

#include <sstream>
#include "log_manager/LogManager.h"

#define LOG_INFO(msg_expr) \
    do { \
        std::ostringstream _log_oss; \
        _log_oss << msg_expr; \
        log_manager::LogManager::Instance().Log(log_manager::LogLevel::INFO, __FILE__, __LINE__, _log_oss.str()); \
    } while (0)

#define LOG_WARN(msg_expr) \
    do { \
        std::ostringstream _log_oss; \
        _log_oss << msg_expr; \
        log_manager::LogManager::Instance().Log(log_manager::LogLevel::WARN, __FILE__, __LINE__, _log_oss.str()); \
    } while (0)

#define LOG_ERROR(msg_expr) \
    do { \
        std::ostringstream _log_oss; \
        _log_oss << msg_expr; \
        log_manager::LogManager::Instance().Log(log_manager::LogLevel::ERROR, __FILE__, __LINE__, _log_oss.str()); \
    } while (0)

#endif // LOG_MANAGER_LOGMACROS_H
