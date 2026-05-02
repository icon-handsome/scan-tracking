#pragma once

#include <QtCore/QString>

namespace scan_tracking {
namespace vision {
namespace lanyou {

enum class LanyouIssueCode : int {
    Success = 0,
    InputEmpty = 1001,
    PointCloudInvalid = 1002,
    FileNotFound = 1003,
    ConfigMissing = 1004,
    ParameterError = 1005,
    ResultInvalid = 1006,
    AlgorithmFailure = 1007,
};

inline int issueNumber(LanyouIssueCode code)
{
    return static_cast<int>(code);
}

inline QString issueName(LanyouIssueCode code)
{
    switch (code) {
    case LanyouIssueCode::Success:
        return QStringLiteral("success");
    case LanyouIssueCode::InputEmpty:
        return QStringLiteral("input-empty");
    case LanyouIssueCode::PointCloudInvalid:
        return QStringLiteral("point-cloud-invalid");
    case LanyouIssueCode::FileNotFound:
        return QStringLiteral("file-not-found");
    case LanyouIssueCode::ConfigMissing:
        return QStringLiteral("config-missing");
    case LanyouIssueCode::ParameterError:
        return QStringLiteral("parameter-error");
    case LanyouIssueCode::ResultInvalid:
        return QStringLiteral("result-invalid");
    case LanyouIssueCode::AlgorithmFailure:
        return QStringLiteral("algorithm-failure");
    }
    return QStringLiteral("unknown");
}

inline QString issueTag(LanyouIssueCode code)
{
    return QStringLiteral("[LY-%1-%2]")
        .arg(issueNumber(code))
        .arg(issueName(code));
}

inline QString issueSummary(LanyouIssueCode code)
{
    switch (code) {
    case LanyouIssueCode::Success:
        return QStringLiteral("检测成功");
    case LanyouIssueCode::InputEmpty:
        return QStringLiteral("输入为空");
    case LanyouIssueCode::PointCloudInvalid:
        return QStringLiteral("点云无效");
    case LanyouIssueCode::FileNotFound:
        return QStringLiteral("文件不存在");
    case LanyouIssueCode::ConfigMissing:
        return QStringLiteral("配置缺失");
    case LanyouIssueCode::ParameterError:
        return QStringLiteral("参数错误");
    case LanyouIssueCode::ResultInvalid:
        return QStringLiteral("结果无效");
    case LanyouIssueCode::AlgorithmFailure:
        return QStringLiteral("算法失败");
    }
    return QStringLiteral("未知错误");
}

}  // namespace lanyou
}  // namespace vision
}  // namespace scan_tracking
