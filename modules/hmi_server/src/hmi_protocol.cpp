/**
 * @file hmi_protocol.cpp
 * @brief HMI 显控通信协议 JSON 组包辅助函数实现
 */

#include "scan_tracking/hmi_server/hmi_protocol.h"

#include <QtCore/QDateTime>
#include <QtCore/QJsonDocument>
#include <QtEndian>

namespace scan_tracking {
namespace hmi_server {

QJsonObject buildEnvelope(const QString& type, const QString& msgId, const QJsonObject& payload)
{
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = type;
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    return envelope;
}

QJsonObject buildResponsePayload(bool success, const QString& message)
{
    QJsonObject payload;
    payload[QStringLiteral("success")] = success;
    payload[QStringLiteral("message")] = message;
    return payload;
}

QByteArray serializeFrame(const QJsonObject& envelope)
{
    // 将 JSON 对象序列化为紧凑格式的 UTF-8 字节流
    const QByteArray json = QJsonDocument(envelope).toJson(QJsonDocument::Compact);
    const quint32 length = static_cast<quint32>(json.size());

    // 构建帧：4 字节大端长度头 + JSON 正文
    QByteArray frame(4, 0);
    qToBigEndian(length, reinterpret_cast<uchar*>(frame.data()));
    frame.append(json);
    return frame;
}

}  // namespace hmi_server
}  // namespace scan_tracking
