/**
 * @brief Modbus TCP 通信服务实现
 * 
 * 负责 IPC 与 PLC 之间的 Modbus TCP 通信，提供异步读写、自动重连等功能。
 */

#include "scan_tracking/modbus/modbus_service.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"

#include <QtCore/QLoggingCategory>
#include <QtSerialBus/QModbusDataUnit>
#include <QtSerialBus/QModbusDevice>
#include <QtSerialBus/QModbusReply>
#include <QtSerialBus/QModbusTcpClient>
#include <QtCore/QMutexLocker>

namespace scan_tracking::modbus {

Q_LOGGING_CATEGORY(LOG_MODBUS, "modbus")

/**
 * @brief 构造函数：初始化客户端和重连定时器
 */
ModbusService::ModbusService(QObject* parent)
    : QObject(parent)
    , m_client(new QModbusTcpClient(this))
    , m_reconnectTimer(new QTimer(this))
{
    // 设置为单次触发，避免重连时并发建链
    m_reconnectTimer->setSingleShot(true);

    // 绑定底层信号到内部槽函数
    connect(m_client, &QModbusTcpClient::stateChanged, this, &ModbusService::onStateChanged);
    connect(m_client, &QModbusTcpClient::errorOccurred, this, &ModbusService::onModbusError);
    connect(m_reconnectTimer, &QTimer::timeout, this, &ModbusService::reconnectIfNeeded);
}

/**
 * @brief 析构函数：断开连接并清理资源
 */
ModbusService::~ModbusService()
{
    disconnectDevice();
}

/**
 * @brief 检查是否已连接到 Modbus 服务器
 */
bool ModbusService::isConnected() const
{
    // 仅返回 ConnectedState，屏蔽其他中间状态
    return m_client != nullptr && m_client->state() == QModbusDevice::ConnectedState;
}

/**
 * @brief 发起 Modbus TCP 连接
 */
bool ModbusService::connectDevice()
{
    if (m_client == nullptr) {
        return false;
    }

    // 启用自动重连标志
    m_reconnectEnabled = true;

    // 如果已在连接中或已连接，忽略本次请求
    if (m_client->state() == QModbusDevice::ConnectedState ||
        m_client->state() == QModbusDevice::ConnectingState) {
        qInfo(LOG_MODBUS) << "Connect request ignored because client is already active.";
        return true;
    }

    // 如果处于过渡状态（如正在断开），也忽略
    if (m_client->state() != QModbusDevice::UnconnectedState) {
        qInfo(LOG_MODBUS) << "Connect request ignored because client is transitioning.";
        return false;
    }

    // 从配置管理器读取连接参数
    auto* configManager = common::ConfigManager::instance();
    if (configManager == nullptr) {
        const QString error = QStringLiteral("ConfigManager not loaded");
        qCritical(LOG_MODBUS) << error;
        emit errorOccurred(error);
        return false;
    }

    const auto& conf = configManager->modbusConfig();
    m_unitId = conf.unitId;
    
    // 设置连接参数：IP、端口、超时、重试次数
    m_client->setConnectionParameter(QModbusDevice::NetworkAddressParameter, conf.host);
    m_client->setConnectionParameter(QModbusDevice::NetworkPortParameter, conf.port);
    m_client->setTimeout(conf.timeoutMs);
    m_client->setNumberOfRetries(3);
    m_reconnectTimer->setInterval(conf.reconnectIntervalMs);

    qInfo(LOG_MODBUS).noquote()
        << "Connecting to Modbus server" << conf.host
        << ":" << conf.port
        << "unitId=" << m_unitId;

    // 尝试建立连接，失败则调度重连
    if (!m_client->connectDevice()) {
        const QString error = m_client->errorString();
        qCritical(LOG_MODBUS).noquote() << "Modbus client setup failed:" << error;
        emit errorOccurred(error);
        scheduleReconnect(QStringLiteral("client setup failed"));
        return false;
    }

    return true;
}

/**
 * @brief 断开连接并禁用自动重连
 */
void ModbusService::disconnectDevice()
{
    if (m_client == nullptr) {
        return;
    }

    // 禁用重连，防止断开后再次尝试连接
    m_reconnectEnabled = false;
    m_reconnectTimer->stop();
    qInfo(LOG_MODBUS) << "Disconnecting Modbus client and disabling reconnect.";
    
    // 调用底层断开接口
    m_client->disconnectDevice();
}

/**
 * @brief 处理连接状态变化
 */
void ModbusService::onStateChanged(QModbusDevice::State state)
{
    // 连接成功：停止重连定时器，发出信号
    if (state == QModbusDevice::ConnectedState) {
        m_reconnectTimer->stop();
        qInfo(LOG_MODBUS) << "Modbus connected.";
        emit connected();
        return;
    }

    // 正在连接：仅记录日志
    if (state == QModbusDevice::ConnectingState) {
        qInfo(LOG_MODBUS) << "Modbus connection attempt in progress.";
        return;
    }

    // 未连接：发出断开信号，根据标志决定是否重连
    if (state == QModbusDevice::UnconnectedState) {
        qWarning(LOG_MODBUS) << "Modbus unconnected.";
        emit disconnected();
        if (m_reconnectEnabled) {
            scheduleReconnect(QStringLiteral("connection dropped"));
        } else {
            qInfo(LOG_MODBUS) << "Reconnect skipped because service is stopping.";
        }
    }
}

/**
 * @brief 处理 Modbus 错误事件
 */
void ModbusService::onModbusError(QModbusDevice::Error error)
{
    // 忽略无错误情况
    if (error == QModbusDevice::NoError) {
        return;
    }

    // 获取错误描述并上报
    const QString errorString =
        m_client ? m_client->errorString() : QStringLiteral("Unknown Modbus error");
    qWarning(LOG_MODBUS).noquote() << "Modbus error:" << errorString << "(code" << error << ")";
    emit errorOccurred(errorString);
}

/**
 * @brief 异步读取保持寄存器
 */
bool ModbusService::readRegisters(int startAddress, quint16 numberOfEntries)
{
    QMutexLocker locker(&m_mutex);
    // 检查连接状态
    if (!isConnected()) {
        qWarning(LOG_MODBUS) << "Cannot read registers while disconnected.";
        emit registerReadFailed(startAddress, QStringLiteral("Modbus client is disconnected"));
        return false;
    }

    // 构建读取请求数据单元
    QModbusDataUnit readUnit(
        QModbusDataUnit::HoldingRegisters,
        startAddress,
        numberOfEntries);

    // 发送读请求
    if (auto* reply = m_client->sendReadRequest(readUnit, m_unitId)) {
        // 如果回复已完成（同步情况），直接处理
        if (reply->isFinished()) {
            handleReadReply(reply);
        } else {
            // 否则等待异步完成信号
            connect(reply, &QModbusReply::finished, this, [this, reply]() {
                handleReadReply(reply);
            });
        }
        return true;
    }

    // 请求创建失败，记录错误
    const QString error = m_client->errorString();
    qCritical(LOG_MODBUS).noquote() << "Read request creation failed:" << error;
    emit registerReadFailed(startAddress, error);
    emit errorOccurred(error);
    return false;
}

/**
 * @brief 处理读取操作的回复
 */
void ModbusService::handleReadReply(QModbusReply* reply)
{
    if (reply == nullptr) {
        return;
    }

    // 检查是否有错误
    if (reply->error() == QModbusDevice::NoError) {
        // 成功：提取寄存器值并发出信号
        const auto unit = reply->result();
        QVector<quint16> values;
        values.reserve(static_cast<int>(unit.valueCount()));
        for (uint index = 0; index < unit.valueCount(); ++index) {
            values.push_back(unit.value(index));
        }
        emit registersRead(unit.startAddress(), values);
    } else {
        // 失败：记录错误并发出失败信号
        const QString error = reply->errorString();
        qWarning(LOG_MODBUS).noquote() << "Modbus read failed:" << error;
        emit registerReadFailed(reply->result().startAddress(), error);
        emit errorOccurred(error);
    }

    // 延迟删除回复对象
    reply->deleteLater();
}

/**
 * @brief 写入单个寄存器（便捷方法）
 */
bool ModbusService::writeRegister(int startAddress, quint16 value)
{
    return writeRegisters(startAddress, {value});
}

/**
 * @brief 异步写入多个保持寄存器
 */
bool ModbusService::writeRegisters(int startAddress, const QVector<quint16>& values)
{
    QMutexLocker locker(&m_mutex);
    // 检查连接状态
    if (!isConnected()) {
        qWarning(LOG_MODBUS) << "Skipping write while disconnected.";
        return false;
    }

    // 构建写入数据单元
    QModbusDataUnit writeUnit(
        QModbusDataUnit::HoldingRegisters,
        startAddress,
        values.size());
    for (int index = 0; index < values.size(); ++index) {
        writeUnit.setValue(static_cast<uint>(index), values[index]);
    }

    // 发送写请求
    if (auto* reply = m_client->sendWriteRequest(writeUnit, m_unitId)) {
        if (reply->isFinished()) {
                handleWriteReply(reply, startAddress, values.size());
        } else {
                // 等待异步完成，传递地址和数量用于错误诊断
                connect(reply, &QModbusReply::finished, this, [this, reply, startAddress, values]() {
                    handleWriteReply(reply, startAddress, values.size());
            });
        }
        return true;
    }

    // 请求创建失败
    const QString error = m_client->errorString();
    qCritical(LOG_MODBUS).noquote() << "Write request creation failed:" << error;
    emit errorOccurred(error);
    return false;
}

/**
 * @brief 定时器超时回调：尝试重连
 */
void ModbusService::reconnectIfNeeded()
{
    // 检查重连是否被禁用
    if (!m_reconnectEnabled) {
        qInfo(LOG_MODBUS) << "Reconnect timer fired after shutdown request; skipping.";
        return;
    }

    // 确认客户端存在且处于未连接状态
    if (m_client == nullptr || m_client->state() != QModbusDevice::UnconnectedState) {
        return;
    }

    qInfo(LOG_MODBUS) << "Attempting Modbus reconnect.";
    // 发起重连
    connectDevice();
}

/**
 * @brief 调度延迟重连任务
 */
void ModbusService::scheduleReconnect(const QString& reason)
{
    // 检查重连是否被禁用或客户端不存在
    if (!m_reconnectEnabled || m_client == nullptr) {
        return;
    }

    // 仅在未连接状态下才调度重连
    if (m_client->state() != QModbusDevice::UnconnectedState) {
        return;
    }

    // 如果已有重连任务在排队，避免重复调度
    if (m_reconnectTimer->isActive()) {
        qInfo(LOG_MODBUS).noquote()
            << "Reconnect already scheduled in" << m_reconnectTimer->remainingTime()
            << "ms after" << reason;
        return;
    }

    // 启动重连定时器
    qWarning(LOG_MODBUS).noquote()
        << "Scheduling Modbus reconnect in" << m_reconnectTimer->interval()
        << "ms due to" << reason;
    m_reconnectTimer->start();
}

/**
 * @brief 处理写入操作的回复结果
 */
void ModbusService::handleWriteReply(QModbusReply* reply, int startAddress, int /*registerCount*/)
{
    if (reply == nullptr) {
        return;
    }

    // 检查写入是否失败
    if (reply->error() != QModbusDevice::NoError) {
        const QString error = reply->errorString();
        // 记录详细错误信息：地址、数量、错误码
        const auto result = reply->result();
        qWarning(LOG_MODBUS).noquote()
            << "Modbus write FAILED | offset=" << startAddress
            << "| registerCount=" << result.valueCount()
            << "| errorCode=" << static_cast<int>(reply->error())
            << "| errorDetail:" << error;
        
        // 发出细粒度失败信号（P1修复）
        emit registerWriteFailed(startAddress, error);
        emit errorOccurred(error);
    } else {
        // 成功：仅在 DEBUG 模式下记录日志（P2改进）
#ifdef QT_DEBUG
        const auto result = reply->result();
        qDebug(LOG_MODBUS).noquote()
            << "Modbus write OK | offset=" << startAddress
            << "| registerCount=" << result.valueCount();
#endif
    }

    // 延迟删除回复对象
    reply->deleteLater();
}

}  // namespace scan_tracking::modbus

