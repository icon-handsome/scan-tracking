#pragma once

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QString>
#include <QtSerialBus/QModbusTcpClient>
#include <QtCore/QMutex>

class QModbusReply;

namespace scan_tracking {
namespace modbus {

/**
 * @brief 工业级 Modbus 通信基础模块
 *
 * 职责：
 * 1. 负责 IPC(Client) 与 PLC(Server) 的 Modbus TCP 连接管理
 * 2. 处理通信异常（如掉线、重试机制设防等）
 * 3. 提供读、写 Holding Registers 的核心异步与信号机制
 *
 * 与外部集成：
 * 采用异步回调（signal/slot）供流控中心或业务模块使用。
 */
class ModbusService : public QObject {
    Q_OBJECT
public:
    explicit ModbusService(QObject* parent = nullptr);
    ~ModbusService() override;

    /**
     * @brief 使用系统配置发起设备连接
     * 会自动读取 ConfigManager 中加载好的 [Modbus] host 和 port。
     */
    bool connectDevice();

    /**
     * @brief 断开与设备的连接
     */
    void disconnectDevice();

    /**
     * @brief 检查设备当前连接状态
     */
    bool isConnected() const;

    /**
     * @brief 异步读取保持寄存器 (Holding Registers)
     * @param startAddress PLC侧 Modbus 地址偏移（不需要 +40001，直接传 0 ~ N）
     * @param numberOfEntries 要读取的寄存器个数
     */
    bool readRegisters(int startAddress, quint16 numberOfEntries);

    /**
     * @brief 异步写入单个寄存器
     */
    bool writeRegister(int startAddress, quint16 value);

    /**
     * @brief 异步写入多个寄存器
     */
    bool writeRegisters(int startAddress, const QVector<quint16>& values);

signals:
    /// 当指定起始地址的一批寄存器数据更新时被发出
    void registersRead(int startAddress, QVector<quint16> values);

    /// 当指定起始地址的寄存器读取失败时发出
    void registerReadFailed(int startAddress, const QString& errorString);
    
    /// P1修复：当指定起始地址的寄存器写入失败时发出
    void registerWriteFailed(int startAddress, const QString& errorString);
    
    /// 当通信或连接发生错误时发出
    void errorOccurred(const QString& errorString);
    
    /// 当成功连上 PLC
    void connected();
    
    /// 当网络意外或者主动断开
    void disconnected();

private slots:
    void onStateChanged(QModbusDevice::State state);
    void onModbusError(QModbusDevice::Error error);
    void reconnectIfNeeded();

private:
    void scheduleReconnect(const QString& reason);
    void handleReadReply(QModbusReply* reply);
        // P2改进：增加registerCount参数用于更详细的错误诊断
        void handleWriteReply(QModbusReply* reply, int startAddress, int registerCount = 0);
    

    QModbusTcpClient* m_client = nullptr;
    QTimer* m_reconnectTimer = nullptr;
    int m_unitId = 1;
    bool m_reconnectEnabled = true;
    QMutex m_mutex;
};

} // namespace modbus
} // namespace scan_tracking

