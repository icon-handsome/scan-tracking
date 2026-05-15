import paramiko
import time

JUMP = dict(hostname='frp-put.com', port=46132, username='gddw', password='200015', timeout=20)

TARGETS = [
    dict(name='座舱工控机', ip='192.168.1.10', username='twowin',  password='tw'),
    dict(name='挖机工控机', ip='192.168.1.11', username='ubuntu',  password='ZC@pwd123.'),
]

def connect_target(t, retries=4, wait=20):
    name = t['name']
    for attempt in range(1, retries + 1):
        try:
            print(f'[{name}] 尝试第 {attempt} 次...')
            jump = paramiko.SSHClient()
            jump.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            jump.connect(**JUMP)

            channel = jump.get_transport().open_channel(
                'direct-tcpip', (t['ip'], 22), ('127.0.0.1', 0)
            )

            target = paramiko.SSHClient()
            target.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            target.connect(t['ip'], username=t['username'], password=t['password'],
                           sock=channel, timeout=15)

            cmds = ['whoami', 'hostname', 'uname -a', 'uptime']
            info = {}
            for cmd in cmds:
                stdin, stdout, stderr = target.exec_command(cmd, timeout=10)
                out = stdout.read().decode('utf-8', errors='replace').strip()
                info[cmd] = out

            target.close()
            jump.close()

            print(f'\n✓ {name} ({t["ip"]}) 连接成功')
            print(f'  whoami  : {info.get("whoami", "-")}')
            print(f'  hostname: {info.get("hostname", "-")}')
            print(f'  uname   : {info.get("uname -a", "-")}')
            print(f'  uptime  : {info.get("uptime", "-")}')
            return True

        except Exception as e:
            print(f'  [{name}] 第{attempt}次失败: {e}')
            if attempt < retries:
                print(f'  等待 {wait}s 后重试...')
                time.sleep(wait)

    print(f'\n✗ {name}: 全部重试失败')
    return False

# 先等一段时间让跳板机冷却
print('等待 25s 让跳板机冷却...')
time.sleep(25)

for t in TARGETS:
    connect_target(t)
    print('--- 间隔 25s ---')
    time.sleep(25)

print('\n[ALL DONE]')
