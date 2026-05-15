import paramiko
import time
import threading

JUMP = dict(hostname='frp-put.com', port=46132, username='gddw', password='200015', timeout=20)

TARGETS = [
    dict(name='座舱工控机',   ip='192.168.1.10', username='twowin',  password='tw'),
    dict(name='挖机工控机',   ip='192.168.1.11', username='ubuntu',  password='ZC@pwd123.'),
    dict(name='春天屏',       ip='192.168.1.15', username='linaro',  password='linaro'),
]

results = {}
lock = threading.Lock()

def connect_target(t):
    name = t['name']
    try:
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

        with lock:
            results[name] = {'status': 'OK', 'info': info}
        print(f'[OK] {name} ({t["ip"]}) 连接成功')

    except Exception as e:
        with lock:
            results[name] = {'status': 'FAIL', 'error': str(e)}
        print(f'[FAIL] {name} ({t["ip"]}): {e}')

# 串行连接，每次间隔避免跳板机限速
for t in TARGETS:
    connect_target(t)
    time.sleep(6)

print('\n========== 连接汇总 ==========')
for name, r in results.items():
    if r['status'] == 'OK':
        info = r['info']
        print(f'\n✓ {name}')
        print(f'  whoami  : {info.get("whoami", "-")}')
        print(f'  hostname: {info.get("hostname", "-")}')
        print(f'  uname   : {info.get("uname -a", "-")}')
        print(f'  uptime  : {info.get("uptime", "-")}')
    else:
        print(f'\n✗ {name}: {r["error"]}')
