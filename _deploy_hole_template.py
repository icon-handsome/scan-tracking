"""Upload Hole template assets from third_party to IPC deploy_debug."""
import os
import sys

import paramiko

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HOST = "192.168.8.14"
USER = "Administrator"
PASSWORD = "123456"
DEPLOY = r"D:\work\LY\deploy_debug"

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
LOCAL_HOLE = os.path.join(REPO_ROOT, "third_party", "Hole")

UPLOADS = [
    (
        os.path.join(LOCAL_HOLE, "template", "Zhu_Mian_Kai_Kong_Template_cut_trans.pcd"),
        rf"{DEPLOY}\hole\config\template\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd",
    ),
    (
        os.path.join(LOCAL_HOLE, "template", "Zhu_Mian_Kai_Kong_Template_cut_trans.pcd"),
        rf"{DEPLOY}\template\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd",
    ),
    (
        os.path.join(LOCAL_HOLE, "config", "default.json"),
        rf"{DEPLOY}\hole\config\default.json",
    ),
]


def run_cmd(client: paramiko.SSHClient, cmd: str) -> str:
    _, stdout, stderr = client.exec_command(f'cmd /c "{cmd}"', timeout=120)
    out = stdout.read().decode("utf-8", errors="replace").strip()
    err = stderr.read().decode("utf-8", errors="replace").strip()
    if err:
        out = (out + "\nERR: " + err).strip()
    return out


def ensure_remote_dir(sftp: paramiko.SFTPClient, remote_dir: str) -> None:
    parts = remote_dir.replace("/", "\\").split("\\")
    current = parts[0] + "\\"
    for part in parts[1:]:
        if not part:
            continue
        current = current.rstrip("\\") + "\\" + part
        try:
            sftp.stat(current)
        except FileNotFoundError:
            sftp.mkdir(current)


def main() -> int:
    print(f"Connecting to {USER}@{HOST} ...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        HOST,
        username=USER,
        password=PASSWORD,
        timeout=20,
        look_for_keys=False,
        allow_agent=False,
    )

    print("\n===== Remote pre-check =====")
    for label, cmd in [
        ("DEPLOY", f'if exist "{DEPLOY}" (echo YES) else (echo NO)'),
        ("EXE", f'if exist "{DEPLOY}\\scan-tracking.exe" (echo YES) else (echo NO)'),
        ("OLD_TEMPLATE_ROOT", f'if exist "{DEPLOY}\\template\\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd" (echo YES) else (echo NO)'),
        ("OLD_TEMPLATE_CFG", f'if exist "{DEPLOY}\\hole\\config\\template\\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd" (echo YES) else (echo NO)'),
    ]:
        print(f"{label}: {run_cmd(client, cmd)}")

    sftp = client.open_sftp()
    print("\n===== Upload =====")
    for local_path, remote_path in UPLOADS:
        if not os.path.isfile(local_path):
            print(f"SKIP missing local: {local_path}")
            continue
        remote_dir = os.path.dirname(remote_path)
        ensure_remote_dir(sftp, remote_dir)
        size_mb = os.path.getsize(local_path) / (1024 * 1024)
        print(f"Uploading {local_path}")
        print(f"       -> {remote_path} ({size_mb:.2f} MB)")
        sftp.put(local_path, remote_path)
        remote_stat = sftp.stat(remote_path)
        print(f"       OK remote size={remote_stat.st_size}")

    sftp.close()

    print("\n===== Remote post-check =====")
    for label, cmd in [
        ("TEMPLATE_ROOT", f'if exist "{DEPLOY}\\template\\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd" (echo YES) else (echo NO)'),
        ("TEMPLATE_CFG", f'if exist "{DEPLOY}\\hole\\config\\template\\Zhu_Mian_Kai_Kong_Template_cut_trans.pcd" (echo YES) else (echo NO)'),
        ("HOLE_CONFIG", f'if exist "{DEPLOY}\\hole\\config\\default.json" (echo YES) else (echo NO)'),
    ]:
        print(f"{label}: {run_cmd(client, cmd)}")

    client.close()
    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
