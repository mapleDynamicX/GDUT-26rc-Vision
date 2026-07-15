#!/bin/bash
 
echo "正在禁用 ibus-daemon..."
 
# 1. 杀死当前所有 ibus-daemon 进程
pkill -9 ibus-daemon 2>/dev/null
echo "✓ 已终止当前 ibus-daemon 进程"
 
# 2. 移除可执行权限（阻止开机自启）
sudo chmod 000 /usr/bin/ibus-daemon 2>/dev/null
sudo chmod 000 /usr/lib/ibus/ibus-daemon 2>/dev/null
echo "✓ 已移除 ibus-daemon 可执行权限"
 
# 3. 禁用 systemd 用户服务（如果存在）
systemctl --user disable ibus-daemon.service 2>/dev/null
systemctl --user stop ibus-daemon.service 2>/dev/null
echo "✓ 已禁用 ibus 系统服务"

cd ~/RC2026/shell/
python3 run_shell.py

echo "Starting IBus daemon..."

# 1. Restore executable permissions (undo the chmod 000 from disable script)
sudo chmod 755 /usr/bin/ibus-daemon 2>/dev/null
sudo chmod 755 /usr/lib/ibus/ibus-daemon 2>/dev/null
echo "✓ Restored executable permissions for ibus-daemon"

# 2. Re-enable systemd user service (if it exists on your system)
systemctl --user enable ibus-daemon.service 2>/dev/null
systemctl --user start ibus-daemon.service 2>/dev/null
echo "✓ Enabled ibus user service (if available)"

# 3. Start ibus-daemon in background daemon mode
# -d = run as daemon (background)
# -r = replace any existing ibus process
ibus-daemon -d -r 2>/dev/null

# 4. Verify if ibus started successfully
echo ""
if pgrep -x "ibus-daemon" > /dev/null; then
    echo "✅ IBus started successfully"
    echo "   IBus daemon is running in the background"
else
    echo "❌ Failed to start IBus"
    echo "   Please check if ibus is installed correctly"
fi

