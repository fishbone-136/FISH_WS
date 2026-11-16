# TigerVNC 与 Wi-Fi

连接地址：`fishbone-wifi.local::5900`。当前服务共享本机 X11 桌面，使用 TLSVnc。

## 2026-11-02 检查与修复

- 原 `99-remote-wifi-powersave.conf` 被排序靠后的 Ubuntu 配置
  `default-wifi-powersave-on.conf` 覆盖，NetworkManager 合并结果为
  `wifi.powersave=3`，网卡实际为 `Power save: on`。
- `stabilize-wifi.sh` 改用 `zz-remote-wifi-powersave.conf`，明确给全部
  8 个已保存的无线配置禁用省电，并立即关闭网卡省电，无需重连。
- `wifi-powersave-dispatcher.sh` 安装到 NetworkManager 的 dispatcher.d，
  在无线连接激活或 DHCP 更新后再次关闭省电。
- 原配置备份：`/var/backups/tigervnc-wifi.8hH1n7qQ/`，仅 root 可读。

已验证：合并配置为 `wifi.powersave=2`、8 个无线配置为 `disable`、
实际网卡为 `Power save: off`；VNC、Avahi、NetworkManager 均正常运行。
通过域名完成了 TLSVnc 密码认证和 1440×900 桌面初始化，现有用户连接保持。
初次检查时未主动切换 Wi-Fi；后续换网验证结果见下文。

## 同一 Wi-Fi 的远程实测

2026-11-02，主机连接 `hw123`，无线地址为 `192.168.3.79`。
远程电脑 `192.168.3.82` 成功建立 TLSVnc 连接；服务端确认完成认证、
进入桌面并持续传输画面，检查时已发送约 19 MB 数据。
用户确认从远程电脑输入 `fishbone-wifi.local::5900`、密码 `123`
也能看到桌面，同一 Wi-Fi 的域名连接测试通过。
这次测试未更换 Wi-Fi，也未测试重启后的首次系统登录。
按用户要求保留系统密码登录，未启用自动登录。

## 更换 Wi-Fi 后的远程实测

2026-11-02，用户将主机从 `hw123` 切换至 `Xiaomi_CE3B`。
服务在 10:47:43 撤回旧域名记录，10:47:47 将
`fishbone-wifi.local` 更新到新的无线地址 `192.168.31.110`。
远程电脑 `192.168.31.96` 于 10:49:17 重新连接，10:49:19 完成认证、
进入桌面；检查时新连接已经传输了约 15 MB 数据。
网卡保持 `Power save: off`。用户反馈换网后 TigerVNC 仍可连接。
此结果验证了这两个 Wi-Fi 之间的换网重连，不代表任意网络都允许 mDNS
和设备互访。

`console.py` 每两秒检查无线 IPv4 地址，通过 Avahi 更新
`fishbone-wifi.local`，不会写死原来的 IP。换网后等待无线连接和 DHCP
完成，再用原地址重连；原 TCP 连接无法在 IP 变化后保持。
客户端与主机需要处于可互访且允许 mDNS 的局域网。

日志中 2026-11-02 09:34:58 的断开来自 GDM 登录界面切换至用户桌面，
服务随后切换到新的 X11 会话。这种切换需要客户端重连。

重用优化脚本：`sudo bash scripts/tigervnc/stabilize-wifi.sh`。
检查实际状态：`iw dev wlP1p1s0 get power_save`。
