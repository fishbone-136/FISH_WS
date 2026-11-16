# 邦邦 — Unitree Go2 本地智能交互平台

邦邦运行于 Ubuntu 22.04 / ROS 2 Humble，集成语音对话、受控动作、D435i 视觉
引导、手势、Nav2 和 SLAM。语音与视觉功能不再调用 DeepSeek、Kimi、Google ASR
或其他云端 API。

## 界面使用

运行 `scripts/run_ui.sh` 后，主窗口按当前屏幕的可用区域最大化。界面右上角内置
最小化、最大化/还原、关闭按钮；双击顶部可切换最大化，还原后可拖动顶部和右下角
调整窗口。小屏幕上内容区可以滚动。底部提供“首页 / 导航”两个入口；导航页首次打开时
才创建显示桥接线程，并嵌入当前 LIO 导航的完整 RViz 窗口。

导航已经运行时，从工作区根目录执行以下命令，再点击底部“导航”即可接入：

```bash
cd /home/fishbone/FISH_ws
QT_QPA_PLATFORM=xcb bash scripts/run_ui.sh
```

默认“接入已运行的当前导航”模式只连接现有节点，不启动第二套导航。RViz直接读取
当前安装的 `go2_lio_navigation/config/lio_environment.rviz`，包含地图、点云、地形、
代价地图、路径和原有 Nav2 选点工具，导航代码和参数不由 UI 修改。
“二维选点”是辅助页，使用 `/safety/navigation_global` 和 `odom_lio` 坐标；
保存按钮调用当前 `/mapping/save`，保存结果路径显示在日志中。

导航尚未启动时，可选“启动当前 LIO 导航”，再点击“连接 / 启动”。该按钮调用原有
`scripts/run_lio_navigation.sh`，仅设置 `use_rviz:=false`，由界面内的 RViz负责显示，
其他导航配置沿用现有入口；缺少历史地形显示节点时由界面补充显示节点，不改变导航输入。
内嵌窗口使用 X11/xcb；Wayland 桌面可通过以上命令使用 XWayland。

点击语音、手势或视觉功能即可启动，切换功能时会先停止上一项；重复点击当前功能
不会重复启动或关闭。点击「停止当前功能」返回待机。
手势模式点击后直接启用识别，仍保留 OK 手势切换待机的能力。眼动仪尚未接入，
对应入口暂不可用。

语音的麦克风发现和模型加载在后台进行；Whisper 就绪后即可监听，不再等待 Qwen、
Kokoro 和运动接口全部启动。Qwen 同时后台预热；运动接口在实际收到动作口令时连接。
第一次问答仍可能需要等待模型加载，日志会记录启动到监听的实际耗时。

运行日志保存在 `log/ui_runtime.log`（自动轮转），包含界面交互、录音长度、主模型
识别、复核加载和复核解码耗时。背景音或低置信度音频被过滤后，界面恢复监听状态，
不会保留“正在识别”的假回复。校准门限以实际环境底噪为准，配置的参考上限不会把
门限压到实际底噪以下；只有 ASR 判定为静音的长时间稳定噪声才用于上调门限。

点击「视觉感知」读取深度相机当前彩色画面，简洁描述物体、人物和位置；只有主动询问
文字或标牌内容时才读字，不再自动追加文字识别段落。彩色和深度预览使用 640×360、30 fps，
按新帧刷新，不重复处理相同帧。
在「交互日志」输入框输入追问，再点击发送或按 Ctrl+Enter；每轮读取最新画面并保留
最近四轮文字上下文。「重新识别当前画面」开始新的场景描述。相机断开、帧过期或模型
不可用会显示错误，不会用 Go2 前置相机或旧画面替代 D435i。

## 本地交互链路

```text
麦克风
  → faster-whisper（独立进程，中英文自动识别）
  → 固定安全口令门控
      ├─ 普通聊天 → 本机 Ollama / qwen2.5:3b
      ├─ 授权动作 → Qwen 语义分析 → 关键词/JSON → 白名单/限速/限时
      ├─ 动作 → Unitree SDK2 服务探测/API 回执 → SportClient.Move
      ├─ 视觉引导 → D435i RGB-D → 引导线/红绿灯/近障 → 短动作闭环
      └─ 一键报警 → 立即停车 + 本地记录 + ROS 2/现场报警程序
```

只有以下中英文句式能够授权物理动作：

- `执行指令，向前走一米`
- `执行命令，向前走两步`
- `执行左转九十度动作`
- `执行动作，沿地面引导线前进并识别红绿灯`
- `execute command, move forward one meter`
- `execute a left turn action`

裸说“前进”“go forward”等只会得到安全提示，不会移动。`停止 / 急停 / stop /
emergency stop` 属于安全制动，可随时直接说；`一键报警 / 紧急报警 / call for help /
raise the alarm` 会先停车再触发报警链路。

## D435i 视觉引导

语音中明确使用执行口令并要求“沿地面引导线”“识别红绿灯”时，Qwen 只生成
`visual_guide` 受限动作，实际闭环由本地 OpenCV 和 D435i 完成：

- 白色/黄色地面引导线中心偏差控制左右修正；
- 红灯停车、绿灯沿线通行；
- 深度图中央 15 分位距离小于默认 0.60 m 时停车；
- 丢线、连续无帧、异常或达到 120 秒上限时停车；
- 引导在线程中运行，麦克风仍可接收急停和报警。

这是实验性辅助功能，不能替代 Unitree 遥控器急停，也不能作为道路交通或生命安全
系统。首次实机测试必须架空或置于宽阔无人场地。

## 配置

先保证本机 Ollama 已运行并存在 Qwen 模型：

```bash
ollama serve
ollama pull qwen2.5:3b
ollama list
```

复制配置并按实际网卡修改：

```bash
cp go2_jiaohu/.env.example go2_jiaohu/.env
```

关键配置：

```dotenv
GO2_NETWORK_INTERFACE=eth0
GO2_ROBOT_NAME=邦邦
GO2_VOICE_INPUT_DEVICE=pulse
# 按 `pactl list short sources/sinks` 的实际名称填写；下面以 XVF3800 为例：
GO2_AUDIO_ALSA_CARD=Array
GO2_AUDIO_PULSE_CARD=alsa_card.usb-Seeed_Studio_reSpeaker_XVF3800_4-Mic_Array_...-00
GO2_AUDIO_PULSE_PROFILE=output:analog-stereo+input:analog-stereo
GO2_AUDIO_AGC=off
GO2_AUDIO_MIC_GAIN=80%
GO2_AUDIO_PLAYBACK_CONTROL=PCM,1  # XVF3800 的独立硬件播放级；其他设备可留空
GO2_AUDIO_PLAYBACK_VOLUME=85%
GO2_AUDIO_OUTPUT_VOLUME=125%
PULSE_SOURCE=alsa_input.usb-Seeed_Studio_reSpeaker_XVF3800_4-Mic_Array_...-00.analog-stereo
PULSE_SINK=alsa_output.usb-Seeed_Studio_reSpeaker_XVF3800_4-Mic_Array_...-00.analog-stereo
OLLAMA_HOST=http://127.0.0.1:11434
OLLAMA_MODEL=qwen2.5:3b
OLLAMA_TIMEOUT=30
OLLAMA_CONTEXT_LENGTH=2048
OLLAMA_CHAT_MAX_PREDICT=64
OLLAMA_HISTORY_MESSAGES=4
OLLAMA_KEEP_ALIVE=30m
OLLAMA_WARMUP=1
OLLAMA_WARMUP_DEPENDENCY_TIMEOUT=30
WHISPER_MODEL=voice/models/faster-whisper-small
WHISPER_CPU_FALLBACK_MODEL=voice/models/faster-whisper-small
WHISPER_DEVICE=cuda
WHISPER_COMPUTE_TYPE=float16
WHISPER_LOCAL_FILES_ONLY=1
WHISPER_MIN_AVG_LOGPROB=-0.65
WHISPER_MAX_NO_SPEECH_PROB=0.70
WHISPER_SUSPICIOUS_MIN_AVG_LOGPROB=-0.35
WHISPER_BEAM_SIZE=2
WHISPER_BEST_OF=2
WHISPER_CHUNK_LENGTH=10
WHISPER_MAX_NEW_TOKENS=96
WHISPER_REPETITION_PENALTY=1.15
WHISPER_NO_REPEAT_NGRAM_SIZE=3
WHISPER_HOTWORDS=""
WHISPER_RESCUE_MODEL=voice/models/faster-whisper-large-v3-turbo
WHISPER_PRELOAD_RESCUE=0
WHISPER_RESCUE_BEAM_SIZE=1
WHISPER_RESCUE_BEST_OF=1
WHISPER_RESCUE_MIN_AVG_LOGPROB=-0.45
WHISPER_RESCUE_MIN_LANGUAGE_PROB=0.45
WHISPER_RESCUE_MAX_NO_SPEECH_PROB=0.45
WHISPER_COMMAND_RESCUE_MIN_AVG_LOGPROB=-0.35
WHISPER_COMMAND_RESCUE_MAX_NO_SPEECH_PROB=0.25
WHISPER_COMMAND_RESCUE_MIN_LANGUAGE_PROB=0.60
GO2_MIC_MIN_ENERGY=200
GO2_MIC_MAX_ENERGY=450
GO2_MIC_PHRASE_THRESHOLD=0.45
GO2_MIC_PAUSE_THRESHOLD=0.45
GO2_MIC_NON_SPEAKING_DURATION=0.25
GO2_TTS_MODEL_DIR=voice/models/kokoro-pytorch
GO2_TTS_VOICE=zm_yunxi
GO2_TTS_SPEED=1.04
GO2_TTS_VOLUME=3.0
GO2_TTS_ECHO_COOLDOWN_SECONDS=0.25
GO2_OBSTACLE_STOP_DISTANCE_M=0.60
GO2_MOTION_BACKEND=sdk
GO2_SDK_RPC_TIMEOUT=2.0
GO2_SDK_VERIFY_SERVER=1
GO2_SDK_MOVE_INTERVAL=0.05
GO2_INTERACTIVE_COMMAND_TOPIC=/go2/interactive_command
GO2_INTERACTIVE_STATUS_TOPIC=/go2/interactive_status
GO2_COMMAND_ACK_TIMEOUT=2.0
GO2_STEP_LENGTH_M=0.40
BANGBANG_ALARM_ROS_TOPIC=/bangbang/emergency_alert
BANGBANG_ALARM_COMMAND=
```

当前部署从 `voice/models/faster-whisper-small` 读取低延迟本地模型，并启用
`WHISPER_LOCAL_FILES_ONLY=1`，启动时不会访问 Hugging Face，也不受系统代理影响。
Jetson AGX Orin 使用工作区 `third_party` 中的 CUDA 12.6 运行库和按 sm_87 编译的
CTranslate2 4.8.2，正常启动日志应显示 `cuda / float16`。模型只在开启语音交互时
装载一次；首次进入就绪状态需要等待权重载入，界面会明确显示“正在加载语音模型”。
若 CUDA 初始化失败，系统会自动切换到本地 `small` 模型的 CPU 可用计算类型。
重新部署时可用下面的命令下载到相同目录：

```bash
env -u ALL_PROXY -u all_proxy hf download Systran/faster-whisper-small \
  --local-dir go2_jiaohu/voice/models/faster-whisper-small
```

`small` 配合双束解码作为低延迟 GPU 主模型，本次现场日志为约 1.9–2.0 秒。
清晰的固定动作口令只解码一次；只有声学置信度、静音概率或语言概率处于边界时，
才懒加载更快且更小的 `large-v3-turbo` 单束复核。这样不再让每条动作口令固定承担
两遍 Whisper 的约 6–10 秒耗时。CUDA 不可用时继续使用本地 `small` CPU 模型。
CTranslate2/Whisper 被隔离到独立进程，避免和 ROS 2 DDS 动态库冲突。

进入语音模式时会等待 Kokoro 完成 GPU 权重分配，再用 1 token 最终预热本地 Qwen，
避免日志中“预热后首轮仍重新加载 4.6 秒”的问题，并保持模型 30 分钟。授权动作始终
先由 Qwen 分析；随后确定性提取前进、后退、平移、转向、步数等常用关键词并转换为
SDK 参数，复杂表达采用 Qwen 的 JSON 动作。两种结果都必须经过同一白名单、限速和
限时校验。每次 Ollama 调用都会记录模型加载、prompt 和生成阶段耗时。

语音播报使用本地 Kokoro `zm_yunxi` 男声。PyTorch、cuDNN 和模型权重均在工作区
隔离目录中运行，正常日志会显示 `本地 Kokoro CUDA 温柔男声已就绪`；模型在后台
预热，不阻塞界面。中英文共用柔和男声音色，提问语言由硬语言锁确定：中文问题只
生成中文回复，英文问题只生成英文回复。Kokoro 不可用时才回退到系统 TTS。

`scripts/run_ui.sh` 会在启动前重建 UAC 双工 profile 并重置 PulseAudio 输入源，
修复 Jetson 上 USB 麦克风显示为运行中、但采集数据全零的情况。XVF3800 默认关闭 AGC，并将
麦克风使用兼顾灵敏度与削波余量的 80%，静态触发门限为 200，并要求至少
0.45 秒连续语音；输出使用 160% 系统增益、85% 独立硬件播放级与
Kokoro 软限幅响度增强。邦邦播报期间会临时静音麦克风并在恢复
监听前清空缓冲，防止扬声器回声触发自问自答。`字幕by索兰娅` 等已知 Whisper
片尾幻觉会被精确裁掉而保留前面的真实口令；解码阶段启用重复惩罚和三元组去重，
并在安全门前拒绝热词提示回显与短语循环。运动热词提示默认关闭，避免底噪被转写成
整套动作词。繁体控制字形和句首“包包/帮帮”等名称误识别也会在门控前归一化。

实机动作会先等待隔离的 SDK2 worker 返回首个 API 结果，再开始 TTS 播报；不再等完整
回复播完后才下发。worker 启动时通过 `GetServerApiVersion()` 验证 Go2 Sport 服务，
运动期间以 20 Hz 续发 `Move`，租约结束发送零速度和 `StopMove`。网卡错误、服务未
匹配或 API 返回非零时界面会明确拒绝，不再把“写入管道”当作成功。SDK 模式没有
ROS 控制权仲裁，因此测试前必须停止 Nav2、SLAM 中的底盘输出和其他 SportClient。

视觉描述默认使用 `qwen3-vl:2b-instruct`（需 Ollama 0.12.7 或更新版本），安装命令为
`ollama pull qwen3-vl:2b-instruct`。可通过 `OLLAMA_VISION_MODEL` 指定其他本地 Qwen
视觉模型。`qwen2.5:3b` 仅用于文本对话；语音视觉引导仍使用独立的本地 OpenCV 闭环。

## 报警接入

一键报警始终执行三件事：停止 Go2、追加
`log/bangbang_emergency_alerts.jsonl`、尝试发布 ROS 2 话题
`/bangbang/emergency_alert`（`std_msgs/msg/String`）。如需连接声光、短信或值班平台，
将现场程序的绝对路径及参数写入 `BANGBANG_ALARM_COMMAND`。系统以参数数组运行它，
并通过 `BANGBANG_ALARM_EVENT` 环境变量传入报警 JSON，不经过 shell。

在没有配置 ROS 2 订阅者或现场程序时，界面会明确提示“只记录了本地报警”，不会谎报
消息已经送达外部人员。

## 安装与运行

从工作区根目录执行：

```bash
bash scripts/install_dependencies.sh
bash scripts/build.sh
bash scripts/check_system.sh
bash scripts/run_ui.sh
```

默认 SDK 模式只要求 Go2 位于配置网卡且 Sport 服务可达，不要求 `go2_base`。实机语音
移动前应退出 Nav2、建图及其他底盘控制程序；如明确设置 `GO2_MOTION_BACKEND=ros`，
才需要启动带 `/go2/interactive_command` 入口的 `go2_base`。

`run_ui.sh` 使用进程锁保证只启动一个 UI。首页的语音、手势、眼动和视觉模式采用
独占生命周期：选择新模式时先停止旧模式，再启动新模式；重复点击不会创建第二个
线程。切换页面不取消目标或停止导航；关闭 UI 仅停止 UI 自己启动的导航与显示进程，
不会停止外部已运行的导航。导航页的“关闭导航”是明确的完整关闭操作，会取消目标，
退出同一 ROS 域内本机的导航启动器、子进程、速度安全节点和底盘控制节点，包括本页接入的已有导航。
关闭过程中禁止启动语音 / 手势；确认进程全部退出后显示“导航已完全关闭”，即可回首页开启交互控制。
关闭后再次进入导航页不会自动重启，需手动点击“启动 / 重试”；重新导航需要初始化定位和地图。
“取消目标”和“急停”是明确的控制操作，包含由内嵌 RViz 发布的目标。
导航正在运行时不允许同时启动首页的语音/手势运动控制，视觉描述等只读功能仍可使用；
先点击“关闭导航”并等待关闭完成后，即可使用原有交互控制。

视觉对话支持明确的目标移动请求，例如“走到黑色椅子前一米”。移动使用 D435i 对齐深度、实测相机安装标定和 Nav2 避障导航；普通画面问题仍只回答，不执行动作。输入“停止”或点击停止按钮取消本次视觉移动。实机使用前需配置相机 TF 或实测安装参数，详见工作区根目录 [视觉对话目标导航说明](../视觉对话目标导航说明_20261111.md)。

独立入口：

```bash
# 接收另一台机器发来的逐行 ASR 文本，端口 9999
cd go2_jiaohu && python -m voice.main

# D435i 本地视觉/语音文本调试；/cam 分析画面
cd go2_jiaohu && python -m vlm.main

# 无实机 UI 测试
cd go2_jiaohu && python offline_test/main.py
```

## 主要模块

- `go2_ui/activity_manager.py`：UI 功能单实例、先停后启的统一生命周期管理。
- `offline_test/speech_controller.py`：本地 ASR、Qwen 对话、安全入口。
- `deploy/speech_controller.py`：实机动作队列、急停和视觉引导接入。
- `voice/command_safety.py`：中英文固定口令、动作白名单和物理参数限幅。
- `voice/visual_guidance.py`：引导线、红绿灯与 D435i 深度近障检测。
- `voice/emergency_alarm.py`：本地审计、ROS 2 报警和现场程序适配。
- `voice/ollama_client.py`：只允许回环地址的本机 Ollama 客户端，含预热和分阶段计时。
- `voice/asr_worker.py`：中英文 faster-whisper 独立推理进程。
- `voice/sdk_worker.py`：Unitree SDK2 服务探测、动作执行和回执的隔离进程。
- `voice/ros_command_worker.py`：可选 ROS 兼容后端的动作与 ACK 隔离进程。
- `voice/tts_worker.py`：Jetson CUDA Kokoro 中英文男声独立推理进程。

完整 ROS 2、建图和导航步骤见工作区根目录 `Go2_Humble_使用教程.md`。
