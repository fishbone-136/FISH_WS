# 模型、环境与本地数据

GitHub 仓库只保存项目源码、配置模板、测试和构建脚本。模型、已安装环境、编译产物、地图与实机测试数据均由 `.gitignore` 排除；本机已有文件不受影响。

## 未上传的资源

以下为首次整理时的本机占用，使用 GiB/MiB（1024 进制）；不同版本下载后大小可能不同。

| 本地路径 | 约占用 | 用途及恢复方式 |
| --- | ---: | --- |
| `go2_jiaohu/voice/models/faster-whisper-small/` | 464 MiB | 主语音识别模型，也是 CUDA 不可用时的 CPU 回退模型 |
| `go2_jiaohu/voice/models/faster-whisper-large-v3-turbo/` | 1.51 GiB | GPU 上按需进行低置信度语音复核 |
| `go2_jiaohu/voice/models/faster-whisper-large-v3/` | 2.88 GiB | 旧部署模型，当前默认配置不使用，无需为默认部署下载 |
| `go2_jiaohu/voice/models/kokoro-pytorch/` | 313 MiB | 当前 Kokoro 语音合成模型，包含权重、配置和音色 |
| `go2_jiaohu/voice/models/kokoro-multi-lang-v1_0/` | 384 MiB | 备用 ONNX 合成模型，当前 PyTorch 合成链不依赖它 |
| `go2_jiaohu/voice/models/kokoro-int8-multi-lang-v1_1/` | 207 MiB | 备用量化 ONNX 合成模型 |
| `go2_jiaohu/voice/kws-model/` | 34 MiB | 旧关键词模型目录；如启用对应功能，另行放置匹配的 ONNX、tokens 和关键词文件 |
| `ros2_ws/src/go2_navigation/go2_navigation/yolov8n.pt` | 6.3 MiB | 本地人物检测权重 |
| `ros2_ws/src/go2_navigation/go2_navigation/yolov8n.onnx` | 12.3 MiB | YOLO 的 ONNX 导出版本，按需提供 |
| `go2_jiaohu/third_party/cuda-sysroot/` | 2.34 GiB | 原 Jetson 部署的 CUDA 12.6/cuDNN 运行库，须匹配设备和 JetPack |
| `go2_jiaohu/third_party/kokoro-python/` | 932 MiB | 隔离的 Kokoro/PyTorch 及音频依赖 |
| `go2_jiaohu/third_party/ctranslate2-cuda/` | 29 MiB | 原 Orin sm_87 的 CTranslate2 编译库和 wheel |
| `go2_jiaohu/third_party/person_detector_deps/` | 80 MiB | 独立人物检测进程的 Ultralytics/OpenCV 依赖 |
| `.venv/` | 403 MiB | Python 环境，用安装脚本重新创建 |
| `third_party/unitree_sdk2_python/` | 约 3 MiB | Unitree SDK，安装脚本会从上游下载 |
| `third_party/livox_sdk2_build/`、`third_party/livox_sdk2_install/` | 约 7 MiB | 从已收录的 Livox SDK 源码重新构建 |
| `ros2_ws/build*/`、`ros2_ws/install*/`、`ros2_ws/log*/` | 随构建变化 | colcon 生成，不应从旧设备复制使用 |
| `maps/`、`reports/`、`backups/` | 随运行变化 | 地图、现场采样、历史测试记录及备份，单独保存 |
| `Go2_Humble_Integrated/`、`1/`、压缩包 | 历史副本 | 不属于当前维护的源码入口 |

## 配置本机路径

从仓库根目录执行：

```bash
cp go2_jiaohu/.env.example go2_jiaohu/.env
```

模型路径相对 `go2_jiaohu/` 解析，建议沿用模板：

```dotenv
WHISPER_MODEL=voice/models/faster-whisper-small
WHISPER_CPU_FALLBACK_MODEL=voice/models/faster-whisper-small
WHISPER_RESCUE_MODEL=voice/models/faster-whisper-large-v3-turbo
WHISPER_LOCAL_FILES_ONLY=1
WHISPER_PRELOAD_RESCUE=0
GO2_TTS_MODEL_DIR=voice/models/kokoro-pytorch
GO2_TTS_VOICE=zm_yunxi
```

原部署 `.env` 可能包含 `/home/fishbone/go2_ws/...` 等旧绝对路径，已排除上传。新部署应使用模板并调整网卡、音频设备、`LIVOX_WS` 等本机设置。`WHISPER_DEVICE=cuda` 需要可用的 CUDA 构建；仅使用 CPU 时可设置 `WHISPER_DEVICE=cpu` 和 `WHISPER_COMPUTE_TYPE=int8`，此时不执行 GPU 复核。

## 下载语音模型

在已经安装 `huggingface-hub` 的环境中，从仓库根目录执行：

```bash
hf download Systran/faster-whisper-small \
  --local-dir go2_jiaohu/voice/models/faster-whisper-small
hf download dropbox-dash/faster-whisper-large-v3-turbo \
  --local-dir go2_jiaohu/voice/models/faster-whisper-large-v3-turbo
hf download hexgrad/Kokoro-82M \
  --include config.json kokoro-v1_0.pth voices/zm_yunxi.pt \
  --local-dir go2_jiaohu/voice/models/kokoro-pytorch
```

来源：

- [Whisper small](https://huggingface.co/Systran/faster-whisper-small)
- [Whisper large-v3-turbo](https://huggingface.co/dropbox-dash/faster-whisper-large-v3-turbo)（原 mobiuslabsgmbh 仓库已重定向到此地址）
- [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M)

Whisper 目录需要 `model.bin`、配置和 tokenizer 等配套文件，不能只复制权重。Kokoro 需要 `config.json`、`kokoro-v1_0.pth` 和 `voices/zm_yunxi.pt`；对应的 Python 库、中文/英文 G2P 资源也需另行安装。模型的下载和使用遵循各上游许可。

## 人物检测与其他模型

从 [Ultralytics 官方模型发布](https://github.com/ultralytics/assets/releases) 获取匹配的 `yolov8n.pt`，放到 `ros2_ws/src/go2_navigation/go2_navigation/`，或在 `.env` 中设置 `GO2_PERSON_DETECTOR_MODEL` 为本机绝对路径。人物检测进程使用 CUDA PyTorch 和隔离安装的 Ultralytics；它不会自动下载权重。ONNX 导出件按对应功能需要另行生成，不包含在 Git 中。

Ollama 的模型存储在 Ollama 自己的数据目录，也不属于本仓库。安装 Ollama 后，根据配置下载本地模型：

```bash
ollama pull qwen2.5:3b
ollama pull qwen3-vl:2b-instruct
```

## 环境与编译

目标平台为 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10；原 GPU 部署为 Jetson AGX Orin。保留了 `go2_jiaohu/requirements.txt` 和 `scripts/install_dependencies.sh`、`scripts/install_interaction_dependencies.sh`，但这些脚本不包含模型，也不能完整重建原 Jetson GPU 二进制环境。

- 按目标设备的 JetPack/CUDA/cuDNN 版本安装匹配的 PyTorch、torchaudio、torchvision 和 CTranslate2。原本地 wheel 未上传；缺少该 wheel 时，安装脚本会尝试 PyPI 版本，ARM64 上可能仅有 CPU 支持。
- Kokoro 使用 `kokoro==0.9.4`、`misaki==0.9.4` 及其音频/G2P 依赖。原隔离目录的 PyTorch 为 2.8.0，不能把通用 wheel 当成经过验证的 Jetson CUDA 构建。
- 人物检测安装脚本使用 `ultralytics==8.3.162` 与 `opencv-python-headless==4.10.0.84`，放在独立目录，避免替换 ROS/UI 依赖的系统 OpenCV。
- 保留 Ubuntu 的 NumPy/OpenCV/PyQt5，避免给 ROS Humble 的 `cv_bridge` 引入不兼容的 ABI。原隔离环境不是锁定依赖清单，不能据此保证在其他设备上一键复现。
- `third_party/livox_sdk2_system_spdlog/` 是小体积、经过本地修改的源码，随仓库保留。构建前安装 `libspdlog-dev`，运行 `bash scripts/build_livox_sdk.sh`；编译库仍被忽略。
- `scripts/env.sh` 当前要求一个已构建的 Livox ROS 2 underlay，需先准备其 `install/setup.bash`。在启动脚本前导出 `LIVOX_WS` 指向该工作区，并在 `.env` 中填写相同路径。可参考 [Livox 驱动说明](../ros2_ws/src/livox_ros_driver2/README.md) 构建；本项目保留的驱动有本地修改，见其 `LOCAL_CHANGES.md`。

依赖就绪后再从项目根目录执行：

```bash
bash scripts/build.sh
bash scripts/check_system.sh
```

以上为部署指引，本次整理没有安装依赖、重编译 ROS 或运行机器人动作。
