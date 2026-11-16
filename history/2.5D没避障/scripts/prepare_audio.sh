#!/usr/bin/env bash
# 为邦邦语音交互准备 PulseAudio/USB UAC 输入输出。所有操作均可重复执行。

set -u

# 这款 UAC 在 Jetson 上偶尔会留下 RUNNING 但全零的 ALSA/Pulse 采集流。
# suspend-source 不足以恢复时，启动 UI 前重建整个双工 profile 才能可靠复位。
if [[ -n "${GO2_AUDIO_PULSE_CARD:-}" ]] && command -v pactl >/dev/null 2>&1; then
  if pactl set-card-profile "${GO2_AUDIO_PULSE_CARD}" off >/dev/null 2>&1; then
    sleep 0.3
    if pactl set-card-profile "${GO2_AUDIO_PULSE_CARD}" \
      "${GO2_AUDIO_PULSE_PROFILE:-output:analog-stereo+input:mono-fallback}" \
      >/dev/null 2>&1; then
      sleep 0.3
      printf '[OK]   USB 双工音频流已重建: %s\n' "${GO2_AUDIO_PULSE_CARD}"
    else
      printf '[WARN] USB 音频 profile 恢复失败: %s\n' \
        "${GO2_AUDIO_PULSE_CARD}" >&2
    fi
  fi
fi

if [[ -n "${PULSE_SOURCE:-}" ]] && command -v pactl >/dev/null 2>&1; then
  if pactl set-default-source "${PULSE_SOURCE}" >/dev/null 2>&1; then
    pactl set-source-mute "${PULSE_SOURCE}" 0 >/dev/null 2>&1 || true
    # 重启单个 source，修复 Jetson 上偶发的 RUNNING 但 PCM 数据全零问题。
    pactl suspend-source "${PULSE_SOURCE}" 1 >/dev/null 2>&1 || true
    sleep 0.2
    pactl suspend-source "${PULSE_SOURCE}" 0 >/dev/null 2>&1 || true
    printf '[OK]   语音输入已连接并重置: %s\n' "${PULSE_SOURCE}"
  else
    printf '[WARN] 找不到配置的语音输入: %s\n' "${PULSE_SOURCE}" >&2
  fi
fi

if [[ -n "${PULSE_SINK:-}" ]] && command -v pactl >/dev/null 2>&1; then
  if pactl set-default-sink "${PULSE_SINK}" >/dev/null 2>&1; then
    pactl set-sink-mute "${PULSE_SINK}" 0 >/dev/null 2>&1 || true
    pactl set-sink-volume "${PULSE_SINK}" \
      "${GO2_AUDIO_OUTPUT_VOLUME:-125%}" >/dev/null 2>&1 || true
    printf '[OK]   语音输出已连接: %s\n' "${PULSE_SINK}"
  else
    printf '[WARN] 找不到配置的语音输出: %s\n' "${PULSE_SINK}" >&2
  fi
fi

if [[ -n "${GO2_AUDIO_ALSA_CARD:-}" ]] && command -v amixer >/dev/null 2>&1; then
  # 部分 USB 阵列（如 XVF3800）除 PulseAudio 音量外还有独立的硬件播放级。
  # 该级过低时，即使 sink 已超过 100%，实际扬声器仍会很小。
  if [[ -n "${GO2_AUDIO_PLAYBACK_CONTROL:-}" ]]; then
    amixer -c "${GO2_AUDIO_ALSA_CARD}" \
      sset "${GO2_AUDIO_PLAYBACK_CONTROL}" \
      "${GO2_AUDIO_PLAYBACK_VOLUME:-100%}" unmute \
      >/dev/null 2>&1 || true
  fi
  # 保持 AGC 关闭，避免自动把底噪继续抬升；输入增益由部署配置明确控制。
  amixer -c "${GO2_AUDIO_ALSA_CARD}" sset 'Auto Gain Control' \
    "${GO2_AUDIO_AGC:-off}" >/dev/null 2>&1 || true
  amixer -c "${GO2_AUDIO_ALSA_CARD}" sset 'Mic' \
    "${GO2_AUDIO_MIC_GAIN:-80%}" cap \
    >/dev/null 2>&1 || true
fi

exit 0
