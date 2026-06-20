#!/usr/bin/env bash
# Build and deploy the Android relay app to a connected device.
# Usage: ./deploy.sh [server_url]
#   server_url: default WebSocket URL to embed in the app (default: auto-detected laptop IP:8765)
set -euo pipefail

cd "$(dirname "$0")"

# Auto-detect laptop WiFi IP (prefer non-VPN interface)
DEFAULT_IP=$(ip -4 addr show | grep -oP 'inet\s+\K[0-9.]+' | grep -v '127.0.0.1' | grep -v '^10\.' | head -1)
SERVER_URL="${1:-ws://${DEFAULT_IP}:8765}"
echo ">>> Default server URL: $SERVER_URL"

# Update default URL in RelayActivity.kt
ACTIVITY="app/src/main/kotlin/com/derpbot/app/RelayActivity.kt"
sed -i \
  -e "s|ws://[0-9.]\+:8765|${SERVER_URL}|g" \
  "$ACTIVITY"

# Build
export ANDROID_HOME="${ANDROID_HOME:-$HOME/Android}"
export PATH="$PATH:$ANDROID_HOME/cmdline-tools/bin:$ANDROID_HOME/platform-tools"
echo ">>> Building APK..."
./gradlew assembleDebug 2>&1 | tail -3

# Pick WiFi ADB device if available, else any single device
DEVICES=()
while IFS= read -r line; do
  DEVICES+=("$(echo "$line" | awk '{print $1}')")
done < <(adb devices | grep 'device$')

if [ ${#DEVICES[@]} -eq 0 ]; then
  echo ">>> ERROR: No ADB devices found"
  exit 1
fi

# Prefer WiFi (IP:port) device over USB serial
SERIAL=""
for d in "${DEVICES[@]}"; do
  if [[ "$d" == *:* ]]; then
    SERIAL="$d"
    break
  fi
done
SERIAL="${SERIAL:-${DEVICES[0]}}"
echo ">>> Using device: $SERIAL"

# Deploy
APK="app/build/outputs/apk/debug/app-debug.apk"
echo ">>> Installing APK..."
adb -s "$SERIAL" install -r "$APK" 2>&1 || {
  echo ">>> Install failed (signature mismatch?), uninstalling and reinstalling..."
  adb -s "$SERIAL" uninstall com.derpbot.app 2>/dev/null || true
  adb -s "$SERIAL" install "$APK"
}

echo ">>> Waking screen (Samsung Freecess freezes the app before BLE reaches ready if the screen is off/locked)..."
adb -s "$SERIAL" shell settings put system screen_off_timeout 2147483647 || true
adb -s "$SERIAL" shell input keyevent KEYCODE_WAKEUP
adb -s "$SERIAL" shell wm dismiss-keyguard

echo ">>> Starting app..."
adb -s "$SERIAL" shell am start -n com.derpbot.app/.RelayActivity
echo ">>> Done!"