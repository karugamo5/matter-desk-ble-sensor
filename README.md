# matter-desk-ble-sensor v1.0
XIAO ESP32S3でApple Watchを検出するMatterに対応したセンサー

## 概要 / Overview
このプログラムは Apple Watch 由来の BLE を観測し、Matter の人感センサー Occupancy (`0/1`) を更新します。

## 注意 / Disclaimer
個人開発のプログラムです。自己責任でお使いください。
（開発においてAIが使用されています）

## 前提条件 / Prerequisites
- **[ESP-IDF](https://github.com/espressif/esp-idf/blob/master/docs/en/get-started/macos-setup.rst) が必要です（必須）**
- 対応ボード: Seeed Studio XIAO ESP32S3 Plus [販売ページ](https://www.switch-science.com/products/10470)
- 搭載チップ: ESP32-S3
- 開発環境: macOS


動作確認は XIAO ESP32S3 Plus のみですが、ESP32-S3 系ボードであれば動作する可能性があります（未検証）。
Apple Watch（検出対象）はApple Watch SE 3 (26.3)で動作確認しています。

## 主な機能 / Features
- Apple Watch BLE 近接検出（受信広告ベース）
- Identity Resolving Key (IRK) を併用した検出もオプションで可能
- 部屋の距離判定（既定 7m）で遠方のデバイスを除外
- 複数のApple Watchを検出した時は一番近いデバイスを判定
- Apple Watch の画面ロックが解除されている場合のみ在席「判定」

## ディレクトリ構成 / Repository Layout
- `main/app_main.cpp`: アプリ本体（BLE/Matter 状態機械）
- `main/idf_component.yml`: 依存コンポーネント定義
- `tools/apply_esp_matter_compat_patch.sh`: esp_matter の互換パッチ適用
- `sdkconfig.defaults`: 既定設定

## クイックスタート / Quick Start
### 1) 設定 / Configure
`main/app_main.cpp` の以下を環境に合わせて調整してください。
- `kRoomScopeMaxDistanceM`
- `kPresenceDistanceThresholdM`

### 1.5) Identity Resolving Key (IRK)　を使用する場合
`main/app_main.cpp` の`kTargetIrkBase64`を設定してください。
値の取得は、[Home Addistantの記事](https://www.home-assistant.io/integrations/private_ble_device/#on-macos)を参照

### 2) ビルド / Build
```bash
cd ./matter-desk-ble-sensor
. $HOME/esp/esp-idf/export.sh

idf.py set-target esp32s3
idf.py fullclean
idf.py reconfigure
./tools/apply_esp_matter_compat_patch.sh
idf.py build
```

### 3) 書き込み・モニタ / Flash and Monitor
```bash
ls /dev/cu.*
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```
モニタ終了は `Ctrl+]` です。

### 4) 初期化（全消去） / Reset (Erase Flash)
NVS やコミッショニング情報も含めて初期化したい場合に実行します。
```bash
idf.py -p /dev/cu.usbmodemXXXX erase-flash
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

## ログの見方 / Logs
`kDebugMode`を `true` に設定すると、BLEスキャンの詳細や距離推定の内部状態もログに出力され、Apple Watchを検出するとLEDが点灯します。
### 状態ログ / State Logs
- `[ble] scan started`
- `[ble] scan complete reason=...`
- `[ble] present=1 ...`
- `[ble] present timeout -> 0`
- `[matter] occupancy=1/0`

## 現在のスキャン設定 / Current Scan Settings
- `kScanDurationMs=600`（1回のスキャン継続時間。200ms間隔の送信を約3周期観測）
- `kScanRetryMs=100`（次回スキャン開始までの待ち時間。小さいほど検知遅延が減る）
- `kPresenceTimeoutMs=10000`（最終検知から不在に戻すまでの時間）
- `kRoomScopeMaxDistanceM=7.0`（部屋内とみなす最大距離。遠方BLEを除外）
- `kPresenceDistanceThresholdM=1.0`（在席判定の距離しきい値）

## 仕組み / Mechanism
1. BLE広告を受信します。
   （IRK 未設定時は BLE-only。IRK 設定時は BLE + IRK 照合を併用）
2. Apple Manufacturer Data を解析し、`data_flags` から状態を判定します。
   - `0x98` → `active`
   - `0x18` → `locked`
3. 受信 RSSI から距離を推定し、EMA（平滑化）した距離で評価します。
4. `kRoomScopeMaxDistanceM` を超える信号は「部屋外」として解析対象から除外します。
5. 複数候補がある場合、同一スキャン内で最短距離の候補を代表として採用します。
6. 代表候補が `active` かつ `kPresenceDistanceThresholdM` 以内のときのみ在席ヒットを加算し、連続ヒット条件を満たすと `present=1` に遷移します。
7. 最終検知から `kPresenceTimeoutMs` を超えると `present=0` に戻し、Matter Occupancy を更新します。

### 参考文献
- [Github: ESPHome-Apple-Watch-detection](https://github.com/dalehumby/ESPHome-Apple-Watch-detection)
- [Github: continuity : An Apple Continuity Protocol Reverse Engineering Project](https://github.com/furiousMAC/continuity/blob/master/messages/nearby_info.md)

## 商標
- Apple、Apple Watch は Apple Inc. の商標です。
- Matter は Connectivity Standards Alliance の商標です。

## Build Dependencies (from `dependencies.lock`)
このリポジトリ自体にはサードパーティ実装を同梱していませんが、ビルド時に以下の依存コンポーネントを使用します。

- `espressif/esp_matter` `1.4.2~1` (direct)
- `espressif/cbor` `0.6.1~4`
- `espressif/esp-serial-flasher` `0.0.11`
- `espressif/esp_delta_ota` `1.1.4`
- `espressif/esp_diag_data_store` `1.1.0`
- `espressif/esp_diagnostics` `1.3.1`
- `espressif/esp_encrypted_img` `2.3.0`
- `espressif/esp_insights` `1.3.2`
- `espressif/esp_rcp_update` `1.3.1`
- `espressif/esp_secure_cert_mgr` `2.9.0`
- `espressif/jsmn` `1.1.0`
- `espressif/json_generator` `1.1.2`
- `espressif/json_parser` `1.0.3`
- `espressif/mdns` `1.10.0`
- `espressif/rmaker_common` `1.6.0`
- `ESP-IDF` `5.5.3`
