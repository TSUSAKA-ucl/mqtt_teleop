# mqtt_teleop

MQTT で受信した目標ジョイント角を **ZOH で oversample → 2次ベッセル LPF → `/control/move_js`** に変換する
ROS 2 (Humble) ノード。`MQTT_teleoperation_rl.py` / `MQTT_Client.py` の C++ 版。

## ビルド Build

1. Create a ROS 2 workspace (if not already created):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
2. Clone the repository into the `src` directory:
   ```bash
   git clone https://github.com/TSUSAKA-ucl/mqtt_teleop.git
   ```
3. Install dependencies:
   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```
   or manually install the required packages:
   ```
   sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev nlohmann-json3-dev libssl-dev
   ```
4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mqtt_teleop
   ```
   If you want to make a compile_commands.json for your IDE, you can use the following command:
   ```bash
   colcon build --packages-select mqtt_teleop --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   ln -s build/compile_commands.json .
   ```
## テスト
0. `mkcert` で作った、`rootCA.pem`とmqtt brokerおよびNext.js devサーバー用の証明書を、それぞれ`~/.local/share/mkcert/`と`~/.local/share/ssl/`に置く。`rootCA.pem` は `CA_CERTS_PATH` 環境変数で指定する。
1. [`./Docker/up.sh`](https://github.com/TSUSAKA-ucl/MetaworkMQTT/blob/dev-wss/Docker/up.sh) in [`MetaworkMQTT`](https://github.com/TSUSAKA-ucl/MetaworkMQTT). (MQTT broker + MetaworkMQTT manager)
2. [`pnpm dev-mkcert`](https://github.com/TSUSAKA-ucl/piper-metawork/blob/main/package.json) in [`piper-metawork`](https://github.com/TSUSAKA-ucl/piper-metawork) (Next.js dev server)
3. Launch this ROS 2 node:
   ```
   source ~/ros2_ws/install/setup.bash
   source ~/ros2_ws/install/mqtt_teleop/share/mqtt_teleop/config/environment.bash
   ros2 launch mqtt_teleop teleop-test.launch.py
   ```
4. Access `https://localhost:3000` in your browser. You should see a WebXR PiPER demo page

## アーム動作テスト
0. 以下、broker, devサーバーは192.168.207.145とし、SANの入ったcertとkeyを持っているとする。clientはそれを認証するrootCAのcertを持っているとする
1. broker, MetaworkMQTT manager
   ```
   ./MetaworkMQTT/Docker/up.sh
   ```
2. Next.js devサーバー
   ```
   cd piper-metawork
   ./localcerts/copy-certs.sh
   source ./setup-env.sh # for cross origin
   pnpm dev-mkcert
   ```
3. piper power-on (エッジサーバー上で)
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 run mqtt_teleop v2_both.sh reset 
   ros2 run mqtt_teleop v2_both.sh enable 
   ros2 run mqtt_teleop v2_both.sh go_zero
   ```
4. 起動  
   launch `agx_arm_ctrl_node` and `mqtt_teleop_node` by `teleop_agx_arm.launch.py`  
   環境変数設定(`CA_CERTS_PATH`, `ROBOT_TYPE`, `ROBOT_UUID`)
   ```
   source ~/ros2_ws/src/mqtt_teleop/config/environment.bash
   ```
   launching
   ```
   ros2 launch mqtt_teleop teleop_agx_arm.launch.py  can_port:=can_piper_r4e31  arm_type:=piper  broker_uri:=wss://192.168.207.145:8333
   ```
   `can_port`と`broker_uri`は環境に合わせて変更する。`broker_uri` は
   `tcp:// ssl:// ws:// wss://` のどれでも対応可能　　
   全パラメータは `config/teleop.yaml` に既定値つきで記載済
6. 操作  
   ブラウザーで`https://192.168.207.145:3000`にアクセス　　
   WebXRコントローラーで操作

7. 停止  
   1. 安全な姿勢に移動
   2. `teleop_agx_arm.launch.py`をCtrl+Cで終了
   3. アーム停止  
	  1. `ros2 run mqtt_teleop v2_both.sh stop`
	  2. `ros2 run mqtt_teleop v2_both.sh disable`
	  3. 電源off

## 構成

| ファイル | 役割 |
|---|---|
| `include/mqtt_teleop/edge_state.hpp` | `EdgeState` (+ `STOPPING`) |
| `include/mqtt_teleop/MQTT_Client.hpp`, `src/MQTT_Client.cpp` | `MQTT_Client.py` と同名メソッドの C++ 版 (Paho C++ async_client) |
| `include/mqtt_teleop/lowpass_filter.hpp` | 2次ベッセル(双一次変換・prewarp, DF2T)。ROS 非依存 |
| `src/teleop_node.cpp` | FSM / ROS I/O / 制御ループ (専用スレッド + `clock_nanosleep` 絶対時刻) |

## FSM

```
ACQUIRED --(MQTT目標≈実関節 (0.01rad) が 0.1s 連続, 目標が新鮮)--> RUNNING
RUNNING  --(再acquire / 追従誤差>0.10rad が 0.05s / MQTT途絶 0.2s / NaN / feedback途絶 / 終了要求)--> STOPPING
STOPPING --(入力を直前値に固定して publish 継続。フィルタ収束 かつ 全関節 |vel|<0.05rad/s が 0.2s)--> ACQUIRED (終了要求時は EXITING)
```

* ACQUIRED では ROS へ一切 publish せず、`state_pub_rate_hz` で実関節角を MQTT へ `initialize` として publish。
* Python 版との違い: `on_acquire_topic` は RUNNING/STOPPING 中は state を直接書き換えず、
  `consume_acquire_request()` のフラグを立てるだけ(遷移は FSM が行う)。自動再接続で state を巻き戻さない。
* SIGINT/SIGTERM: soft stop → `{"state":"stop"}` ×2 → `mgr/unregister` → 終了。

## フィルタ(実測値, 本パッケージのコードで計算)

| fs | fc | 60Hz 減衰 | DC群遅延 |
|---|---|---|---|
| 200Hz | 10Hz | -33.5 dB | 21.5 ms |
| 200Hz | **15Hz (既定)** | **-26.4 dB** | **14.2 ms** |
| 200Hz | 20Hz | -21.3 dB | 10.5 ms |
| 200Hz | 30Hz | -14.1 dB | 6.7 ms |

ステップ応答のオーバーシュートは約 0.56%。起動時は実関節角で状態を初期化するためステップは出ません。

## 要確認(私の推測で置いた箇所)

1. **ツール値→グリッパ幅**: 旧 `finger_pos[mm] = tool*0.85+0.4` を `gripper_scale/offset` で m に換算(0..0.1にクランプ)。
2. **MQTT へ送る実関節値 `joints`**: 6要素(gripper 抜き)。frontend が7要素を期待するなら要修正。
3. **`move_js` の挙動**: MIT モード・スムージング無し。publish を止めた時に最終値を保持するか未確認。STOPPING→ACQUIRED 後は publish を止めます。
4. **Paho C++ API**: 開発環境に Paho/ROS が無くビルド未実施(スタブでの構文チェックのみ)。
   Ubuntu 22.04 の apt 版 (C++ 1.2.0 / C 1.3.9) を想定。`ssl_options_builder::verify()` 等でコンパイルエラーが出たら該当行を削除してください。
5. 関節リミット(`joint_lower/upper`)は既定で無効。速度クランプ(3 rad/s)のみ有効。
6. 閾値(0.10rad, 0.2s 等)は暫定値。実機での調整が必要です。
