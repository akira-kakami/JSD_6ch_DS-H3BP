# DS-H3BP1 6ch 変位センサ インターフェース

Zynq PL 向け変位センサ（DS-H3BP1）インターフェース RTL および Vivado シミュレーション環境一式。

6チャンネルの変位センサを同時サンプリングし、64bit タイムスタンプ付き AXI4-Stream パケットとして出力する。AXI4-Lite 経由で測定の開始／停止を制御できる。

---

## ファイル構成

```
.
├── ds_h3bp1_axi_stream.sv   # DUT: センサインターフェース RTL
├── tb_ds_h3bp1_vip.sv       # テストベンチ（Xilinx AXI VIP 使用）
└── create_bd.tcl            # Vivado プロジェクト＆Block Design 自動生成スクリプト
```

---

## モジュール仕様

### `ds_h3bp1_axi_stream`

#### パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `CLK_FREQ_HZ` | `100_000_000` | システムクロック周波数 [Hz] |
| `MEAS_TIME_US` | `500` | 測定時間（MES High 期間） [μs] |
| `SENSOR_CLK_US` | `2` | センサ CLK 半周期 [μs]（最小 2 推奨） |
| `DATA_BITS` | `31` | 1ch あたりのデータビット幅 |
| `NUM_CH` | `6` | チャンネル数 |
| `INTER_MEAS_US` | `20` | 測定間インターバル [μs] |
| `AXI_ADDR_WIDTH` | `4` | AXI4-Lite アドレスビット幅 |

#### ポート一覧

| ポート名 | 方向 | 幅 | 説明 |
|---|---|---|---|
| `clk` | in | 1 | システムクロック |
| `rst_n` | in | 1 | 非同期リセット（アクティブ Low） |
| `timestamp` | in | 64 | 外部フリーランカウンタ（タイムスタンプ） |
| `sens_meas` | out | 1 | センサ MES 信号（全 ch 共通） |
| `sens_clk` | out | 1 | センサ CLK 信号（全 ch 共通） |
| `sens_sout` | in | NUM_CH | センサ SOUT 信号（ch 独立） |
| `s_axil_*` | in/out | - | AXI4-Lite Slave（制御・ステータス） |
| `m_axis_*` | out | 32 | AXI4-Stream Master（データ出力） |
| `busy` | out | 1 | センサ通信中フラグ |
| `dbg_count` | out | NUM_CH×31 | デバッグ用 最終ラッチ値 |

---

## AXI4-Lite レジスタマップ

| オフセット | 名前 | アクセス | ビット | 説明 |
|---|---|---|---|---|
| `0x00` | CTRL | R/W | [0] | enable: 1=測定開始, 0=停止 |
| `0x04` | STATUS | RO | [1] | enabled: 現在の制御状態 |
| | | | [0] | busy: センサ通信中 |
| その他 | - | RO | - | `0xDEAD_BEEF` 返却 |

- リセット後はデフォルト停止（`enable=0`）
- 停止指示は `S_INTER` 状態で反映（測定シーケンスを中断しない安全停止）

---

## AXI4-Stream パケット構造

1パケット = 8 beat（32bit幅）

| Beat | データ | TLAST |
|---|---|---|
| 1 | `timestamp[63:32]` | 0 |
| 2 | `timestamp[31:0]` | 0 |
| 3 | `{1'b0, ch0[30:0]}` | 0 |
| 4 | `{1'b0, ch1[30:0]}` | 0 |
| 5 | `{1'b0, ch2[30:0]}` | 0 |
| 6 | `{1'b0, ch3[30:0]}` | 0 |
| 7 | `{1'b0, ch4[30:0]}` | 0 |
| 8 | `{1'b0, ch5[30:0]}` | **1** |

---

## 測定 FSM

```
              enable=1
S_DISABLED ──────────→ S_IDLE
     ↑                    │ タイムスタンプラッチ / MES・CLK High
     │                    ↓
     │               S_MEAS（500μs カウント）
     │                    │ MES Low
     │                    ↓
     │         ┌──→ S_SHIFT（CLK High 期間カウント＆末尾で CLK 落下）
     │         │         │ sclk_cnt == SCLK_HALF_CNT → CLK Low
     │         │         ↓
     │         │    S_LATCH（CLK 立ち下がりの 1 サイクル後にサンプル）
     │         │         │ bit_idx < DATA_BITS-1
     │         │         ↓
     │         └──── S_CLK_LOW（CLK Low 期間・CLK 立ち上げ）
     │               （31bit 完了）
     │                    ↓
     │               S_TX（AXI-Stream beat 送出）
     │                    │ 全 beat 送出完了
     │                    ↓
     └── enable=0 ── S_INTER（インターバル待機）
                          │ enable=1
                          └──────────→ S_IDLE
```

### センササンプリングタイミング（詳細）

以下は `SCLK_HALF_CNT = 2`（`SENSOR_CLK_US=2`, `CLK_FREQ_HZ=100MHz` → 実際は 199）の簡略例。
周期番号は PLクロックの立ち上がりエッジを示す。

```
PL clk :  __|‾|__|‾|__|‾|__|‾|__|‾|__|‾|__|‾|__|‾|__|‾|__|‾|__
周期    :    0   1   2   3   4   5   6   7   8   9  10  11  12

state   :  [   S_CLK_LOW   ][     S_SHIFT      ][LAT][   S_CLK_LOW   ][S_SHIFT...
sclk_cnt:    0   1   2          0   1   2              0   1   2

sens_clk:  _________________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_____________________
                             ↑                 ↓
                          CLK 立ち上がり     CLK 立ち下がり
                         （周期2→3 の間）   （周期5→6 の間）
                          ※ NBA が周期2     ※ NBA が周期5
                            の posedge で      の posedge で
                            sens_clk<=1 発行   sens_clk<=0 発行

sens_sout: ──────[  Dn-1  ]──────────────────[  Dn  ]──────────────────
                                  ↑                        ↑
                          CLK立ち上がり後                CLK立ち上がり後
                          センサが Dn 出力               センサが Dn+1 出力
                          （#2ns 伝搬遅延後）             （#2ns 伝搬遅延後）

SAMPLE  :                                     ↑ 周期6（S_LATCH）でサンプル
                                              │ ← CLK立ち下がりの 1PLクロック後
                                              └ shift_reg[ch][bit_idx] <= sens_sout[ch]
```

**フロー詳細（各周期の動作）:**

| 周期 | state | sclk_cnt | sens_clk | 動作 |
|---:|---|---:|:---:|---|
| 0〜1 | S_CLK_LOW | 0〜1 | 0 | カウント中 |
| 2 | S_CLK_LOW | 2 | 0 | `sens_clk <= 1` 発行・S_SHIFT へ |
| 3 | S_SHIFT | 0 | **1** | CLK High 確定。センサが #2ns 後に Dn を SOUT へ出力 |
| 4 | S_SHIFT | 1 | 1 | カウント中。SOUT = Dn 安定 |
| 5 | S_SHIFT | 2 | 1 | `sens_clk <= 0` 発行・S_LATCH へ |
| **6** | **S_LATCH** | ─ | **0** | **CLK Low 確定。`sens_sout` をサンプル → `shift_reg`** |
| 7〜9 | S_CLK_LOW | 0〜2 | 0 | カウント中 |
| 9 | S_CLK_LOW | 2 | 0 | `sens_clk <= 1` 発行・次ビットへ |

**CLK 波形の対称性:**

| 期間 | サイクル数（SCLK_HALF_CNT=N） | 備考 |
|---|---|---|
| CLK High（S_SHIFT） | N+1 サイクル | |
| CLK Low（S_LATCH + S_CLK_LOW） | N+2 サイクル | S_LATCH の 1 サイクル分 Low が長い |

- **D0〜D30 共通**: `S_SHIFT` で CLK を落とし → `S_LATCH`（1PLクロック後）でサンプル
- センサは CLK 立ち上がり後に次ビットを SOUT へセット（伝搬遅延あり）
- サンプル点では CLK が確実に Low 確定済みのため、センサ出力が安定している

---

## シミュレーション環境

### 前提条件

- Vivado 2022.x 以降（AXI VIP 搭載版）
- ターゲット: `xc7z020clg400-1`（`create_bd.tcl` で変更可）

### 手順

#### 1. Vivado プロジェクト生成

```bash
vivado -mode batch -source create_bd.tcl
```

または Vivado Tcl Console:

```tcl
source create_bd.tcl
```

生成物:
- Vivado プロジェクト: `vivado/ds_h3bp1_tb_prj/`
- Block Design: `ds_h3bp1_bd`（AXI VIP 内包）
- BD Wrapper: 自動生成

#### 2. シミュレーション実行

Vivado GUI:

```
Flow → Run Simulation → Run Behavioral Simulation
```

または Tcl Console:

```tcl
launch_simulation
run all
```

### テストケース

| TC | 内容 | 期待結果 |
|---|---|---|
| TC1 | リセット直後 STATUS 確認 | `enabled=0`, `busy=0` |
| TC2 | CTRL `enable=1` 書き込み＆readback | write resp=OKAY, readback=1 |
| TC3 | パケット#1 受信・全 ch データ検証 | 全 ch 値一致 |
| TC4 | `enable=0` 安全停止確認 | 測定完了後に停止 |
| TC5 | 停止中 sens_meas / sens_clk 監視 | 1000 サイクル Low 確認 |
| TC6 | `enable=1` 再開 → パケット#2 検証 | 全 ch 値一致 |
| TC7 | 無効アドレス読み出し | `0xDEAD_BEEF` 返却 |
| TC8 | 連続 2 パケット TLAST 境界確認 | 各パケット末尾のみ TLAST=1 |

### シミュレーション短縮パラメータ（TB 内）

| パラメータ | 実機値 | TB 値 |
|---|---|---|
| `MEAS_TIME_US` | 500 μs | 5 μs |
| `SENSOR_CLK_US` | 2 μs | 1 μs |
| `INTER_MEAS_US` | 20 μs | 2 μs |

---

## Block Design 構成（create_bd.tcl）

```
ds_h3bp1_bd
├── axil_mst_0 : axi_vip        (AXI4-Lite Master, ADDR_WIDTH=4, DATA_WIDTH=32)
└── axis_slv_0 : axi4stream_vip (AXI4-Stream Slave, TDATA=32bit, TLAST有効)
```

全ポートは External に引き出し、テストベンチから DUT に直結。

---

## VIP エージェント使用例

```systemverilog
// AXI-Lite Master エージェント
ds_h3bp1_bd_axil_mst_0_mst_t axil_agent;
axil_agent = new("axil_agent", tb.u_bd.axil_mst_0.inst.IF);
axil_agent.start_master();

// AXI4-Stream Slave エージェント
ds_h3bp1_bd_axis_slv_0_slv_t axis_agent;
axis_agent = new("axis_agent", tb.u_bd.axis_slv_0.inst.IF);
axis_agent.start_slave();
```

---

## 設計上の注意事項

- `SENSOR_CLK_US >= 2` を推奨（最小 CLK 半周期 = 2μs）
- 停止指示（`enable=0`）は `S_INTER` 状態まで反映されない（測定シーケンスは必ず完結する）
- リセット中は `sens_meas` / `sens_clk` をともに Low に保持し、センサへの誤信号を防止する
- `timestamp` 入力は外部フリーランカウンタを接続すること（未接続の場合は `0` 固定でも動作可）
