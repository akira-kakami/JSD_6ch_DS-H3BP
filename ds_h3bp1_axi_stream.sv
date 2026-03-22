// =============================================================================
// DS-H3BP1 変位センサ インターフェース（後継機 31bit版）
// 6ch拡張 + AXI Lite制御 + 64bitタイムスタンプ + AXI Stream出力
// for Zynq PL
//
// ■ 構成
//   MES（測定時間）・CLK は全チャンネル共通
//   SOUT は各チャンネル独立（NUM_CH本）
//   全チャンネルを同時サンプリング
//
// ■ AXI Liteレジスタマップ（32bit, 4byteアライン）
//   Offset 0x00 : CTRL   [R/W] bit[0] = enable（1=測定開始, 0=停止）
//   Offset 0x04 : STATUS [RO]  bit[0] = busy（センサ通信中）
//                               bit[1] = enabled（現在の制御状態）
//
//   ・リセット後はデフォルト停止（enable=0）
//   ・停止はS_INTERまたはS_DISABLEDで確定（測定シーケンス中は中断しない）
//
// ■ AXI Streamパケット（8 beat / パケット, 32bit幅）
//   Beat 1 : timestamp[63:32]        TLAST=0
//   Beat 2 : timestamp[31:0]         TLAST=0
//   Beat 3 : {1'b0, ch0_count[30:0]} TLAST=0
//   Beat 4 : {1'b0, ch1_count[30:0]} TLAST=0
//   Beat 5 : {1'b0, ch2_count[30:0]} TLAST=0
//   Beat 6 : {1'b0, ch3_count[30:0]} TLAST=0
//   Beat 7 : {1'b0, ch4_count[30:0]} TLAST=0
//   Beat 8 : {1'b0, ch5_count[30:0]} TLAST=1
//
// ■ sens_sout サンプリングタイミング
//
//   [D0] S_MEAS → S_LATCH_D0 → S_CLK_LOW
//
//     PLclk:    ... __|‾|__|‾|__|‾|__
//                     A   B   C
//     S_MEAS末:        sens_meas <= 0  (clkA更新)
//     sens_meas:            ‾‾‾|_____  (clkA→B間にLOW確定)
//     センサD0出力:               |←遅延→| D0有効
//     S_LATCH_D0(clkB):    遷移のみ、サンプルなし
//     S_WAIT_D0 (clkC):    sens_sout をサンプル  ← sens_meas落下から2PLクロック後
//
//   [D1〜D30] S_CLK_LOW → S_CLK_HIGH (sclk_cnt==SCLK_HALF_CNTでサンプル)
//
//     PLclk:    ... __|‾|__|‾|__|‾|__|‾|__
//                     A   B   C   D
//     S_CLK_LOW末:     sens_clk <= 1  (clkA更新)
//     sens_clk:             ‾‾‾‾‾‾‾‾‾  (clkA→B間にHIGH確定)
//     センサSOUT応答:              |←遅延→| データ有効
//     S_CLK_HIGH sclk_cnt==SCLK_HALF_CNT: sens_sout をサンプル（CLK立ち下がりと同サイクル）
//
//   SENSOR_CLK_US >= 2 必須（最小CLK半周期 = 2μs → sclk_cnt最大値 >= 1）
//
// ■ パラメータ設定例（100MHz動作時）
//   CLK_FREQ_HZ    = 100_000_000
//   MEAS_TIME_US   = 500
//   SENSOR_CLK_US  = 2          → CLK周期4μs, サンプルはCLK立ち上がりから20ns後
//   DATA_BITS      = 31
//   NUM_CH         = 6
//   INTER_MEAS_US  = 20
// =============================================================================

module ds_h3bp1_axi_stream #(
    parameter int CLK_FREQ_HZ    = 100_000_000,
    parameter int MEAS_TIME_US   = 500,
    parameter int SENSOR_CLK_US  = 2,
    parameter int DATA_BITS      = 31,
    parameter int NUM_CH         = 6,
    parameter int INTER_MEAS_US  = 20,
    // AXI Lite アドレスビット幅（最低3bit: 0x00〜0x04の2レジスタ）
    parameter int AXI_ADDR_WIDTH = 4
)(
    // ---- システム ----
    input  logic                      clk,
    input  logic                      rst_n,

    // ---- 64bit タイムスタンプ（外部フリーランカウンタ等） ----
    input  logic [63:0]               timestamp,

    // ---- センサI/F（MES・CLK共通, SOUT独立） ----
    output logic                      sens_meas,
    output logic                      sens_clk,
    input  logic [NUM_CH-1:0]         sens_sout,

    // ---- AXI Lite Slave（制御・ステータス） ----
    // Write address channel
    input  logic [AXI_ADDR_WIDTH-1:0] s_axil_awaddr,
    input  logic                      s_axil_awvalid,
    output logic                      s_axil_awready,
    // Write data channel
    input  logic [31:0]               s_axil_wdata,
    input  logic [3:0]                s_axil_wstrb,
    input  logic                      s_axil_wvalid,
    output logic                      s_axil_wready,
    // Write response channel
    output logic [1:0]                s_axil_bresp,
    output logic                      s_axil_bvalid,
    input  logic                      s_axil_bready,
    // Read address channel
    input  logic [AXI_ADDR_WIDTH-1:0] s_axil_araddr,
    input  logic                      s_axil_arvalid,
    output logic                      s_axil_arready,
    // Read data channel
    output logic [31:0]               s_axil_rdata,
    output logic [1:0]                s_axil_rresp,
    output logic                      s_axil_rvalid,
    input  logic                      s_axil_rready,

    // ---- AXI Stream Master（32bit幅） ----
    output logic [31:0]               m_axis_tdata,
    output logic                      m_axis_tvalid,
    output logic                      m_axis_tlast,
    input  logic                      m_axis_tready,

    // ---- ステータス出力 ----
    output logic                      busy,
    output logic [NUM_CH-1:0][30:0]   dbg_count
);

    // =========================================================================
    // タイミング定数
    // =========================================================================
    localparam int CLKS_PER_US   = CLK_FREQ_HZ / 1_000_000;
    localparam int MEAS_CNT      = MEAS_TIME_US  * CLKS_PER_US - 1;
    localparam int SCLK_HALF_CNT = SENSOR_CLK_US * CLKS_PER_US - 1;
    localparam int INTER_CNT     = INTER_MEAS_US * CLKS_PER_US - 1;
    localparam int TOTAL_BEATS   = 2 + NUM_CH;  // タイムスタンプ2 + データNUM_CH

    // =========================================================================
    // レジスタアドレス定義
    // =========================================================================
    localparam logic [AXI_ADDR_WIDTH-1:0] ADDR_CTRL   = 'h00;
    localparam logic [AXI_ADDR_WIDTH-1:0] ADDR_STATUS = 'h04;

    // =========================================================================
    // AXI Lite 内部レジスタ
    // =========================================================================
    logic        reg_enable;    // CTRL[0]: 測定enable/disable

    // AXI Lite ハンドシェイク用内部レジスタ
    logic [AXI_ADDR_WIDTH-1:0] axil_awaddr_lat;
    logic                      axil_aw_done;   // AWハンドシェイク完了フラグ
    logic                      axil_w_done;    // Wハンドシェイク完了フラグ

    // =========================================================================
    // AXI Lite Write チャンネル
    // =========================================================================

    // AW チャンネル（アドレスラッチ）
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axil_awready  <= 1'b0;
            axil_awaddr_lat <= '0;
            axil_aw_done    <= 1'b0;
        end else begin
            if (!axil_aw_done) begin
                if (s_axil_awvalid && !s_axil_awready) begin
                    s_axil_awready  <= 1'b1;
                    axil_awaddr_lat <= s_axil_awaddr;
                end else begin
                    s_axil_awready  <= 1'b0;
                end
                if (s_axil_awvalid && s_axil_awready)
                    axil_aw_done <= 1'b1;
            end else if (s_axil_bvalid && s_axil_bready) begin
                axil_aw_done <= 1'b0;
            end
        end
    end

    // W チャンネル（データ受信 + レジスタ書き込み）
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axil_wready <= 1'b0;
            axil_w_done   <= 1'b0;
            reg_enable    <= 1'b0;  // ★ リセット後は停止状態
        end else begin
            if (!axil_w_done) begin
                if (s_axil_wvalid && !s_axil_wready) begin
                    s_axil_wready <= 1'b1;
                end else begin
                    s_axil_wready <= 1'b0;
                end

                // AW・W 両方ハンドシェイク完了でレジスタ書き込み
                if (s_axil_wvalid && s_axil_wready && axil_aw_done) begin
                    axil_w_done <= 1'b1;
                    case (axil_awaddr_lat)
                        ADDR_CTRL: begin
                            // CTRL: bit[0] = enable
                            if (s_axil_wstrb[0]) reg_enable <= s_axil_wdata[0];
                        end
                        // STATUS は読み出し専用（書き込み無視）
                        default: ;
                    endcase
                end
            end else if (s_axil_bvalid && s_axil_bready) begin
                axil_w_done <= 1'b0;
            end
        end
    end

    // B チャンネル（書き込み応答）
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axil_bvalid <= 1'b0;
            s_axil_bresp  <= 2'b00;
        end else begin
            if (axil_aw_done && axil_w_done && !s_axil_bvalid) begin
                s_axil_bvalid <= 1'b1;
                s_axil_bresp  <= 2'b00; // OKAY
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI Lite Read チャンネル
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axil_arready <= 1'b0;
            s_axil_rvalid  <= 1'b0;
            s_axil_rdata   <= '0;
            s_axil_rresp   <= 2'b00;
        end else begin
            if (s_axil_arvalid && !s_axil_arready) begin
                s_axil_arready <= 1'b1;
            end else begin
                s_axil_arready <= 1'b0;
            end

            if (s_axil_arvalid && s_axil_arready && !s_axil_rvalid) begin
                s_axil_rvalid <= 1'b1;
                s_axil_rresp  <= 2'b00;
                case (s_axil_araddr)
                    ADDR_CTRL:
                        s_axil_rdata <= {31'd0, reg_enable};
                    ADDR_STATUS:
                        s_axil_rdata <= {30'd0, reg_enable, busy};
                        //                       ^^^^^^^^^^^  ^^^^^
                        //                       bit[1]=enabled  bit[0]=busy
                    default:
                        s_axil_rdata <= 32'hDEAD_BEEF; // 未定義アドレス
                endcase
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // 測定FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_DISABLED = 4'd0,  // 停止中（enable=0）
        S_IDLE     = 4'd1,  // 測定開始待ち（タイムスタンプラッチ → S_MEAS）
        S_MEAS     = 4'd2,  // 測定時間HIGH（全chセンサが内部カウント中）
        S_SHIFT    = 4'd3,  // CLK HIGH末尾でサンプル＆CLK落下、全ビット共通
        S_CLK_LOW  = 4'd4,  // センサCLK LOW期間（立ち上がりのみ、サンプルなし）
        S_TX       = 4'd5,  // AXI Stream beat送出
        S_INTER    = 4'd6   // 次測定までインターバル
    } state_t;

    // サンプルタイミング: CLK HIGH末尾（立ち下がりと同サイクル）
    // D0〜D30 全ビット共通: S_SHIFT の sclk_cnt==SCLK_HALF_CNT でサンプル

    state_t state;

    // ---- 内部レジスタ ----
    logic [$clog2(MEAS_CNT+1)-1:0]       meas_cnt;
    logic [$clog2(SCLK_HALF_CNT+1)-1:0]  sclk_cnt;
    logic [$clog2(INTER_CNT+1)-1:0]      inter_cnt;
    logic [$clog2(DATA_BITS)-1:0]         bit_idx;
    logic [$clog2(TOTAL_BEATS)-1:0]       beat_idx;

    logic [NUM_CH-1:0][DATA_BITS-1:0]    shift_reg;
    logic [NUM_CH-1:0][DATA_BITS-1:0]    data_latch;
    logic [63:0]                          ts_latch;

    // パラメータ制約チェック
    // SENSOR_CLK_US >= 1 必須（SCLK_HALF_CNT >= 0、ただし実用上は >= 2 推奨）
    initial begin
        assert (SCLK_HALF_CNT >= 0)
            else $fatal(1, "SENSOR_CLK_US must be >= 1 (SCLK_HALF_CNT=%0d)", SCLK_HALF_CNT);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_DISABLED;    // ★ リセット後は停止
            meas_cnt      <= '0;
            sclk_cnt      <= '0;
            inter_cnt     <= '0;
            bit_idx       <= '0;
            beat_idx      <= '0;
            shift_reg     <= '0;
            data_latch    <= '0;
            ts_latch      <= '0;
            sens_meas     <= 1'b0;
            sens_clk      <= 1'b0;
            m_axis_tdata  <= '0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            case (state)

                // ----------------------------------------------------------------
                // S_DISABLED: enableが立つまで待機
                //   MES・CLKは必ずLowに保持（センサへの誤信号防止）
                // ----------------------------------------------------------------
                S_DISABLED: begin
                    sens_meas     <= 1'b0;
                    sens_clk      <= 1'b0;
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    if (reg_enable)
                        state <= S_IDLE;
                end

                // ----------------------------------------------------------------
                // S_IDLE: タイムスタンプラッチ → 測定開始
                // ----------------------------------------------------------------
                S_IDLE: begin
                    ts_latch  <= timestamp;
                    sens_meas <= 1'b1;
                    sens_clk  <= 1'b1;  // CLKはMES Highと同時にHighを維持（タイミングチャート準拠）
                    meas_cnt  <= '0;
                    state     <= S_MEAS;
                end

                // ----------------------------------------------------------------
                // S_MEAS: 測定時間（500μs）カウント
                // ----------------------------------------------------------------
                S_MEAS: begin
                    if (meas_cnt == MEAS_CNT[$clog2(MEAS_CNT+1)-1:0]) begin
                        sens_meas <= 1'b0;  // MES立ち下がり → D0確定
                        // CLKはHighのまま維持（sens_clkはS_IDLEで1'b1に設定済み）
                        meas_cnt  <= '0;
                        sclk_cnt  <= '0;
                        bit_idx   <= '0;    // D0から開始
                        state     <= S_SHIFT;
                    end else begin
                        meas_cnt <= meas_cnt + 1'b1;
                    end
                end

                // ----------------------------------------------------------------
                // S_SHIFT: CLK HIGH期間のカウント＆末尾でサンプル
                //   D0〜D30 全ビット共通
                //   sclk_cnt==SCLK_HALF_CNT: sens_sout をサンプル＆CLK立ち下がり
                //   ・D0: MES立ち下がり後の最初のCLK HIGH末尾
                //   ・D1〜D30: 各CLK立ち上がり後のCLK HIGH末尾
                // ----------------------------------------------------------------
                S_SHIFT: begin
                    if (sclk_cnt == SCLK_HALF_CNT[$clog2(SCLK_HALF_CNT+1)-1:0]) begin
                        // CLK HIGH末尾: 全ビット共通サンプル＆CLK落下
                        for (int ch = 0; ch < NUM_CH; ch++)
                            shift_reg[ch][bit_idx] <= sens_sout[ch];

                        sens_clk <= 1'b0;
                        sclk_cnt <= '0;

                        if (bit_idx == (DATA_BITS - 1)) begin
                            // 全31bit取得完了
                            data_latch <= shift_reg;
                            beat_idx   <= '0;
                            state      <= S_TX;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            state   <= S_CLK_LOW;
                        end
                    end else begin
                        sclk_cnt <= sclk_cnt + 1'b1;
                    end
                end

                // ----------------------------------------------------------------
                // S_CLK_LOW: CLK LOW期間（半周期待機してCLK立ち上げ → S_SHIFTへ）
                //   サンプルなし
                // ----------------------------------------------------------------
                S_CLK_LOW: begin
                    if (sclk_cnt == SCLK_HALF_CNT[$clog2(SCLK_HALF_CNT+1)-1:0]) begin
                        sens_clk <= 1'b1;
                        sclk_cnt <= '0;
                        state    <= S_SHIFT;
                    end else begin
                        sclk_cnt <= sclk_cnt + 1'b1;
                    end
                end

                // ----------------------------------------------------------------
                // S_TX: AXI Stream beat送出
                //   beat_idx 0   : ts[63:32]           TLAST=0
                //   beat_idx 1   : ts[31:0]            TLAST=0
                //   beat_idx 2〜7: {1'b0,chN[30:0]}    TLAST=1(最終beat)
                // ----------------------------------------------------------------
                S_TX: begin
                    case (beat_idx)
                        'd0:     m_axis_tdata <= ts_latch[63:32];
                        'd1:     m_axis_tdata <= ts_latch[31:0];
                        default: m_axis_tdata <= {1'b0,
                                     data_latch[beat_idx - 2][DATA_BITS-1:0]};
                    endcase

                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (beat_idx == TOTAL_BEATS - 1) ? 1'b1 : 1'b0;

                    if (m_axis_tvalid && m_axis_tready) begin
                        if (beat_idx == TOTAL_BEATS - 1) begin
                            // 全beat送出完了
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            inter_cnt     <= '0;
                            state         <= S_INTER;
                        end else begin
                            beat_idx <= beat_idx + 1'b1;
                        end
                    end
                end

                // ----------------------------------------------------------------
                // S_INTER: インターバル待機
                //   ★ ここでenable=0を検出して停止（測定シーケンスは完結させる）
                // ----------------------------------------------------------------
                S_INTER: begin
                    if (inter_cnt == INTER_CNT[$clog2(INTER_CNT+1)-1:0]) begin
                        // enable確認：0なら停止、1なら次の測定へ
                        if (reg_enable)
                            state <= S_IDLE;
                        else
                            state <= S_DISABLED;
                    end else begin
                        inter_cnt <= inter_cnt + 1'b1;
                    end
                end

                default: state <= S_DISABLED;
            endcase
        end
    end

    // =========================================================================
    // 出力アサイン
    // =========================================================================
    assign busy = (state == S_MEAS    ||
                   state == S_SHIFT   ||
                   state == S_CLK_LOW);

    generate
        for (genvar ch = 0; ch < NUM_CH; ch++) begin : g_dbg
            assign dbg_count[ch] = data_latch[ch][DATA_BITS-1:0];
        end
    endgenerate

endmodule


// =============================================================================
// テストベンチ
// =============================================================================
`ifdef SIM_TESTBENCH

module tb_ds_h3bp1_axi_stream;

    localparam int CLK_PERIOD_NS = 10;
    localparam int NUM_CH        = 6;
    localparam int DATA_BITS     = 31;
    localparam int TOTAL_BEATS   = 2 + NUM_CH;

    // ---- 信号 ----
    logic              clk, rst_n;
    logic [63:0]       timestamp;
    logic              sens_meas, sens_clk;
    logic [NUM_CH-1:0] sens_sout;
    // AXI Lite
    logic [3:0]  s_axil_awaddr;
    logic        s_axil_awvalid, s_axil_awready;
    logic [31:0] s_axil_wdata;
    logic [3:0]  s_axil_wstrb;
    logic        s_axil_wvalid, s_axil_wready;
    logic [1:0]  s_axil_bresp;
    logic        s_axil_bvalid, s_axil_bready;
    logic [3:0]  s_axil_araddr;
    logic        s_axil_arvalid, s_axil_arready;
    logic [31:0] s_axil_rdata;
    logic [1:0]  s_axil_rresp;
    logic        s_axil_rvalid, s_axil_rready;
    // AXI Stream
    logic [31:0] m_axis_tdata;
    logic        m_axis_tvalid, m_axis_tlast, m_axis_tready;
    logic        busy;
    logic [NUM_CH-1:0][30:0] dbg_count;

    // ---- DUT ----
    ds_h3bp1_axi_stream #(
        .CLK_FREQ_HZ   (100_000_000),
        .MEAS_TIME_US  (500),
        .SENSOR_CLK_US (2),
        .DATA_BITS     (DATA_BITS),
        .NUM_CH        (NUM_CH),
        .INTER_MEAS_US (20),
        .AXI_ADDR_WIDTH(4)
    ) dut (.*);

    // ---- クロック・タイムスタンプ ----
    initial clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) timestamp <= 64'd0;
        else        timestamp <= timestamp + 64'd1;

    // ---- センサモデル ----
    logic [DATA_BITS-1:0] fake_data [NUM_CH];
    int   bit_cnt [NUM_CH];
    logic d0_ready;

    initial begin
        d0_ready = 0;
        for (int i = 0; i < NUM_CH; i++) begin
            fake_data[i] = (i + 1) * 31'h0111111;
            bit_cnt[i]   = 0;
            sens_sout[i] = 0;
        end
    end

    always @(negedge sens_meas) begin
        #1ns;
        for (int i = 0; i < NUM_CH; i++) begin
            sens_sout[i] = fake_data[i][0];
            bit_cnt[i]   = 1;
        end
        d0_ready = 1;
    end

    always @(posedge sens_clk) begin
        if (d0_ready) begin
            #1ns;
            for (int i = 0; i < NUM_CH; i++) begin
                if (bit_cnt[i] < DATA_BITS) begin
                    sens_sout[i] = fake_data[i][bit_cnt[i]];
                    bit_cnt[i]   = bit_cnt[i] + 1;
                end
            end
        end
    end

    // ---- AXI Stream モニタ ----
    logic [63:0] rx_ts;
    int beat = 0;

    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            case (beat)
                0: begin
                    rx_ts[63:32] = m_axis_tdata;
                    $display("[Beat%0d] TS[63:32] = 0x%08X", beat+1, m_axis_tdata);
                    beat = 1;
                end
                1: begin
                    rx_ts[31:0] = m_axis_tdata;
                    $display("[Beat%0d] TS[31: 0] = 0x%08X -> TS=0x%016X",
                             beat+1, m_axis_tdata, {rx_ts[63:32], m_axis_tdata});
                    beat = 2;
                end
                default: begin
                    automatic int ch = beat - 2;
                    $display("[Beat%0d] ch%0d = 0x%07X (exp=0x%07X) TLAST=%0b",
                             beat+1, ch, m_axis_tdata[30:0], fake_data[ch], m_axis_tlast);
                    assert (m_axis_tdata[30:0] == fake_data[ch])
                        else $error("ch%0d mismatch!", ch);
                    if (beat == TOTAL_BEATS - 1) begin
                        $display("--- Packet OK ---");
                        beat = 0; d0_ready = 0;
                    end else beat = beat + 1;
                end
            endcase
        end
    end

    assign m_axis_tready = 1'b1;
    assign s_axil_bready = 1'b1;
    assign s_axil_rready = 1'b1;

    // ---- AXI Lite ライトタスク ----
    task automatic axil_write(input [3:0] addr, input [31:0] data);
        @(posedge clk);
        s_axil_awaddr  = addr;
        s_axil_awvalid = 1'b1;
        s_axil_wdata   = data;
        s_axil_wstrb   = 4'hF;
        s_axil_wvalid  = 1'b1;
        @(posedge clk iff (s_axil_awready && s_axil_wready));
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        @(posedge clk iff s_axil_bvalid);
        $display("[AXI-L WR] addr=0x%02X data=0x%08X resp=%0b", addr, data, s_axil_bresp);
    endtask

    task automatic axil_read(input [3:0] addr);
        @(posedge clk);
        s_axil_araddr  = addr;
        s_axil_arvalid = 1'b1;
        @(posedge clk iff s_axil_arready);
        s_axil_arvalid = 1'b0;
        @(posedge clk iff s_axil_rvalid);
        $display("[AXI-L RD] addr=0x%02X data=0x%08X", addr, s_axil_rdata);
    endtask

    // ---- メインシーケンス ----
    initial begin
        rst_n          = 0;
        s_axil_awvalid = 0; s_axil_wvalid  = 0; s_axil_arvalid = 0;
        s_axil_awaddr  = 0; s_axil_wdata   = 0; s_axil_wstrb   = 0;
        s_axil_araddr  = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        // --- STATUS確認（リセット直後: enabled=0, busy=0）---
        $display("\n=== 初期ステータス確認 ===");
        axil_read(4'h4);   // STATUS

        // --- CTRL: enable=1 → 測定開始 ---
        $display("\n=== CTRL enable=1（測定開始）===");
        axil_write(4'h0, 32'h0000_0001);
        axil_read(4'h4);   // STATUS確認

        // 約1.5パケット待つ（1pkt ≈ 640μs = 64000 clk）
        repeat (80_000) @(posedge clk);

        // --- CTRL: enable=0 → 次のS_INTERで停止 ---
        $display("\n=== CTRL enable=0（停止指示）===");
        axil_write(4'h0, 32'h0000_0000);
        axil_read(4'h4);   // STATUS確認

        // 停止完了まで待つ
        repeat (80_000) @(posedge clk);

        // --- STATUS確認（停止後: enabled=0, busy=0）---
        $display("\n=== 停止後ステータス確認 ===");
        axil_read(4'h4);

        // --- 再度enable=1 → 再測定開始 ---
        $display("\n=== CTRL enable=1（再開）===");
        axil_write(4'h0, 32'h0000_0001);
        repeat (80_000) @(posedge clk);

        $display("\nSimulation done.");
        $finish;
    end

endmodule

`endif // SIM_TESTBENCH
