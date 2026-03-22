// =============================================================================
// tb_ds_h3bp1_vip.sv
// DS-H3BP1 変位センサ インターフェース  AXI VIP テストベンチ
//
// ■ VIP 構成（Block Design: ds_h3bp1_bd）
//   axil_mst_0 : AXI4-Lite Master VIP → DUT の s_axil_* を駆動
//   axis_slv_0 : AXI4-Stream Slave VIP → DUT の m_axis_* を受信
//
// ■ センサモデル（Q7 タイミングチャート準拠）
//   MES High 中: CLK = High 維持
//   MES 立ち下がり: D0 確定（CLK High のまま）
//   CLK 立ち下がりごとに D0, D1 ... D30 を順次出力
//   CLK 立ち上がりで次ビットを SOUT にセット
//
// ■ テストケース
//   TC1 : リセット直後 STATUS 確認（enabled=0, busy=0）
//   TC2 : CTRL enable=1 書き込み・readback 確認
//   TC3 : 1 パケット受信検証（TS + 全 6ch + TLAST）
//   TC4 : CTRL enable=0 → 安全停止確認
//   TC5 : 停止中 sens_meas / sens_clk 出力なし確認
//   TC6 : enable=1 再開 → 2 パケット目整合性確認
//   TC7 : 無効アドレス読み出し（0xDEAD_BEEF 返却確認）
//   TC8 : 連続 2 パケット TLAST 境界確認
// =============================================================================

`timescale 1ns / 1ps

// ============================================================
// VIP パッケージ import
// BD 名と VIP インスタンス名から決まる（create_bd.tcl の設定と一致させること）
//   BD 名      : ds_h3bp1_bd
//   AXI-Lite   : axil_mst_0  → パッケージ名 ds_h3bp1_bd_axil_mst_0_pkg
//   AXI-Stream : axis_slv_0  → パッケージ名 ds_h3bp1_bd_axis_slv_0_pkg
// ============================================================
import axi_vip_pkg::*;
import axi4stream_vip_pkg::*;
import ds_h3bp1_bd_axil_mst_0_pkg::*;
import ds_h3bp1_bd_axis_slv_0_pkg::*;

module tb_ds_h3bp1_vip;

    // ==========================================================
    // パラメータ（シミュレーション短縮版）
    // 実機値: MEAS=500us, SCLK=2us, INTER=20us
    // ==========================================================
    localparam int CLK_FREQ_HZ    = 100_000_000;
    localparam int MEAS_TIME_US   = 5;    // 短縮
    localparam int SENSOR_CLK_US  = 1;    // 短縮
    localparam int DATA_BITS      = 31;
    localparam int NUM_CH         = 6;
    localparam int INTER_MEAS_US  = 2;    // 短縮
    localparam int AXI_ADDR_WIDTH = 4;
    localparam int TOTAL_BEATS    = 2 + NUM_CH;   // 8

    // レジスタアドレス
    localparam logic [31:0] ADDR_CTRL   = 32'h0000_0000;
    localparam logic [31:0] ADDR_STATUS = 32'h0000_0004;
    localparam logic [31:0] ADDR_RSVD   = 32'h0000_000C;  // 無効アドレス

    // タイムアウト（クロックサイクル数）
    localparam int TIMEOUT_CYC = 10_000_000;

    // ==========================================================
    // クロック・リセット
    // ==========================================================
    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    always #5ns clk = ~clk;   // 100 MHz

    // リセットシーケンス
    task automatic do_reset(input int cycles = 20);
        rst_n = 1'b0;
        repeat (cycles) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        $display("[%0t ns] ---- Reset released ----", $time / 1000);
    endtask

    // ==========================================================
    // タイムスタンプ（フリーランカウンタ）
    // ==========================================================
    logic [63:0] timestamp = '0;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) timestamp <= '0;
        else        timestamp <= timestamp + 1;

    // ==========================================================
    // DUT ポート配線
    // ==========================================================
    // AXI-Lite
    logic [AXI_ADDR_WIDTH-1:0] s_axil_awaddr;
    logic        s_axil_awvalid, s_axil_awready;
    logic [31:0] s_axil_wdata;
    logic [3:0]  s_axil_wstrb;
    logic        s_axil_wvalid,  s_axil_wready;
    logic [1:0]  s_axil_bresp;
    logic        s_axil_bvalid,  s_axil_bready;
    logic [AXI_ADDR_WIDTH-1:0] s_axil_araddr;
    logic        s_axil_arvalid, s_axil_arready;
    logic [31:0] s_axil_rdata;
    logic [1:0]  s_axil_rresp;
    logic        s_axil_rvalid,  s_axil_rready;
    // AXI-Stream
    logic [31:0] m_axis_tdata;
    logic        m_axis_tvalid, m_axis_tlast, m_axis_tready;
    // センサ
    logic              sens_meas, sens_clk;
    logic [NUM_CH-1:0] sens_sout;
    // デバッグ
    logic              busy;
    logic [NUM_CH-1:0][30:0] dbg_count;

    // ==========================================================
    // DUT インスタンス
    // ==========================================================
    ds_h3bp1_axi_stream #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .MEAS_TIME_US  (MEAS_TIME_US),
        .SENSOR_CLK_US (SENSOR_CLK_US),
        .DATA_BITS     (DATA_BITS),
        .NUM_CH        (NUM_CH),
        .INTER_MEAS_US (INTER_MEAS_US),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH)
    ) u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .timestamp      (timestamp),
        .sens_meas      (sens_meas),
        .sens_clk       (sens_clk),
        .sens_sout      (sens_sout),
        .s_axil_awaddr  (s_axil_awaddr),
        .s_axil_awvalid (s_axil_awvalid),
        .s_axil_awready (s_axil_awready),
        .s_axil_wdata   (s_axil_wdata),
        .s_axil_wstrb   (s_axil_wstrb),
        .s_axil_wvalid  (s_axil_wvalid),
        .s_axil_wready  (s_axil_wready),
        .s_axil_bresp   (s_axil_bresp),
        .s_axil_bvalid  (s_axil_bvalid),
        .s_axil_bready  (s_axil_bready),
        .s_axil_araddr  (s_axil_araddr),
        .s_axil_arvalid (s_axil_arvalid),
        .s_axil_arready (s_axil_arready),
        .s_axil_rdata   (s_axil_rdata),
        .s_axil_rresp   (s_axil_rresp),
        .s_axil_rvalid  (s_axil_rvalid),
        .s_axil_rready  (s_axil_rready),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tlast   (m_axis_tlast),
        .m_axis_tready  (m_axis_tready),
        .busy           (busy),
        .dbg_count      (dbg_count)
    );

    // ==========================================================
    // VIP エージェント宣言
    // ==========================================================
    ds_h3bp1_bd_axil_mst_0_mst_t  axil_agent;
    ds_h3bp1_bd_axis_slv_0_slv_t  axis_agent;

    // ==========================================================
    // AXI-Lite BFM タスク
    // ==========================================================

    // レジスタ書き込み（AW + W チャンネルを同時発行）
    task automatic axil_write(
        input  logic [31:0] addr,
        input  logic [31:0] data,
        output logic [1:0]  resp
    );
        axi_transaction wr_trans;
        axi_transaction wr_resp;

        wr_trans = axil_agent.wr_driver.create_transaction("wr");
        wr_trans.set_write_cmd(addr, XIL_AXI_BURST_TYPE_FIXED,
                               0, 0, XIL_AXI_SIZE_4BYTE);
        wr_trans.set_data_beat(0, data);
        wr_trans.set_strb_beat(0, 4'hF);
        axil_agent.wr_driver.send(wr_trans);
        axil_agent.wr_driver.wait_rsp(wr_resp);
        resp = wr_resp.get_bresp();

        $display("[%6t ns]  AXIL WR  addr=%08Xh  data=%08Xh  resp=%02b",
                 $time/1000, addr, data, resp);
    endtask

    // レジスタ読み出し
    task automatic axil_read(
        input  logic [31:0] addr,
        output logic [31:0] rdata,
        output logic [1:0]  rresp
    );
        axi_transaction rd_trans;
        axi_transaction rd_resp;

        rd_trans = axil_agent.rd_driver.create_transaction("rd");
        rd_trans.set_read_cmd(addr, XIL_AXI_BURST_TYPE_FIXED,
                              0, 0, XIL_AXI_SIZE_4BYTE);
        axil_agent.rd_driver.send(rd_trans);
        axil_agent.rd_driver.wait_rsp(rd_resp);
        rdata = rd_resp.get_data_beat(0);
        rresp = rd_resp.get_rresp(0);

        $display("[%6t ns]  AXIL RD  addr=%08Xh  data=%08Xh  rresp=%02b",
                 $time/1000, addr, rdata, rresp);
    endtask

    // ==========================================================
    // センサモデル（Q7 タイミングチャート準拠）
    //
    //   MES High  : CLK を High で維持（S_IDLE/S_MEAS と同期）
    //   MES 立ち下がり : D0 (LSB) を SOUT に出力
    //   CLK 立ち下がり : 現ビットが DUT にサンプルされる
    //   CLK 立ち上がり : 次ビットを SOUT にセット（D1, D2 ...）
    // ==========================================================
    // 各 ch のテストデータ: ch_n = (n+1) × 0x0111111
    logic [DATA_BITS-1:0] fake_data [NUM_CH];
    logic [DATA_BITS-1:0] shift     [NUM_CH];
    logic                 tx_active = 1'b0;

    initial begin
        for (int i = 0; i < NUM_CH; i++)
            fake_data[i] = (i[30:0] + 1) * 31'h011_1111;
        sens_sout  = '0;
    end

    // MES 立ち下がり → D0 出力・シフトレジスタロード
    always @(negedge sens_meas) begin
        #2ns;   // センサ伝搬遅延模擬
        for (int i = 0; i < NUM_CH; i++) begin
            sens_sout[i] = fake_data[i][0];
            shift[i]     = {1'b0, fake_data[i][DATA_BITS-1:1]};  // D1〜D30
        end
        tx_active = 1'b1;
    end

    // CLK 立ち上がり → 次ビットを SOUT へ
    always @(posedge sens_clk) begin
        if (tx_active) begin
            #2ns;
            for (int i = 0; i < NUM_CH; i++) begin
                sens_sout[i] = shift[i][0];
                shift[i]     = {1'b0, shift[i][DATA_BITS-1:1]};
            end
        end
    end

    // MES 立ち上がり → 送信リセット（次サイクル準備）
    always @(posedge sens_meas) tx_active = 1'b0;

    // ==========================================================
    // AXI-Stream パケット受信キュー（信号レベルモニタ）
    //
    //   axis_agent が m_axis_tready を常時 1 に駆動するため、
    //   tvalid & tready のハンドシェイクでビートをキャプチャ。
    // ==========================================================
    typedef struct packed {
        logic [63:0] ts;
        logic [30:0] ch [NUM_CH];
    } pkt_t;

    pkt_t rx_queue[$];
    int   rx_cnt = 0;

    // 受信中作業バッファ
    pkt_t  cur;
    int    beat_idx = 0;

    always_ff @(posedge clk) begin
        if (rst_n && m_axis_tvalid && m_axis_tready) begin
            case (beat_idx)
                0: begin cur.ts[63:32] = m_axis_tdata; beat_idx <= 1; end
                1: begin cur.ts[31:0]  = m_axis_tdata; beat_idx <= 2; end
                default: begin
                    // beat 2〜7: チャンネルデータ
                    cur.ch[beat_idx-2] = m_axis_tdata[30:0];

                    // TLAST チェック
                    if (beat_idx == TOTAL_BEATS-1) begin
                        if (!m_axis_tlast)
                            $error("[MON] beat%0d: TLAST expected but absent", beat_idx);
                        rx_queue.push_back(cur);
                        rx_cnt   <= rx_cnt + 1;
                        beat_idx <= 0;
                    end else begin
                        if (m_axis_tlast)
                            $error("[MON] beat%0d: premature TLAST", beat_idx);
                        beat_idx <= beat_idx + 1;
                    end
                end
            endcase
        end
    end

    // ==========================================================
    // 検証ユーティリティ
    // ==========================================================

    // n パケット到着を待つ
    task automatic wait_pkts(input int n, input string ctx);
        automatic int tgt     = rx_cnt + n;
        automatic int timeout = 0;
        while (rx_cnt < tgt) begin
            @(posedge clk);
            if (++timeout > TIMEOUT_CYC)
                $fatal(1, "[TIMEOUT] wait_pkts(%0d) @ %s", n, ctx);
        end
    endtask

    // パケット内容検証
    task automatic verify_pkt(input string label);
        pkt_t p;
        int   fails = 0;

        if (rx_queue.size() == 0) begin
            $error("[%s] queue is empty", label);
            return;
        end
        p = rx_queue.pop_front();

        $display("  ┌─ %s ─────────────────────────────", label);
        $display("  │  TS = %016Xh", p.ts);

        for (int ch = 0; ch < NUM_CH; ch++) begin
            logic [30:0] exp_val = fake_data[ch];
            if (p.ch[ch] === exp_val) begin
                $display("  │  ch%0d = %07Xh  [PASS]", ch, p.ch[ch]);
            end else begin
                $error("  │  ch%0d = %07Xh  [FAIL] exp=%07Xh",
                       ch, p.ch[ch], exp_val);
                fails++;
            end
        end

        if (fails == 0)
            $display("  └─ ALL PASS ────────────────────────────");
        else
            $display("  └─ %0d FAIL(s) ─────────────────────────", fails);
    endtask

    // STATUS レジスタ検証（xはドントケア）
    task automatic check_status(
        input string label,
        input logic  exp_enabled,   // 'x' でドントケア
        input logic  exp_busy       // 'x' でドントケア
    );
        logic [31:0] rdata;
        logic [1:0]  rresp;
        axil_read(ADDR_STATUS, rdata, rresp);

        if (rresp !== 2'b00)
            $error("[%s] STATUS rresp=%02b (exp 00)", label, rresp);

        if (exp_enabled !== 1'bx && rdata[1] !== exp_enabled)
            $error("[%s] enabled: got=%0b exp=%0b", label, rdata[1], exp_enabled);
        else if (exp_enabled !== 1'bx)
            $display("  enabled=%0b  [PASS]", rdata[1]);

        if (exp_busy !== 1'bx && rdata[0] !== exp_busy)
            $error("[%s] busy: got=%0b exp=%0b", label, rdata[0], exp_busy);
        else if (exp_busy !== 1'bx)
            $display("  busy=%0b  [PASS]", rdata[0]);
    endtask

    // ==========================================================
    // メイン テストシーケンス
    // ==========================================================
    initial begin : main_test
        logic [31:0] rdata;
        logic [1:0]  wresp, rresp;

        // ---- VIP エージェント初期化 ----
        // ★ BD インスタンス名が変わった場合はここを変更
        axil_agent = new("axil_agent",
            tb_ds_h3bp1_vip.u_bd.axil_mst_0.inst.IF);
        axil_agent.start_master();

        axis_agent = new("axis_agent",
            tb_ds_h3bp1_vip.u_bd.axis_slv_0.inst.IF);
        // Slave VIP: tready を常時 High（バックプレッシャーなし）
        axis_agent.start_slave();

        // ---- リセット ----
        do_reset(20);

        // ============================================================
        // TC1: リセット直後 STATUS 確認
        // ============================================================
        $display("\n══ TC1: Reset 後 STATUS 確認 ═════════════════════");
        check_status("TC1", 1'b0, 1'b0);

        // ============================================================
        // TC2: CTRL enable=1 書き込み & readback
        // ============================================================
        $display("\n══ TC2: enable=1 書き込み ════════════════════════");
        axil_write(ADDR_CTRL, 32'h0000_0001, wresp);
        if (wresp !== 2'b00)
            $error("[TC2] write resp=%02b", wresp);

        axil_read(ADDR_CTRL, rdata, rresp);
        if (rdata[0] !== 1'b1)
            $error("[TC2] CTRL readback: exp=1, got=%0b", rdata[0]);
        else
            $display("  CTRL[0] readback=1  [PASS]");

        repeat (10) @(posedge clk);
        check_status("TC2-after-enable", 1'b1, 1'bx);

        // ============================================================
        // TC3: 1パケット目 受信・検証
        // ============================================================
        $display("\n══ TC3: パケット#1 検証 ══════════════════════════");
        wait_pkts(1, "TC3");
        verify_pkt("TC3-Pkt1");

        // ============================================================
        // TC4: enable=0 → 安全停止確認（現測定シーケンス完結後）
        // ============================================================
        $display("\n══ TC4: enable=0 安全停止 ════════════════════════");
        axil_write(ADDR_CTRL, 32'h0000_0000, wresp);

        // 最大1測定サイクル分待つ
        begin
            automatic int clks_per_us = CLK_FREQ_HZ / 1_000_000;
            automatic int max_wait =
                (MEAS_TIME_US + DATA_BITS * 2 * SENSOR_CLK_US + INTER_MEAS_US + 20)
                * clks_per_us;
            repeat (max_wait) @(posedge clk);
        end

        check_status("TC4-stopped", 1'b0, 1'b0);

        // ============================================================
        // TC5: 停止中 sens_meas / sens_clk = 0 確認（1000 サイクル）
        // ============================================================
        $display("\n══ TC5: 停止中 sens_meas/clk 監視 ════════════════");
        begin
            automatic int errs = 0;
            repeat (1000) begin
                @(posedge clk);
                if (sens_meas === 1'b1 || sens_clk === 1'b1) errs++;
            end
            if (errs == 0)
                $display("  [PASS] sens_meas & sens_clk stayed LOW (1000 cyc)");
            else
                $error("  [FAIL] errs=%0d", errs);
        end

        // ============================================================
        // TC6: enable=1 再開 → パケット#2 検証
        // ============================================================
        $display("\n══ TC6: 再開 → パケット#2 検証 ═══════════════════");
        axil_write(ADDR_CTRL, 32'h0000_0001, wresp);
        wait_pkts(1, "TC6");
        verify_pkt("TC6-Pkt2");

        // ============================================================
        // TC7: 無効アドレス読み出し → 0xDEAD_BEEF
        // ============================================================
        $display("\n══ TC7: 無効アドレス読み出し ═════════════════════");
        axil_read(ADDR_RSVD, rdata, rresp);
        if (rdata === 32'hDEAD_BEEF)
            $display("  [PASS] Got 0xDEAD_BEEF");
        else
            $error("  [FAIL] Got 0x%08X (exp 0xDEAD_BEEF)", rdata);

        // ============================================================
        // TC8: 連続2パケット TLAST 境界確認
        // ============================================================
        $display("\n══ TC8: 連続2パケット TLAST 境界確認 ══════════════");
        wait_pkts(2, "TC8");
        verify_pkt("TC8-Pkt3");
        verify_pkt("TC8-Pkt4");

        // ============================================================
        // 終了
        // ============================================================
        repeat (20) @(posedge clk);
        $display("\n╔═════════════════════════════════════════╗");
        $display("║      All Test Cases Completed           ║");
        $display("╚═════════════════════════════════════════╝");
        $finish;
    end

    // ==========================================================
    // Block Design インスタンス（VIP を収容）
    // AXI-Lite ポートを DUT へ接続
    // AXI-Stream ポートを DUT へ接続
    // ==========================================================
    ds_h3bp1_bd_wrapper u_bd (
        // クロック・リセット（両 VIP 共通）
        .aclk_0      (clk),
        .aresetn_0   (rst_n),
        .aclk_1      (clk),
        .aresetn_1   (rst_n),

        // AXI-Lite Master VIP → DUT s_axil_*
        .M_AXI_0_awaddr  (s_axil_awaddr),
        .M_AXI_0_awvalid (s_axil_awvalid),
        .M_AXI_0_awready (s_axil_awready),
        .M_AXI_0_wdata   (s_axil_wdata),
        .M_AXI_0_wstrb   (s_axil_wstrb),
        .M_AXI_0_wvalid  (s_axil_wvalid),
        .M_AXI_0_wready  (s_axil_wready),
        .M_AXI_0_bresp   (s_axil_bresp),
        .M_AXI_0_bvalid  (s_axil_bvalid),
        .M_AXI_0_bready  (s_axil_bready),
        .M_AXI_0_araddr  (s_axil_araddr),
        .M_AXI_0_arvalid (s_axil_arvalid),
        .M_AXI_0_arready (s_axil_arready),
        .M_AXI_0_rdata   (s_axil_rdata),
        .M_AXI_0_rresp   (s_axil_rresp),
        .M_AXI_0_rvalid  (s_axil_rvalid),
        .M_AXI_0_rready  (s_axil_rready),

        // AXI-Stream Slave VIP ← DUT m_axis_*
        .S_AXIS_0_tdata  (m_axis_tdata),
        .S_AXIS_0_tvalid (m_axis_tvalid),
        .S_AXIS_0_tlast  (m_axis_tlast),
        .S_AXIS_0_tready (m_axis_tready)
    );

    // ==========================================================
    // グローバル タイムアウト ウォッチドッグ
    // ==========================================================
    initial begin
        #200ms;
        $error("[WATCHDOG] Global timeout!");
        $finish;
    end

    // ==========================================================
    // Waveform ダンプ（xsim）
    // ==========================================================
    initial begin
        $dumpfile("tb_ds_h3bp1_vip.vcd");
        $dumpvars(0, tb_ds_h3bp1_vip);
    end

endmodule
