# ==============================================================================
# create_bd.tcl
# DS-H3BP1 テストベンチ用 Vivado Block Design 自動生成スクリプト
#
# 実行方法:
#   vivado -mode batch -source create_bd.tcl
#   または Vivado Tcl Console: source create_bd.tcl
#
# 生成されるBD構成:
#   ds_h3bp1_bd
#   ├── axil_mst_0  : axi_vip        (AXI4-Lite Master, ADDR_WIDTH=4)
#   └── axis_slv_0  : axi4stream_vip (AXI4-Stream Slave, TDATA=32bit)
#
# TBからのアクセス例:
#   ds_h3bp1_bd_axil_mst_0_mst_t agent = new("...", tb.bd_i.axil_mst_0.inst.IF);
#   ds_h3bp1_bd_axis_slv_0_slv_t agent = new("...", tb.bd_i.axis_slv_0.inst.IF);
# ==============================================================================

set prj_name  "ds_h3bp1_tb_prj"
set prj_dir   [file join [file dirname [file normalize [info script]]] vivado]
set part      "xc7z020clg400-1"   ;# Zynq-7020 (環境に合わせて変更)
set bd_name   "ds_h3bp1_bd"
set src_dir   [file dirname [file normalize [info script]]]

# ------------------------------------------------------------------------------
# プロジェクト作成
# ------------------------------------------------------------------------------
create_project ${prj_name} ${prj_dir} -part ${part} -force
set_property target_language  SystemVerilog [current_project]
set_property simulator_language Mixed       [current_project]

# ------------------------------------------------------------------------------
# RTL ソース追加
# ------------------------------------------------------------------------------
add_files -norecurse [file join ${src_dir} ds_h3bp1_axi_stream.sv]
set_property file_type SystemVerilog [get_files ds_h3bp1_axi_stream.sv]

add_files -fileset sim_1 -norecurse [file join ${src_dir} tb_ds_h3bp1_vip.sv]
set_property file_type SystemVerilog [get_files tb_ds_h3bp1_vip.sv]

puts "INFO: RTL sources added."

# ------------------------------------------------------------------------------
# Block Design 作成
# ------------------------------------------------------------------------------
create_bd_design ${bd_name}
current_bd_design ${bd_name}

# ---- AXI4-Lite Master VIP ----
set axil_mst [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:axi_vip:1.1 axil_mst_0]
set_property -dict [list \
    CONFIG.INTERFACE_MODE  {MASTER}   \
    CONFIG.PROTOCOL        {AXI4LITE} \
    CONFIG.ADDR_WIDTH      {4}        \
    CONFIG.DATA_WIDTH      {32}       \
    CONFIG.HAS_BRESP       {1}        \
    CONFIG.HAS_RRESP       {1}        \
    CONFIG.HAS_WSTRB       {1}        \
] $axil_mst

# ---- AXI4-Stream Slave VIP ----
set axis_slv [create_bd_cell -type ip \
    -vlnv xilinx.com:ip:axi4stream_vip:1.1 axis_slv_0]
set_property -dict [list \
    CONFIG.INTERFACE_MODE  {SLAVE} \
    CONFIG.TDATA_NUM_BYTES {4}     \
    CONFIG.HAS_TLAST       {1}     \
    CONFIG.HAS_TREADY      {1}     \
] $axis_slv

# ---- 全ポートを External に引き出す ----
make_bd_intf_pins_external [get_bd_intf_pins axil_mst_0/M_AXI]
make_bd_intf_pins_external [get_bd_intf_pins axis_slv_0/S_AXIS]
make_bd_pins_external      [get_bd_pins axil_mst_0/aclk]
make_bd_pins_external      [get_bd_pins axil_mst_0/aresetn]
make_bd_pins_external      [get_bd_pins axis_slv_0/aclk]
make_bd_pins_external      [get_bd_pins axis_slv_0/aresetn]

validate_bd_design
save_bd_design
puts "INFO: Block Design [${bd_name}] created and validated."

# ------------------------------------------------------------------------------
# BD Wrapper 生成
# ------------------------------------------------------------------------------
set wrapper [make_wrapper -files [get_files ${bd_name}.bd] -top]
add_files -norecurse $wrapper
set_property top ${bd_name}_wrapper [get_filesets sources_1]
puts "INFO: Wrapper generated: $wrapper"

# ------------------------------------------------------------------------------
# シミュレーション設定
# ------------------------------------------------------------------------------
set_property top              tb_ds_h3bp1_vip [get_filesets sim_1]
set_property top_lib          xil_defaultlib    [get_filesets sim_1]

# xelab オプション: SV assertion 有効化
set_property -name {xsim.elaborate.xelab.more_options} \
    -value {-sv_root . -sv_lib dpi -assert} \
    [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "INFO: Simulation settings configured."
puts ""
puts "======================================================="
puts "  Done! Open Vivado GUI and:"
puts "  Flow -> Run Simulation -> Run Behavioral Simulation"
puts "  Or run: launch_simulation; run all"
puts "======================================================="
