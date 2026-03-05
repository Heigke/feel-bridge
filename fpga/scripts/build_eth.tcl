# =============================================================================
# build_eth.tcl — Vivado batch build for NS-RAM Ethernet Bridge on Arty A7-100T
# =============================================================================
# Usage: vivado -mode batch -source fpga/scripts/build_eth.tcl
# Target: xc7a100tcsg324-1 (Arty A7-100T)
# =============================================================================

set proj_dir  [file normalize [file dirname [info script]]/..]
set rtl_dir   $proj_dir/rtl
set xdc_dir   $proj_dir/constraints
set eth_dir   $proj_dir/ip/verilog-ethernet
set out_dir   $proj_dir/output

file mkdir $out_dir

# -------------------------------------------------------------------------
# Create in-memory project
# -------------------------------------------------------------------------
create_project -in_memory -part xc7a100tcsg324-1

# -------------------------------------------------------------------------
# NS-RAM RTL sources
# -------------------------------------------------------------------------
read_verilog $rtl_dir/nsram_eth_top.v
read_verilog $rtl_dir/avalanche_model.v
read_verilog $rtl_dir/lif_membrane.v
read_verilog $rtl_dir/neuron_scheduler.v
read_verilog $rtl_dir/nsram_neuron_bank.v
read_verilog $rtl_dir/uart_rx.v
read_verilog $rtl_dir/uart_tx.v

# -------------------------------------------------------------------------
# verilog-ethernet IP sources (from Arty example Makefile)
# -------------------------------------------------------------------------
read_verilog $eth_dir/rtl/ssio_sdr_in.v
read_verilog $eth_dir/rtl/mii_phy_if.v
read_verilog $eth_dir/rtl/eth_mac_mii_fifo.v
read_verilog $eth_dir/rtl/eth_mac_mii.v
read_verilog $eth_dir/rtl/eth_mac_1g.v
read_verilog $eth_dir/rtl/axis_gmii_rx.v
read_verilog $eth_dir/rtl/axis_gmii_tx.v
read_verilog $eth_dir/rtl/lfsr.v
read_verilog $eth_dir/rtl/eth_axis_rx.v
read_verilog $eth_dir/rtl/eth_axis_tx.v
read_verilog $eth_dir/rtl/udp_complete.v
read_verilog $eth_dir/rtl/udp_checksum_gen.v
read_verilog $eth_dir/rtl/udp.v
read_verilog $eth_dir/rtl/udp_ip_rx.v
read_verilog $eth_dir/rtl/udp_ip_tx.v
read_verilog $eth_dir/rtl/ip_complete.v
read_verilog $eth_dir/rtl/ip.v
read_verilog $eth_dir/rtl/ip_eth_rx.v
read_verilog $eth_dir/rtl/ip_eth_tx.v
read_verilog $eth_dir/rtl/ip_arb_mux.v
read_verilog $eth_dir/rtl/arp.v
read_verilog $eth_dir/rtl/arp_cache.v
read_verilog $eth_dir/rtl/arp_eth_rx.v
read_verilog $eth_dir/rtl/arp_eth_tx.v
read_verilog $eth_dir/rtl/eth_arb_mux.v
read_verilog $eth_dir/lib/axis/rtl/arbiter.v
read_verilog $eth_dir/lib/axis/rtl/priority_encoder.v
read_verilog $eth_dir/lib/axis/rtl/axis_fifo.v
read_verilog $eth_dir/lib/axis/rtl/axis_async_fifo.v
read_verilog $eth_dir/lib/axis/rtl/axis_async_fifo_adapter.v
read_verilog $eth_dir/lib/axis/rtl/sync_reset.v

# -------------------------------------------------------------------------
# Constraints
# -------------------------------------------------------------------------
read_xdc $xdc_dir/arty_a7.xdc

# verilog-ethernet timing constraints (CDC for MAC FIFOs, MII PHY, etc.)
read_xdc $eth_dir/syn/vivado/mii_phy_if.tcl
read_xdc $eth_dir/syn/vivado/eth_mac_fifo.tcl
read_xdc $eth_dir/lib/axis/syn/vivado/axis_async_fifo.tcl
read_xdc $eth_dir/lib/axis/syn/vivado/sync_reset.tcl

# -------------------------------------------------------------------------
# Memory init files for exp/recip LUTs
# -------------------------------------------------------------------------
add_files $rtl_dir/exp_lut.hex
set_property file_type {Memory Initialization Files} [get_files exp_lut.hex]
add_files $rtl_dir/recip_vt_lut.hex
set_property file_type {Memory Initialization Files} [get_files recip_vt_lut.hex]

# -------------------------------------------------------------------------
# Synthesis
# -------------------------------------------------------------------------
puts "================================================================"
puts "  SYNTHESIS — nsram_eth_top for xc7a100tcsg324-1"
puts "================================================================"

synth_design -top nsram_eth_top -part xc7a100tcsg324-1 \
    -flatten_hierarchy rebuilt

report_utilization -file $out_dir/post_synth_utilization_eth.rpt
report_timing_summary -file $out_dir/post_synth_timing_eth.rpt

set crit_warns [get_msg_config -count -severity {CRITICAL WARNING}]
puts "Post-synth critical warnings: $crit_warns"

# -------------------------------------------------------------------------
# Optimization
# -------------------------------------------------------------------------
puts "================================================================"
puts "  OPTIMIZATION"
puts "================================================================"
opt_design

# -------------------------------------------------------------------------
# Placement
# -------------------------------------------------------------------------
puts "================================================================"
puts "  PLACEMENT"
puts "================================================================"
place_design
report_utilization -file $out_dir/post_place_utilization_eth.rpt
report_timing_summary -file $out_dir/post_place_timing_eth.rpt

# -------------------------------------------------------------------------
# Physical optimization
# -------------------------------------------------------------------------
phys_opt_design

# -------------------------------------------------------------------------
# Routing
# -------------------------------------------------------------------------
puts "================================================================"
puts "  ROUTING"
puts "================================================================"
route_design

# -------------------------------------------------------------------------
# Post-implementation reports
# -------------------------------------------------------------------------
puts "================================================================"
puts "  POST-IMPLEMENTATION REPORTS"
puts "================================================================"

report_utilization -file $out_dir/post_route_utilization_eth.rpt
report_timing_summary -file $out_dir/post_route_timing_eth.rpt
report_power -file $out_dir/post_route_power_eth.rpt
report_drc -file $out_dir/post_route_drc_eth.rpt
report_methodology -file $out_dir/post_route_methodology_eth.rpt
report_timing_summary

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "================================================================"
puts "  Worst Negative Slack (WNS): $wns ns"
puts "================================================================"

if {$wns < 0} {
    puts "WARNING: Timing NOT met! WNS = $wns ns"
} else {
    puts "Timing MET. WNS = $wns ns"
}

# -------------------------------------------------------------------------
# Bitstream generation
# -------------------------------------------------------------------------
puts "================================================================"
puts "  BITSTREAM GENERATION"
puts "================================================================"

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

write_bitstream -force $out_dir/nsram_eth_top.bit

puts "================================================================"
puts "  BUILD COMPLETE"
puts "  Bitstream: $out_dir/nsram_eth_top.bit"
puts "================================================================"
