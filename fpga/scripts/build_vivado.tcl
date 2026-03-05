# =============================================================================
# build_vivado.tcl — Vivado batch build for NS-RAM Bridge on Arty A7-100T
# =============================================================================
# Usage: vivado -mode batch -source build_vivado.tcl
# Target: xc7a100tcsg324-1 (Arty A7-100T)
# =============================================================================

set proj_dir  [file normalize [file dirname [info script]]/..]
set rtl_dir   $proj_dir/rtl
set xdc_dir   $proj_dir/constraints
set out_dir   $proj_dir/output

file mkdir $out_dir

# -------------------------------------------------------------------------
# Create in-memory project (no project directory clutter)
# -------------------------------------------------------------------------
create_project -in_memory -part xc7a100tcsg324-1

# -------------------------------------------------------------------------
# Add RTL sources
# -------------------------------------------------------------------------
read_verilog [glob $rtl_dir/avalanche_model.v]
read_verilog [glob $rtl_dir/lif_membrane.v]
read_verilog [glob $rtl_dir/neuron_scheduler.v]
read_verilog [glob $rtl_dir/nsram_neuron_bank.v]
read_verilog [glob $rtl_dir/nsram_bridge_top.v]
read_verilog [glob $rtl_dir/uart_rx.v]
read_verilog [glob $rtl_dir/uart_tx.v]
# Note: cmd_decoder.v is standalone reference, not instantiated by top

# -------------------------------------------------------------------------
# Add constraints
# -------------------------------------------------------------------------
read_xdc $xdc_dir/arty_a7.xdc

# -------------------------------------------------------------------------
# Add memory initialization file for exp LUT
# -------------------------------------------------------------------------
add_files $rtl_dir/exp_lut.hex
set_property file_type {Memory Initialization Files} [get_files exp_lut.hex]
add_files $rtl_dir/recip_vt_lut.hex
set_property file_type {Memory Initialization Files} [get_files recip_vt_lut.hex]

# -------------------------------------------------------------------------
# Synthesis
# -------------------------------------------------------------------------
puts "================================================================"
puts "  SYNTHESIS — nsram_bridge_top for xc7a100tcsg324-1"
puts "================================================================"

synth_design -top nsram_bridge_top -part xc7a100tcsg324-1 \
    -flatten_hierarchy rebuilt \
    -retiming

# Post-synthesis reports
report_utilization -file $out_dir/post_synth_utilization.rpt
report_timing_summary -file $out_dir/post_synth_timing.rpt

# Check for critical warnings
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

# Post-placement reports
report_utilization -file $out_dir/post_place_utilization.rpt
report_timing_summary -file $out_dir/post_place_timing.rpt

# -------------------------------------------------------------------------
# Physical optimization (optional — helps timing closure)
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

report_utilization -file $out_dir/post_route_utilization.rpt
report_timing_summary -file $out_dir/post_route_timing.rpt
report_power -file $out_dir/post_route_power.rpt
report_drc -file $out_dir/post_route_drc.rpt
report_methodology -file $out_dir/post_route_methodology.rpt

# Print timing summary to stdout
report_timing_summary

# Check timing
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "================================================================"
puts "  Worst Negative Slack (WNS): $wns ns"
puts "================================================================"

if {$wns < 0} {
    puts "WARNING: Timing NOT met! WNS = $wns ns"
    puts "Consider reducing clk_phy frequency or adding pipeline stages."
} else {
    puts "Timing MET. WNS = $wns ns"
}

# -------------------------------------------------------------------------
# Bitstream generation
# -------------------------------------------------------------------------
puts "================================================================"
puts "  BITSTREAM GENERATION"
puts "================================================================"

write_bitstream -force $out_dir/nsram_bridge_top.bit

puts "================================================================"
puts "  BUILD COMPLETE"
puts "  Bitstream: $out_dir/nsram_bridge_top.bit"
puts "================================================================"
