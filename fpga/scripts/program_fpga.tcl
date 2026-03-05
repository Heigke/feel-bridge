# =============================================================================
# program_fpga.tcl — Program Arty A7-100T with NS-RAM bridge bitstream
# =============================================================================
# Usage: vivado -mode batch -source program_fpga.tcl
# =============================================================================

set proj_dir [file normalize [file dirname [info script]]/..]
set bit_file $proj_dir/output/nsram_bridge_top.bit

puts "================================================================"
puts "  Programming FPGA with: $bit_file"
puts "================================================================"

# Open hardware manager
open_hw_manager

# Connect to hw_server (local)
connect_hw_server -allow_non_jtag

# Open the JTAG target — auto-detect
open_hw_target

# Get the device (should be xc7a100t)
set device [lindex [get_hw_devices] 0]
puts "Found device: $device"

# Set the bitstream
set_property PROGRAM.FILE $bit_file $device

# Program
program_hw_devices $device

puts "================================================================"
puts "  FPGA programmed successfully!"
puts "================================================================"

# Close
close_hw_target
disconnect_hw_server
close_hw_manager
