open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set device [lindex [get_hw_devices] 0]
current_hw_device $device
set_property PROGRAM.FILE {fpga/output/nsram_eth_top.bit} $device
program_hw_devices $device

puts "PROGRAMMING COMPLETE"
close_hw_manager
