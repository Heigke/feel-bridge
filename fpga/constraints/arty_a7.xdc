## Arty A7-100T Pin Constraints for NS-RAM FPGA Bridge
## Target: xc7a100tcsg324-1

## ---- Clock ----
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }];
create_clock -period 10.000 -name sys_clk [get_ports CLK100MHZ]

## ---- Clock domain crossing constraints ----
## CLKOUT0 = 125 MHz (Ethernet), CLKOUT1 = 25 MHz (PHY ref), CLKOUT2 = 10 MHz (physics)
## CDC handled via synchronizers in neuron bank and Ethernet bridge.
## 10 MHz (physics) ↔ 100 MHz (sys)
set_false_path -from [get_clocks sys_clk] -to [get_clocks -of_objects [get_pins u_mmcm/CLKOUT2]]
set_false_path -from [get_clocks -of_objects [get_pins u_mmcm/CLKOUT2]] -to [get_clocks sys_clk]

## ---- UART (USB-UART via FTDI) ----
set_property -dict { PACKAGE_PIN A9    IOSTANDARD LVCMOS33 } [get_ports { uart_txd_in }];
set_property -dict { PACKAGE_PIN D10   IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }];

## ---- Switches ----
set_property -dict { PACKAGE_PIN A8    IOSTANDARD LVCMOS33 } [get_ports { sw[0] }];
set_property -dict { PACKAGE_PIN C11   IOSTANDARD LVCMOS33 } [get_ports { sw[1] }];
set_property -dict { PACKAGE_PIN C10   IOSTANDARD LVCMOS33 } [get_ports { sw[2] }];
set_property -dict { PACKAGE_PIN A10   IOSTANDARD LVCMOS33 } [get_ports { sw[3] }];

## ---- Buttons ----
set_property -dict { PACKAGE_PIN D9    IOSTANDARD LVCMOS33 } [get_ports { btn[0] }];
set_property -dict { PACKAGE_PIN C9    IOSTANDARD LVCMOS33 } [get_ports { btn[1] }];
set_property -dict { PACKAGE_PIN B9    IOSTANDARD LVCMOS33 } [get_ports { btn[2] }];
set_property -dict { PACKAGE_PIN B8    IOSTANDARD LVCMOS33 } [get_ports { btn[3] }];

## ---- LEDs (active high) ----
set_property -dict { PACKAGE_PIN H5    IOSTANDARD LVCMOS33 } [get_ports { led[0] }];
set_property -dict { PACKAGE_PIN J5    IOSTANDARD LVCMOS33 } [get_ports { led[1] }];
set_property -dict { PACKAGE_PIN T9    IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
set_property -dict { PACKAGE_PIN T10   IOSTANDARD LVCMOS33 } [get_ports { led[3] }];

## ---- RGB LEDs ----
## led_rgb[11:0] = {led3_b, led3_g, led3_r, led2_b, led2_g, led2_r,
##                  led1_b, led1_g, led1_r, led0_b, led0_g, led0_r}

## RGB LED 0  (led_rgb[2:0])
set_property -dict { PACKAGE_PIN G6    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[0] }];  # led0_r
set_property -dict { PACKAGE_PIN F6    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[1] }];  # led0_g
set_property -dict { PACKAGE_PIN E1    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[2] }];  # led0_b

## RGB LED 1  (led_rgb[5:3])
set_property -dict { PACKAGE_PIN G3    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[3] }];  # led1_r
set_property -dict { PACKAGE_PIN J4    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[4] }];  # led1_g
set_property -dict { PACKAGE_PIN G4    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[5] }];  # led1_b

## RGB LED 2  (led_rgb[8:6])
set_property -dict { PACKAGE_PIN J3    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[6] }];  # led2_r
set_property -dict { PACKAGE_PIN J2    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[7] }];  # led2_g
set_property -dict { PACKAGE_PIN H4    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[8] }];  # led2_b

## RGB LED 3  (led_rgb[11:9])
set_property -dict { PACKAGE_PIN K1    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[9] }];  # led3_r
set_property -dict { PACKAGE_PIN H6    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[10] }]; # led3_g
set_property -dict { PACKAGE_PIN K2    IOSTANDARD LVCMOS33 } [get_ports { led_rgb[11] }]; # led3_b

## ---- Ethernet MII PHY (DP83848, 100BASE-T) ----
set_property -dict { PACKAGE_PIN G18   IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12 } [get_ports { phy_ref_clk }];
set_property -dict { PACKAGE_PIN C16   IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12 } [get_ports { phy_reset_n }];
set_property -dict { PACKAGE_PIN F15   IOSTANDARD LVCMOS33 } [get_ports { phy_rx_clk }];
set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[0] }];
set_property -dict { PACKAGE_PIN E17   IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[1] }];
set_property -dict { PACKAGE_PIN E18   IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[2] }];
set_property -dict { PACKAGE_PIN G17   IOSTANDARD LVCMOS33 } [get_ports { phy_rxd[3] }];
set_property -dict { PACKAGE_PIN G16   IOSTANDARD LVCMOS33 } [get_ports { phy_rx_dv }];
set_property -dict { PACKAGE_PIN C17   IOSTANDARD LVCMOS33 } [get_ports { phy_rx_er }];
set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports { phy_tx_clk }];
set_property -dict { PACKAGE_PIN H14   IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12 } [get_ports { phy_txd[0] }];
set_property -dict { PACKAGE_PIN J14   IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12 } [get_ports { phy_txd[1] }];
set_property -dict { PACKAGE_PIN J13   IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12 } [get_ports { phy_txd[2] }];
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12 } [get_ports { phy_txd[3] }];
set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12 } [get_ports { phy_tx_en }];
set_property -dict { PACKAGE_PIN D17   IOSTANDARD LVCMOS33 } [get_ports { phy_col }];
set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { phy_crs }];

## MII PHY clocks (25 MHz = 40ns period)
create_clock -period 40.000 -name phy_rx_clk [get_ports phy_rx_clk]
create_clock -period 40.000 -name phy_tx_clk [get_ports phy_tx_clk]

## Ethernet CDC constraints
set_false_path -to [get_ports { phy_ref_clk phy_reset_n }]
set_output_delay 0 [get_ports { phy_ref_clk phy_reset_n }]

## 125 MHz ↔ 100 MHz CDC
set_false_path -from [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]] -to [get_clocks sys_clk]
set_false_path -from [get_clocks sys_clk] -to [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]]

## 125 MHz ↔ 10 MHz CDC (Ethernet ↔ physics: all crossing via toggle-handshake synchronizers)
set_false_path -from [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]] -to [get_clocks -of_objects [get_pins u_mmcm/CLKOUT2]]
set_false_path -from [get_clocks -of_objects [get_pins u_mmcm/CLKOUT2]] -to [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]]
