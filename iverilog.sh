# SPDX-License-Identifier: MIT
#!/bin/bash

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}  MMCM/PLL Clocking Wizard Simulation${NC}"
echo -e "${BLUE}=======================================${NC}"

# Compile with iverilog
echo -e "${GREEN}Compiling with iverilog...${NC}"
iverilog -g2012 \
    -o a.vvp \
    -s tb_clk_wiz_0 \
    mmcmpll_sim_core.v \
    clk_wiz_sim.v

if [ $? -ne 0 ]; then
    echo -e "${RED}Compilation failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Compilation successful!${NC}"

# Run simulation
echo -e "${GREEN}Running simulation...${NC}"
vvp -n a.vvp | tee log.txt

if [ $? -ne 0 ]; then
    echo -e "${RED}Simulation failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Simulation completed!${NC}"

# Check if VCD file was generated
if [ -f sim.vcd ]; then
    echo -e "${GREEN}VCD waveform file generated: clk_wiz_sim.vcd${NC}"
    echo -e "${BLUE}To view waveforms, run: gtkwave clk_wiz_sim.vcd${NC}"
    # setsid surfer sim.vcd &
else
    echo -e "${RED}Warning: VCD file not generated${NC}"
fi

echo -e "${BLUE}=======================================${NC}"
echo -e "${GREEN}Done!${NC}"
echo -e "${BLUE}=======================================${NC}"


