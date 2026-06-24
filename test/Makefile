SIM          ?= icarus
TOPLEVEL_LANG ?= verilog

# Explicitly order: DUT first, then tb
VERILOG_SOURCES  = design.sv
VERILOG_SOURCES += tb.v

COMPILE_ARGS += -g2012
COMPILE_ARGS += -Ptt_um_luck_engine.ROLL_CYCLES=48 \
                -Ptt_um_luck_engine.TM_HALF_BIT_CYCLES=2 \
                -Ptt_um_luck_engine.CLICK_WINDOW_CYCLES=20
         
            
TOPLEVEL = tb
COCOTB_TEST_MODULE   = test

include $(shell cocotb-config --makefiles)/Makefile.sim
