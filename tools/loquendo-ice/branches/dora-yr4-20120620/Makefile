#------------------------------------------------------------------------------#

SERVER_EXEC=lasr-ice
MONITOR_EXEC=lasr-monitor
CONTROL_EXEC=lasrctrl

BIN_DIR=bin

.PHONY: main
main: listener-monitor server control

.PHONY: server
server: $(BIN_DIR)/$(SERVER_EXEC)

.PHONY: listener-monitor
listener-monitor: $(BIN_DIR)/$(MONITOR_EXEC)

.PHONY: control
control: $(BIN_DIR)/$(CONTROL_EXEC)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

$(BIN_DIR)/$(SERVER_EXEC): compile-server
	cp src/server/$(SERVER_EXEC) $(BIN_DIR)

$(BIN_DIR)/$(MONITOR_EXEC): compile-listener-monitor
	cp src/listener-monitor/$(MONITOR_EXEC) $(BIN_DIR)

$(BIN_DIR)/$(CONTROL_EXEC): compile-control
	cp src/control/$(CONTROL_EXEC) $(BIN_DIR)

.PHONY: compile-server
compile-server:
	make -C src server

.PHONY: compile-listener-monitor
compile-listener-monitor:
	make -C src listener-monitor

.PHONY: compile-control
compile-control:
	make -C src control

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: clean
clean:
	make -C src clean
	rm -f $(BIN_DIR)/$(SERVER_EXEC) $(BIN_DIR)/$(MONITOR_EXEC) $(BIN_DIR)/$(CONTROL_EXEC)
