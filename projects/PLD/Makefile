%.jed: %.pld
	@echo -n Compiling $< into $@...
	@galette $< 
	@echo done.

all: NewCPUCardDecoder.jed \
	RPiCH351Interface.jed \
	ConnectionFunctionalityBoard.jed \
	CacheAndMemoryDecoder.jed

install_RPiCH351Interface: RPiCH351Interface.jed
	minipro -p "ATF22V10CQZ(UES)" --write $<
install_NewCPUCardDecoder: NewCPUCardDecoder.jed
	minipro -p ATF16V8C --write $< 
install_ConnectionFunctionalityBoard: ConnectionFunctionalityBoard.jed
	minipro -p "ATF22V10CQZ(UES)" --write $<

install_CacheAndMemoryDecoder: CacheAndMemoryDecoder.jed
	minipro -p "ATF22V10CQZ(UES)" --write $< 


clean:
	@echo -n "Cleaning..."
	@rm -rf *.chp *.fus *.jed *.pin
	@echo "done"

.PHONY: help clean
