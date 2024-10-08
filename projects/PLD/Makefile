%.jed: %.pld
	@echo -n Compiling $< into $@...
	@galette $< 
	@echo done.

all: NewCPUCardDecoder.jed \
	RPiCH351Interface.jed \
	ConnectionFunctionalityBoard.jed

install_RPiCH351Interface: RPiCH351Interface.jed
	minipro -p "ATF22V10CQZ(UES)" --write RPiCH351Interface.jed 
install_NewCPUCardDecoder: NewCPUCardDecoder.jed
	minipro -p ATF16V8C --write NewCPUCardDecoder.jed
install_ConnectionFunctionalityBoard: ConnectionFunctionalityBoard.jed
	minipro -p "ATF22V10CQZ(UES)" --write ConnectionFunctionalityBoard.jed 


clean:
	@echo -n "Cleaning..."
	@rm -rf *.chp *.fus *.jed *.pin
	@echo "done"

.PHONY: help clean
