%.jed: %.pld
	@echo -n Compiling $< into $@...
	@galette $< 
	@echo done.

all: Type103PLD.jed \
	Type103aPLD.jed \
	Type103aCounter.jed \
	SRAMCardDecoder.jed \
	BusDecoder.jed \
	CPUDimmChip1.jed \
	CPUDimmChip2.jed \
	1MEGDecoderCard.jed \
	OperationDetect.jed \
	BusDecoder32Bit.jed \
	Flash32Decoder.jed \
	SRAM32Decoder.jed \
	NarrowWideAdapterDetect.jed \
	WideAuxDecoder.jed \
	RAM2DecoderChip.jed \
	ReadySynch.jed \
	TransactionMachine.jed \
	StateMachine0.jed \
	NewEBIDecoder.jed \
	EBIMemoryAccessorV2.jed \
	NewIntegratedChip2.jed \
	NewIntegratedChip1.jed \
	NewSRAMDecoderChip1.jed \
	NewSRAMDecoderChip2.jed \
	NewBasicHardwareChip.jed \
	AddressDecoder.jed \
	InterconnectV0.jed \
	NewCPUCardDecoder.jed \
	SODIUM_SRAMCard1MB.jed \
	RPiCH351Interface.jed

install_RPiCH351Interface: RPiCH351Interface.jed
	minipro -p "ATF22V10CQZ(UES)" --write RPiCH351Interface.jed 
install_SODIUM_SRAMCard1MB: SODIUM_SRAMCard1MB.jed
	minipro -p ATF16V8C --write SODIUM_SRAMCard1MB.jed
install_NewCPUCardDecoder: NewCPUCardDecoder.jed
	minipro -p ATF16V8C --write NewCPUCardDecoder.jed

install_InterconnectV0: InterconnectV0.jed
	minipro -p "ATF22V10CQZ(UES)" --write InterconnectV0.jed 

install_Type103: Type103PLD.jed
	minipro -p ATF16V8C --write Type103PLD.jed 

install_Type103a: Type103aPLD.jed
	minipro -p ATF16V8C --write Type103aPLD.jed 

install_SRAMCardDecoder: SRAMCardDecoder.jed
	minipro -p ATF16V8C --write SRAMCardDecoder.jed 

install_Type103aCounter: Type103aCounter.jed
	minipro -p "ATF22V10CQZ(UES)" --write Type103aCounter.jed 

install_BusDecoder: BusDecoder.jed
	minipro -p ATF16V8B --write BusDecoder.jed 

install_CPUDimmChip1: CPUDimmChip1.jed
	minipro -p ATF16V8B --write CPUDimmChip1.jed 

install_CPUDimmChip2: CPUDimmChip2.jed
	minipro -p ATF16V8B --write CPUDimmChip2.jed 

install_CPUDimmChip1_revC: CPUDimmChip1.jed
	minipro -p ATF16V8C --write CPUDimmChip1.jed 

install_CPUDimmChip2_revC: CPUDimmChip2.jed
	minipro -p ATF16V8C --write CPUDimmChip2.jed 

install_1MEGDecoderCard: 1MEGDecoderCard.jed
	minipro -p ATF16V8C --write 1MEGDecoderCard.jed 

install_OperationDetect: OperationDetect.jed
	minipro -p ATF16V8C --write OperationDetect.jed 

install_BusDecoder32bit: BusDecoder32Bit.jed
	minipro -p ATF16V8B --write BusDecoder32Bit.jed 
	
install_Flash32Decoder: Flash32Decoder.jed
	minipro -p ATF16V8B --write Flash32Decoder.jed 

install_SRAM32Decoder: SRAM32Decoder.jed
	minipro -p ATF16V8B --write SRAM32Decoder.jed 

install_NarrowWideAdapterDetect: NarrowWideAdapterDetect.jed
	minipro -p ATF16V8B --write NarrowWideAdapterDetect.jed 

install_WideAuxDecoder: WideAuxDecoder.jed 
	minipro -p ATF16V8B --write WideAuxDecoder.jed 

install_RAM2DecoderChip: RAM2DecoderChip.jed
	minipro -p "ATF22V10CQ(UES)" --write RAM2DecoderChip.jed 

install_ReadySynch: ReadySynch.jed
	minipro -p "ATF22V10CQ(UES)" --write ReadySynch.jed 

install_TransactionMachine: TransactionMachine.jed
	minipro -p "ATF22V10CQ(UES)" --write TransactionMachine.jed 

install_StateMachine0: StateMachine0.jed
	minipro -p "ATF22V10CQ(UES)" --write StateMachine0.jed 

install_NewEBIDecoder: NewEBIDecoder.jed
	minipro -p "ATF22V10CQ(UES)" --write NewEBIDecoder.jed 

install_EBIMemoryAccessorV2: EBIMemoryAccessorV2.jed
	minipro -p "ATF22V10CQ(UES)" --write EBIMemoryAccessorV2.jed 

install_NewIntegratedChip2: NewIntegratedChip2.jed
	minipro -p "ATF22V10CQ(UES)" --write NewIntegratedChip2.jed 

install_NewIntegratedChip1: NewIntegratedChip1.jed
	minipro -p ATF16V8C --write NewIntegratedChip1.jed 

install_NewSRAMDecoderChip1: NewSRAMDecoderChip1.jed
	minipro -p "ATF22V10CQ(UES)" --write NewSRAMDecoderChip1.jed 

install_NewSRAMDecoderChip2: NewSRAMDecoderChip2.jed
	minipro -p "ATF22V10CQ(UES)" --write NewSRAMDecoderChip2.jed 

install_NewBasicHardwareChip: NewBasicHardwareChip.jed
	minipro -p "ATF22V10CQ(UES)" --write NewBasicHardwareChip.jed

install_AddressDecoder: AddressDecoder.jed
	minipro -p ATF16V8C --write AddressDecoder.jed 


clean:
	@echo -n "Cleaning..."
	@rm -rf *.chp *.fus *.jed *.pin
	@echo "done"

.PHONY: help clean
