# MINIMIG-DE0_NANO
# fpga makefile
# 2012, rok.krajnc@gmail.com
# 2015, Stefan Kristiansson


### programs ###
MAP=quartus_map
FIT=quartus_fit
ASM=quartus_asm


### project ###
PROJECT=minimig_de0_nano

### build rules ###

# all
all:
	@echo Making FPGA programming files ...
	@make map
	@make fit
	@make asm

map:
	@echo Running mapper ...
	@$(MAP) $(PROJECT)

fit:
	@echo Running fitter ...
	@$(FIT) $(PROJECT)

asm:
	@echo Running assembler ...
	@$(ASM) $(PROJECT)


# clean
clean:
	@echo clean
	@rm -rf ./out/
	@rm -rf ./db/
	@rm -rf ./incremental_db/
