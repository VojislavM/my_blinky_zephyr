PROJ_DIR 		:= ./
PROJECT_NAME    := my_blinky

.PHONY: all clean_all help

# Build project
all: 
	@echo file build
	sudo docker run --name ncs --rm -v ${PWD}:/workdir/ncs/$(PROJECT_NAME) ncs /bin/bash -c 'cd ncs/$(PROJECT_NAME); west build -p always -b nrf9160_pca10090ns'
	
clean_all:
	sudo rm -rf ./build
	
# Flash the program
flash:
	@echo Flashing: ./build/zephyr/merged.hex
	nrfjprog -f nrf91 --program ./build/zephyr/merged.hex --sectoranduicrerase
	nrfjprog -f nrf91 --reset
	
erase:
	nrfjprog -f nrf91 --eraseall
	
help:
	@echo project name: $(PROJECT_NAME)
	@echo project dir: $(PROJ_DIR)
	@echo project path:
	pwd $(PROJ_DIR)
	@echo following targets are available:
	@echo		all		   - build project
	@echo		flash      - flashing binary
	@echo       erase      - erase flash on targets
