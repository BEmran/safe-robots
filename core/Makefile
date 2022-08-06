SHELL := /bin/bash

#############################################
## .PHONY: install coverage test docs help
#############################################
.PHONY: build test clang-format-all cmake-format-all help
.DEFAULT_GOAL := help

################################
## Display option information
################################
define PRINT_HELP_PYSCRIPT
import re, sys

for line in sys.stdin:
	match = re.match(r'^([a-zA-Z_-]+):.*?## (.*)$$', line)
	if match:
		target, help = match.groups()
		print("%-20s %s" % (target, help))
endef
export PRINT_HELP_PYSCRIPT

###################
## Define Target
###################
help:
	@python3 -c "$$PRINT_HELP_PYSCRIPT" < $(MAKEFILE_LIST)

build: ## build target
	@rm -rf build
	@mkdir build
	@cmake -S . -B build
	@cd build; \
	make

clean: ## clean build file
	@if [ -d "./build" ]; then	\
		rm -rf build;			\
	fi

clang-format-all: ## clang format all source and header files
	utils/clang-format
	
cmake-format-all: ## cmake format all CMakeList.txt files
	utils/cmake-format

test: ## run tests with ctest
	@if [ ! -d "./build" ]; then    								\
		echo "Please build the project first using: make build";	\
	else															\
	cd build;														\
		ctest -VV;													\
	fi

