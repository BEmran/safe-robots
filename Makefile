SHELL := /bin/bash

#############################################
## .PHONY: install coverage test docs help
#############################################
.PHONY: build-all build-all-release build-dependencies test test-debug clang-format-all cmake-format-all help
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

build-all: ## build target
	@if [ ! -d "./build" ]; then	\
		mkdir build;				\
	fi
	@cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
	@cd build; \
	make -j8

build-all-release: ## build target as reslease
	@if [ ! -d "./build" ]; then	\
		mkdir build;				\
	fi
	@cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
	@cd build; \
	make -j8
build-dependencies: ## build project dependencies
	@cd dependencies; \
	make build-all -j2

clean: ## clean build file
	@if [ -d "./build" ]; then	\
		rm -rf build;			\
	fi

clang-format-all: ## clang format all source and header files
	utils/clang-format

cmake-format-all: ## cmake format all CMakeList.txt files
	utils/cmake-format

test-debug: ## run tests with debug information
	@if [ ! -d "./build" ]; then    								\
		echo "Please build the project first using: make build";	\
	else															\
	cd build;														\
		ctest -VV;													\
	fi

test: ## run tests
	@if [ ! -d "./build" ]; then    								\
		echo "Please build the project first using: make build";	\
	else															\
	cd build;														\
		ctest --output-on-failure;									\
	fi