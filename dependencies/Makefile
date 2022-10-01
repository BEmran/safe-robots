SHELL := /bin/bash

#############################################
## .PHONY: install coverage test docs help
#############################################
.PHONY: build-all clean help
.DEFAULT_GOAL := help

YAML_CPP = yaml-cpp
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
	@if [ ! -d "./build/yaml-cpp" ]; then	\
		mkdir -p ./build/yaml-cpp;				\
		mkdir -p ./install/yaml-cpp;				\
	fi
	@cmake -S ${YAML_CPP} -B ./build/yaml-cpp -DCMAKE_INSTALL_PREFIX=./install/yaml-cpp -DYAML_BUILD_SHARED_LIBS=OFF
	@cd ./build/yaml-cpp; \
	make -j8 && make install

clean: ## clean build file
	@if [ -d "./build" ]; then	\
		rm -rf ./build;			\
	fi
	@if [ -d "./install" ]; then	\
		rm -rf ./install;			\
	fi