SHELL := /bin/bash

#############################################
## .PHONY: install coverage test docs help
#############################################
.PHONY: build-all build-robot-control-library build-yamal-cpp clean help
.DEFAULT_GOAL := help

YAML_CPP = yaml-cpp
RCL = rcl
GTEST = googletest
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

build-all: build-robot-control-library build-yamal-cpp build-google-test ## build al ldependencies
	
build-robot-control-library: ## build robot control library
	@if [ ! -d "./build/${RCL}" ]; then	\
		mkdir -p ./build/${RCL};				\
		mkdir -p ./install/${RCL};				\
	fi
	@cmake -S librobotcontrol -B ./build/${RCL} -DCMAKE_INSTALL_PREFIX=./install/${RCL}
	@cd ./build/${RCL}; \
	make -j8

build-yamal-cpp: ## build yamal-cpp
	@if [ ! -d "./build/${YAML_CPP}" ]; then	\
		mkdir -p ./build/${YAML_CPP};				\
		mkdir -p ./install/${YAML_CPP};				\
	fi
	@cmake -S ${YAML_CPP} -B ./build/${YAML_CPP} -DCMAKE_INSTALL_PREFIX=./install/${YAML_CPP}-DCMAKE_INSTALL_PREFIX=./install/${YAML_CPP} -DYAML_BUILD_SHARED_LIBS=OFF
	@cd ./build/${YAML_CPP}; \
	make -j8 && make install

build-google-test: ## build google-test
	@if [ ! -d "./build/${GTEST}" ]; then	\
		mkdir -p ./build/${GTEST};				\
	fi
	@cmake -S ${GTEST} -B ./build/${GTEST}
	@cd ./build/${GTEST}; \
	make -j8

clean: ## clean build file
	@if [ -d "./build" ]; then	\
		rm -rf ./build;			\
	fi
	@if [ -d "./install" ]; then	\
		rm -rf ./install;			\
	fi