# No build step -- this is Python and an Arduino sketch. These are the things
# you actually do with the repo. The first two touch hardware; the rest do not.

# Prefer the repo venv if one exists. The system pythons on this machine are a
# mess -- /usr/local/bin/python3 is an x86_64 build with no numpy, and Apple's
# 3.9 launches x86_64 against an arm64 numpy. Create the venv with:
#     uv venv .venv && uv pip install --python .venv/bin/python numpy pyserial
PY ?= $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)
FQBN ?= arduino:avr:mega        # CS on pin 53 is Mega/Due specific

.PHONY: run status replay arduino ports check scope list help
.DEFAULT_GOAL := help

help:                ## show this
	@grep -hE '^[a-z-]+:.*##' $(MAKEFILE_LIST) \
	  | sed 's/:.*##/\t/' | awk -F'\t' '{printf "  make %-8s %s\n", $$1, $$2}'
	@echo
	@echo
	@$(PY) harness.py --list

run:                 ## ON HARDWARE: preflight the board, then run a controller (make run V=eta)
	@$(PY) bench.py $(V) $(if $(PORT),--port $(PORT))

status:              ## ON HARDWARE: graded sensor/coil census + the MIMO gate (make status W=sensors|coils|phi|pulse)
	@$(PY) status.py $(or $(W),sensors) $(if $(PORT),--port $(PORT))

replay:              ## the same census offline against a recorded log (make replay F=data/x.csv W=sensors)
	@$(PY) status.py $(or $(W),sensors) --replay $(F)

arduino:             ## ON HARDWARE: compile arduino.ino and upload it (make arduino FQBN=...)
	@$(PY) bench.py --flash $(if $(PORT),--port $(PORT)) --fqbn $(FQBN)

ports:               ## which serial port is the board on
	@$(PY) bench.py --ports

check:               ## the same suite headless across every version (CI form)
	@$(PY) harness.py --test all

scope:               ## 8-channel scope in the browser: the real board, or the live run's CSV
	@$(PY) scope.py $(if $(PORT),--port $(PORT)) $(if $(HTTP),--http $(HTTP))

list:                ## what versions exist and what each one changes
	@$(PY) harness.py --list
