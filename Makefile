# No build step -- this is Python and an Arduino sketch. These are the four
# things you actually do with the repo.

PY ?= python3

.PHONY: test check sim list help
.DEFAULT_GOAL := help

help:                ## show this
	@grep -hE '^[a-z-]+:.*##' $(MAKEFILE_LIST) \
	  | sed 's/:.*##/\t/' | awk -F'\t' '{printf "  make %-8s %s\n", $$1, $$2}'
	@echo
	@echo
	@$(PY) harness.py --list

test:                ## interactive runner: pick a version, watch it damp, press x to kick
	@$(PY) test/tui.py

check:               ## the same suite headless across every version (CI form)
	@$(PY) harness.py --test all

sim:                 ## browser simulator on the newest version -- make sim V=v0 to pick
	@$(PY) harness.py $(V)

list:                ## what versions exist and what each one changes
	@$(PY) harness.py --list
