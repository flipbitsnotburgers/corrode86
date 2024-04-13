.PHONY: all clean build test run-all

# Default target
all: build

# Build the Rust project
build:
	cargo build --release

# Clean build artifacts and generated binaries
clean:
	cargo clean
	rm -f asm/*.bin

# Test with a specific ASM file
test: build
	./build_and_run.py asm/calc_add.asm

# Run all ASM files
run-all: build
	./build_and_run.py

# Run a specific ASM file (usage: make run FILE=asm/calc_add.asm)
run: build
	@if [ -z "$(FILE)" ]; then \
		echo "Usage: make run FILE=asm/your_file.asm"; \
		exit 1; \
	fi
	./build_and_run.py $(FILE)

# Just assemble all ASM files without running
assemble:
	@for asm in asm/*.asm; do \
		if [ -f "$$asm" ]; then \
			echo "Assembling $$asm..."; \
			nasm -f bin "$$asm" -o "$${asm%.asm}.bin"; \
		fi \
	done

# Show help
help:
	@echo "Available targets:"
	@echo "  make              - Build the project"
	@echo "  make clean        - Clean build artifacts"
	@echo "  make test         - Test with calc_add.asm"
	@echo "  make run-all      - Process all ASM files"
	@echo "  make run FILE=... - Process specific ASM file"
	@echo "  make assemble     - Just assemble all ASM files"