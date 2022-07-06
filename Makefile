flash:
	cargo build --release
	arm-none-eabi-gdb --batch --command=flash.gdb target/thumbv7em-none-eabihf/release/lightning-in-a-box
