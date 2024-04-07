# embedded-drivers

WIP: The homepage for embedded-drivers team.

embedded-hal compatible drivers for various sensors, displays, and IC modules.

## Why another driver crate?

- Rust embedded driver libraries are always outdated
- Most of them are not compatible with the newest embedded-hal, say v1 and v0.2
- Many Rust Embedded developers are from non-embedded backend, they move from the embedded world when their hobby project is done. So, they don't have time to maintain the driver library consistently
- Most of them are over-engineering. You write beautiful Rust code, but I just want to read a sensor value
  - All the hidden commands in datasheets are introduced in builder style. WTF
  - Commands are renamed to following your "rusty" style, but I have C example code, how can I convert them?
  - Unable to do raw register read/write, or any customization without Pull Request. Yet, no one reviews and merges them
  - Soft-fp is introduced!
- Embedded environments are resource-limited, so the driver should be small and fast, not an all-in-one library
- Unable to use DMA, or interrupt, or async/await, or any async feature in the driver
- I have a project with many IC components, but I have to use driver libraries of various styles from different authors, and they might not be compatible with each other
- Every embedded driver is different: If I change a screen to another, I need to look at the example codes and do heavy editing

A unified org is required to handle the situation.

## Goals

Being the "Adafruit" of Rust embedded ecosystem, providing the most up-to-date, easy-to-use, and well-documented driver libraries for different kinds of sensors, displays, or IC modules.

- **Up-to-date and reliable**: Consistently maintained drivers that align with the latest embedded-hal versions (v1, v0.2, and beyond).
- **Ease of use**: Prioritize simplicity and intuitive interfaces tailored to embedded developers, even those new to Rust.
- **Unified style**: Changing to another IC or display? just rename the IC in crate name.
- **Practical focus**: Enable core functionalities like reading sensor values without over-engineered abstractions.
- **Direct datasheet alignment**: Facilitate easy translation from C examples thanks to consistent command naming.
- **Customization**: Support raw register access for tailored modifications.
- **Resource-conscious**: Optimized for size and speed, essential for embedded environments.
- **Advanced features**: Harness DMA, interrupts, and async/await for optimal performance.
- **Coherent ecosystem**: Seamless compatibility between drivers, ensuring smooth project integration.
