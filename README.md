# KAPINE
Kircular Accelerator Prototype Innovative New Experience

# Setup

## KiCAD Library Setup
Run the following command to install the third party KiCAD libraries used in the project

`git submodule update --init --recursive`

This project uses components from [digikey-kicad-library](https://github.com/Digi-Key/digikey-kicad-library) which is configured as a git submodule of this project. Git doesn't clone submodules of a project by default when the project it is cloned so this has to be done manually.


# Hardware

## Pinout

**Application Signal Descriptions**
* m[0..15] -- Electromagnet control signals
* s[0..15] -- Photogate sensor input signals
* Debug 1 -- Debug led 1
* Debug 2 -- Debug led 2
* USART1 -- Command link UART

| Microcontroller Pin | Application Signal | Direction   | Alternate Function |
|---------------------|--------------------|-------------|--------------------|
| PB5                 | m0                 | Digital out | TBD                |
| PD2                 | m1                 | Digital out | TBD                |
| PC11                | m2                 | Digital out | TBD                |
| PA12                | m3                 | Digital out | TBD                |
| PA8                 | m4                 | Digital out | TBD                |
| PC8                 | m5                 | Digital out | TBD                |
| PC6                 | m6                 | Digital out | TBD                |
| PB14                | m7                 | Digital out | TBD                |
| PB11                | m8                 | Digital out | TBD                |
| PB2                 | m9                 | Digital out | TBD                |
| PC5                 | m10                | Digital out | TBD                |
| PA3                 | m11                | Digital out | TBD                |
| PC3                 | m12                | Digital out | TBD                |
| PC1                 | m13                | Digital out | TBD                |
| PB9                 | m14                | Digital out | TBD                |
| PB7                 | m15                | Digital out | TBD                |
| PB4                 | s0                 | Digital in  | TBD                |
| PC12                | s1                 | Digital in  | TBD                |
| PC10                | s2                 | Digital in  | TBD                |
| PA11                | s3                 | Digital in  | TBD                |
| PC9                 | s4                 | Digital in  | TBD                |
| PC7                 | s5                 | Digital in  | TBD                |
| PB15                | s6                 | Digital in  | TBD                |
| PB13                | s7                 | Digital in  | TBD                |
| PB10                | s8                 | Digital in  | TBD                |
| PB1                 | s9                 | Digital in  | TBD                |
| PC4                 | s10                | Digital in  | TBD                |
| PA2                 | s11                | Digital in  | TBD                |
| PC2                 | s12                | Digital in  | TBD                |
| PC0                 | s13                | Digital in  | TBD                |
| PB8                 | s14                | Digital in  | TBD                |
| PB6                 | s15                | Digital in  | TBD                |
| PA0                 | Debug 1            | Digital out | TBD                |
| PA1                 | Debug 2            | Digital out | TBD                |
| PA9                 | USART1_TX          | Digital out | AF7                |
| PA10                | USART1_RX          | Digital in  | AF7                |
