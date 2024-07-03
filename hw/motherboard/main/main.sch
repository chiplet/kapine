EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title "KAPINE Motherboard"
Date "2020-09-03"
Rev "1.2"
Comp "Verneri Hirvonen"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Switch:SW_Push SW1
U 1 1 5E3D0E9E
P 4700 2400
F 0 "SW1" H 4900 2300 50  0000 C CNN
F 1 "SW_Push" H 4600 2300 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 4700 2600 50  0001 C CNN
F 3 "" H 4700 2600 50  0001 C CNN
	1    4700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2400 4950 2400
$Comp
L Device:C_Small C11
U 1 1 5E3D1058
P 4700 2150
F 0 "C11" V 4600 2050 50  0000 C CNN
F 1 "100n" V 4600 2300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4700 2150 50  0001 C CNN
F 3 "~" H 4700 2150 50  0001 C CNN
	1    4700 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 2150 4950 2150
Wire Wire Line
	4950 2150 4950 2400
Connection ~ 4950 2400
Wire Wire Line
	4950 2400 5350 2400
Wire Wire Line
	4600 2150 4450 2150
Wire Wire Line
	4450 2150 4450 2400
Wire Wire Line
	4450 2400 4500 2400
Wire Wire Line
	4450 2400 4250 2400
Connection ~ 4450 2400
$Comp
L power:GND #PWR09
U 1 1 5E3D149B
P 4250 2400
F 0 "#PWR09" H 4250 2150 50  0001 C CNN
F 1 "GND" H 4250 2450 50  0000 C CNN
F 2 "" H 4250 2400 50  0001 C CNN
F 3 "" H 4250 2400 50  0001 C CNN
	1    4250 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5E3D16CD
P 4000 2600
F 0 "R1" V 4100 2600 50  0000 C CNN
F 1 "10k" V 4000 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3930 2600 50  0001 C CNN
F 3 "~" H 4000 2600 50  0001 C CNN
	1    4000 2600
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR05
U 1 1 5E3D201C
P 3650 2350
F 0 "#PWR05" H 3650 2200 50  0001 C CNN
F 1 "VDD" H 3667 2523 50  0000 C CNN
F 2 "" H 3650 2350 50  0001 C CNN
F 3 "" H 3650 2350 50  0001 C CNN
	1    3650 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5E3D20BD
P 3650 2850
F 0 "#PWR06" H 3650 2600 50  0001 C CNN
F 1 "GND" H 3655 2677 50  0000 C CNN
F 2 "" H 3650 2850 50  0001 C CNN
F 3 "" H 3650 2850 50  0001 C CNN
	1    3650 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR01
U 1 1 5E3D29C9
P 8800 4150
F 0 "#PWR01" H 8800 4000 50  0001 C CNN
F 1 "VDD" H 8817 4323 50  0000 C CNN
F 2 "" H 8800 4150 50  0001 C CNN
F 3 "" H 8800 4150 50  0001 C CNN
	1    8800 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5E3D29E5
P 8800 4550
F 0 "#PWR02" H 8800 4300 50  0001 C CNN
F 1 "GND" H 8805 4377 50  0000 C CNN
F 2 "" H 8800 4550 50  0001 C CNN
F 3 "" H 8800 4550 50  0001 C CNN
	1    8800 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5E3D2A82
P 8800 4350
F 0 "C1" H 8892 4396 50  0000 L CNN
F 1 "4.7u" H 8892 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8800 4350 50  0001 C CNN
F 3 "~" H 8800 4350 50  0001 C CNN
	1    8800 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5E3D2AD8
P 9150 4350
F 0 "C3" H 9242 4396 50  0000 L CNN
F 1 "100n" H 9242 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9150 4350 50  0001 C CNN
F 3 "~" H 9150 4350 50  0001 C CNN
	1    9150 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5E3D2B00
P 9500 4350
F 0 "C5" H 9592 4396 50  0000 L CNN
F 1 "100n" H 9592 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9500 4350 50  0001 C CNN
F 3 "~" H 9500 4350 50  0001 C CNN
	1    9500 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5E3D2B2A
P 9850 4350
F 0 "C7" H 9942 4396 50  0000 L CNN
F 1 "100n" H 9942 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9850 4350 50  0001 C CNN
F 3 "~" H 9850 4350 50  0001 C CNN
	1    9850 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4150 8800 4200
Wire Wire Line
	8800 4450 8800 4500
Wire Wire Line
	8800 4200 9150 4200
Wire Wire Line
	9850 4200 9850 4250
Connection ~ 8800 4200
Wire Wire Line
	8800 4200 8800 4250
Wire Wire Line
	8800 4500 9150 4500
Wire Wire Line
	9850 4500 9850 4450
Connection ~ 8800 4500
Wire Wire Line
	8800 4500 8800 4550
Wire Wire Line
	9500 4200 9500 4250
Connection ~ 9500 4200
Wire Wire Line
	9500 4200 9850 4200
Wire Wire Line
	9150 4200 9150 4250
Connection ~ 9150 4200
Wire Wire Line
	9150 4200 9500 4200
Wire Wire Line
	9150 4450 9150 4500
Connection ~ 9150 4500
Wire Wire Line
	9150 4500 9500 4500
Wire Wire Line
	9500 4450 9500 4500
Connection ~ 9500 4500
Wire Wire Line
	9500 4500 9850 4500
Text Notes 8750 3900 0    50   ~ 0
VDD decoupling
Wire Wire Line
	3550 3600 3550 4050
$Comp
L power:GND #PWR07
U 1 1 5E3D8B34
P 3950 3850
F 0 "#PWR07" H 3950 3600 50  0001 C CNN
F 1 "GND" H 3950 3700 50  0000 C CNN
F 2 "" H 3950 3850 50  0001 C CNN
F 3 "" H 3950 3850 50  0001 C CNN
	1    3950 3850
	-1   0    0    1   
$EndComp
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5E3D8F68
P 3950 4050
F 0 "Y1" H 4100 4150 50  0000 L CNN
F 1 "HSE" H 4100 3950 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_Abracon_ABM3C-4Pin_5.0x3.2mm" H 3950 4050 50  0001 C CNN
F 3 "~" H 3950 4050 50  0001 C CNN
	1    3950 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5E3D8F9E
P 3950 4450
F 0 "#PWR08" H 3950 4200 50  0001 C CNN
F 1 "GND" H 3955 4277 50  0000 C CNN
F 2 "" H 3950 4450 50  0001 C CNN
F 3 "" H 3950 4450 50  0001 C CNN
	1    3950 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4050 3550 4050
$Comp
L Device:C_Small C9
U 1 1 5E3DA2D5
P 3550 4200
F 0 "C9" H 3642 4246 50  0000 L CNN
F 1 "27p" H 3642 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3550 4200 50  0001 C CNN
F 3 "~" H 3550 4200 50  0001 C CNN
	1    3550 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4100 3550 4050
Connection ~ 3550 4050
$Comp
L Device:C_Small C10
U 1 1 5E3DACD0
P 4350 4200
F 0 "C10" H 4442 4246 50  0000 L CNN
F 1 "27p" H 4442 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4350 4200 50  0001 C CNN
F 3 "~" H 4350 4200 50  0001 C CNN
	1    4350 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4100 4350 4050
Wire Wire Line
	4350 4050 4100 4050
Wire Wire Line
	3550 4300 3550 4350
Wire Wire Line
	3550 4350 3950 4350
Wire Wire Line
	4350 4350 4350 4300
Wire Wire Line
	3950 4250 3950 4350
Connection ~ 3950 4350
Wire Wire Line
	3950 4350 4350 4350
Wire Wire Line
	3950 4350 3950 4450
Wire Wire Line
	4350 4050 4350 3700
Connection ~ 4350 4050
$Comp
L Device:R R3
U 1 1 5E3E1436
P 4900 3700
F 0 "R3" V 5000 3700 50  0000 C CNN
F 1 "390" V 4900 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4830 3700 50  0001 C CNN
F 3 "~" H 4900 3700 50  0001 C CNN
	1    4900 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 3700 5350 3700
$Comp
L power:VDDA #PWR012
U 1 1 5E3E315A
P 6350 2200
F 0 "#PWR012" H 6350 2050 50  0001 C CNN
F 1 "VDDA" H 6367 2373 50  0000 C CNN
F 2 "" H 6350 2200 50  0001 C CNN
F 3 "" H 6350 2200 50  0001 C CNN
	1    6350 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E3E7218
P 8800 5650
F 0 "#PWR04" H 8800 5400 50  0001 C CNN
F 1 "GND" H 8805 5477 50  0000 C CNN
F 2 "" H 8800 5650 50  0001 C CNN
F 3 "" H 8800 5650 50  0001 C CNN
	1    8800 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5E3E721E
P 8800 5450
F 0 "C2" H 8892 5496 50  0000 L CNN
F 1 "1u" H 8892 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8800 5450 50  0001 C CNN
F 3 "~" H 8800 5450 50  0001 C CNN
	1    8800 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5E3E7225
P 9150 5450
F 0 "C4" H 9242 5496 50  0000 L CNN
F 1 "10n" H 9242 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9150 5450 50  0001 C CNN
F 3 "~" H 9150 5450 50  0001 C CNN
	1    9150 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5250 8800 5300
Wire Wire Line
	8800 5550 8800 5600
Wire Wire Line
	8800 5300 9150 5300
Connection ~ 8800 5300
Wire Wire Line
	8800 5300 8800 5350
Wire Wire Line
	8800 5600 9150 5600
Connection ~ 8800 5600
Wire Wire Line
	8800 5600 8800 5650
Wire Wire Line
	9150 5300 9150 5350
Wire Wire Line
	9150 5550 9150 5600
Text Notes 8750 5000 0    50   ~ 0
VDDA decoupling
$Comp
L Jumper:Jumper_3_Open JP1
U 1 1 5E3EAEFE
P 3650 2600
F 0 "JP1" V 3696 2687 50  0000 L CNN
F 1 "boot select" V 3605 2687 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3650 2600 50  0001 C CNN
F 3 "~" H 3650 2600 50  0001 C CNN
	1    3650 2600
	0    -1   -1   0   
$EndComp
$Comp
L power:VDDA #PWR03
U 1 1 5E3FE15B
P 8800 5250
F 0 "#PWR03" H 8800 5100 50  0001 C CNN
F 1 "VDDA" H 8817 5423 50  0000 C CNN
F 2 "" H 8800 5250 50  0001 C CNN
F 3 "" H 8800 5250 50  0001 C CNN
	1    8800 5250
	1    0    0    -1  
$EndComp
Text Notes 850  650  0    50   ~ 0
JTAG Programmer
Text Label 6800 3700 0    50   ~ 0
SWDIO
Wire Wire Line
	7100 3700 6750 3700
Text Label 6800 3800 0    50   ~ 0
SWCLK
Wire Wire Line
	7100 3800 6750 3800
Text Label 6800 4400 0    50   ~ 0
SWO
Wire Wire Line
	7100 4400 6750 4400
Wire Wire Line
	4150 2600 5350 2600
Wire Wire Line
	3850 2600 3800 2600
Wire Wire Line
	5350 5400 5000 5400
$Comp
L Device:C_Small C6
U 1 1 5F430C9E
P 10200 4350
F 0 "C6" H 10292 4396 50  0000 L CNN
F 1 "100n" H 10292 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10200 4350 50  0001 C CNN
F 3 "~" H 10200 4350 50  0001 C CNN
	1    10200 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4200 10200 4200
Wire Wire Line
	10200 4200 10200 4250
Connection ~ 9850 4200
Wire Wire Line
	10200 4450 10200 4500
Wire Wire Line
	10200 4500 9850 4500
Connection ~ 9850 4500
$Comp
L power:GND #PWR014
U 1 1 5F44F05E
P 5850 5800
F 0 "#PWR014" H 5850 5550 50  0001 C CNN
F 1 "GND" H 5855 5627 50  0000 C CNN
F 2 "" H 5850 5800 50  0001 C CNN
F 3 "" H 5850 5800 50  0001 C CNN
	1    5850 5800
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR013
U 1 1 5F456982
P 5850 2200
F 0 "#PWR013" H 5850 2050 50  0001 C CNN
F 1 "VDD" H 5867 2373 50  0000 C CNN
F 2 "" H 5850 2200 50  0001 C CNN
F 3 "" H 5850 2200 50  0001 C CNN
	1    5850 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3900 7100 3900
$Comp
L Connector:Conn_ARM_JTAG_SWD_10 J2
U 1 1 5F468D97
P 1050 1550
F 0 "J2" H 1300 2150 50  0000 R CNN
F 1 "Conn_ARM_JTAG_SWD_10" H 2050 950 50  0000 R CNN
F 2 "digikey-footprints:PinHeader_2x5_P1.27mm_Drill.7mm" H 1050 1550 50  0001 C CNN
F 3 "http://infocenter.arm.com/help/topic/com.arm.doc.ddi0314h/DDI0314H_coresight_components_trm.pdf" V 700 300 50  0001 C CNN
	1    1050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5F4707A3
P 1050 2150
F 0 "#PWR011" H 1050 1900 50  0001 C CNN
F 1 "GND" H 1055 1977 50  0000 C CNN
F 2 "" H 1050 2150 50  0001 C CNN
F 3 "" H 1050 2150 50  0001 C CNN
	1    1050 2150
	1    0    0    -1  
$EndComp
Text Label 1600 1250 0    50   ~ 0
RESET#
Wire Wire Line
	1600 1250 1550 1250
Wire Wire Line
	1900 1450 1550 1450
Wire Wire Line
	1900 1550 1550 1550
Wire Wire Line
	1900 1650 1550 1650
Wire Wire Line
	1900 1750 1550 1750
Text Label 1600 1450 0    50   ~ 0
SWCLK
Text Label 1600 1550 0    50   ~ 0
SWDIO
Text Label 1600 1650 0    50   ~ 0
SWO
$Comp
L power:VDD #PWR010
U 1 1 5F47F5D9
P 1050 950
F 0 "#PWR010" H 1050 800 50  0001 C CNN
F 1 "VDD" H 1067 1123 50  0000 C CNN
F 2 "" H 1050 950 50  0001 C CNN
F 3 "" H 1050 950 50  0001 C CNN
	1    1050 950 
	1    0    0    -1  
$EndComp
Text Label 1900 1550 0    50   ~ 0
JTAG_TMS
Text Label 7100 3700 0    50   ~ 0
JTAG_TMS
Text Label 1900 1450 0    50   ~ 0
JTAG_TCK
Text Label 7100 3800 0    50   ~ 0
JTAG_TCK
Text Label 1900 1650 0    50   ~ 0
JTAG_TDO
Text Label 7100 4400 0    50   ~ 0
JTAG_TDO
Text Label 1900 1750 0    50   ~ 0
JTAG_TDI
Text Label 7100 3900 0    50   ~ 0
JTAG_TDI
Text Label 5050 2400 0    50   ~ 0
RESET#
Wire Wire Line
	950  2150 1050 2150
Connection ~ 1050 2150
Text Notes 2300 650  0    50   ~ 0
SWD pins for ST-LINK/V2-1
$Comp
L Connector:Conn_01x06_Female J1
U 1 1 5F3FB993
P 2600 1350
F 0 "J1" H 2450 800 50  0000 C CNN
F 1 "SWD holes" H 2450 900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2600 1350 50  0001 C CNN
F 3 "~" H 2600 1350 50  0001 C CNN
	1    2600 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	2800 1550 3100 1550
$Comp
L power:GND #PWR089
U 1 1 5F401CDC
P 3100 1350
F 0 "#PWR089" H 3100 1100 50  0001 C CNN
F 1 "GND" V 3105 1222 50  0000 R CNN
F 2 "" H 3100 1350 50  0001 C CNN
F 3 "" H 3100 1350 50  0001 C CNN
	1    3100 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2800 1050 2850 1050
$Comp
L power:VDD #PWR090
U 1 1 5F4038EE
P 3100 1550
F 0 "#PWR090" H 3100 1400 50  0001 C CNN
F 1 "VDD" V 3100 1750 50  0000 C CNN
F 2 "" H 3100 1550 50  0001 C CNN
F 3 "" H 3100 1550 50  0001 C CNN
	1    3100 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	3100 1350 2800 1350
Wire Wire Line
	2800 1150 2850 1150
Wire Wire Line
	2800 1250 2850 1250
Wire Wire Line
	2800 1450 2850 1450
Text Label 2850 1450 0    50   ~ 0
SWCLK
Text Label 2850 1250 0    50   ~ 0
SWDIO
Text Label 2850 1150 0    50   ~ 0
RESET#
Text Label 2850 1050 0    50   ~ 0
SWO
Text Label 6800 3300 0    50   ~ 0
USART1_TX
Text Label 6800 3400 0    50   ~ 0
USART1_RX
Wire Wire Line
	6750 3300 6800 3300
Wire Wire Line
	6750 3400 6800 3400
$Comp
L Device:LED D38
U 1 1 5F54A51D
P 10650 2550
F 0 "D38" V 10689 2432 50  0000 R CNN
F 1 "DEBUG_2" V 10598 2432 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 10650 2550 50  0001 C CNN
F 3 "~" H 10650 2550 50  0001 C CNN
	1    10650 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10650 2700 10650 2750
$Comp
L Device:R R11
U 1 1 5F54A524
P 10650 2900
F 0 "R11" H 10720 2946 50  0000 L CNN
F 1 "1k" H 10720 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10580 2900 50  0001 C CNN
F 3 "~" H 10650 2900 50  0001 C CNN
	1    10650 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 3050 10650 3100
$Comp
L power:GND #PWR0106
U 1 1 5F54A52C
P 10650 3100
F 0 "#PWR0106" H 10650 2850 50  0001 C CNN
F 1 "GND" H 10655 2927 50  0000 C CNN
F 2 "" H 10650 3100 50  0001 C CNN
F 3 "" H 10650 3100 50  0001 C CNN
	1    10650 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5F54A517
P 10050 3100
F 0 "#PWR0107" H 10050 2850 50  0001 C CNN
F 1 "GND" H 10055 2927 50  0000 C CNN
F 2 "" H 10050 3100 50  0001 C CNN
F 3 "" H 10050 3100 50  0001 C CNN
	1    10050 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 3050 10050 3100
$Comp
L Device:R R10
U 1 1 5F54A509
P 10050 2900
F 0 "R10" H 10120 2946 50  0000 L CNN
F 1 "1k" H 10120 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 2900 50  0001 C CNN
F 3 "~" H 10050 2900 50  0001 C CNN
	1    10050 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 2700 10050 2750
$Comp
L Device:LED D37
U 1 1 5F54A502
P 10050 2550
F 0 "D37" V 10089 2432 50  0000 R CNN
F 1 "DEBUG_1" V 9998 2432 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 10050 2550 50  0001 C CNN
F 3 "~" H 10050 2550 50  0001 C CNN
	1    10050 2550
	0    -1   -1   0   
$EndComp
Text Label 6800 2400 0    50   ~ 0
DEBUG_1
Text Label 6800 2500 0    50   ~ 0
DEBUG_2
Text Label 10050 2350 0    50   ~ 0
DEBUG_1
Wire Wire Line
	10050 2350 10050 2400
Wire Wire Line
	10650 2400 10650 2350
Text Label 10650 2350 0    50   ~ 0
DEBUG_2
Text Notes 10050 2200 0    50   ~ 0
DEBUG LEDs
Wire Wire Line
	4750 3700 4350 3700
Wire Wire Line
	5350 3600 3550 3600
Wire Wire Line
	6800 2500 6750 2500
Wire Wire Line
	6900 2600 6750 2600
$Sheet
S 4450 5300 550  200 
U 5E444A98
F0 "rgb" 50
F1 "rgb.sch" 50
F2 "DIN" I R 5000 5400 50 
$EndSheet
Text Label 6900 4600 0    50   ~ 0
m0
Text Notes 1700 3850 0    50   ~ 0
Electromagnet control signals: m[0..15]\nPhoto gate sensor inputs: s[0..15]
$Sheet
S 750  3400 550  300 
U 5F4CCA9E
F0 "power" 50
F1 "power.sch" 50
$EndSheet
$Comp
L Mechanical:MountingHole H4
U 1 1 5F6B0041
P 700 7600
F 0 "H4" H 800 7646 50  0000 L CNN
F 1 "MountingHole" H 800 7555 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 700 7600 50  0001 C CNN
F 3 "~" H 700 7600 50  0001 C CNN
	1    700  7600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5F6B0201
P 700 7400
F 0 "H3" H 800 7446 50  0000 L CNN
F 1 "MountingHole" H 800 7355 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 700 7400 50  0001 C CNN
F 3 "~" H 700 7400 50  0001 C CNN
	1    700  7400
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5F6B0478
P 700 7200
F 0 "H2" H 800 7246 50  0000 L CNN
F 1 "MountingHole" H 800 7155 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 700 7200 50  0001 C CNN
F 3 "~" H 700 7200 50  0001 C CNN
	1    700  7200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5F6B08BB
P 700 7000
F 0 "H1" H 800 7046 50  0000 L CNN
F 1 "MountingHole" H 800 6955 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 700 7000 50  0001 C CNN
F 3 "~" H 700 7000 50  0001 C CNN
	1    700  7000
	1    0    0    -1  
$EndComp
$Sheet
S 750  2850 550  300 
U 5F593B6F
F0 "uart" 50
F1 "uart.sch" 50
F2 "UART_TX" I R 1300 2950 50 
F3 "UART_RX" O R 1300 3050 50 
$EndSheet
Text Label 1350 2950 0    50   ~ 0
USART1_TX
Wire Wire Line
	1350 2950 1300 2950
Text Label 1350 3050 0    50   ~ 0
USART1_RX
Wire Wire Line
	1350 3050 1300 3050
Wire Wire Line
	6900 4600 6750 4600
Text Label 6900 4500 0    50   ~ 0
s0
Wire Wire Line
	6900 4500 6750 4500
Text Label 5200 3900 2    50   ~ 0
m1
Wire Wire Line
	5200 3900 5350 3900
Text Label 5200 5300 2    50   ~ 0
s1
Wire Wire Line
	5200 5300 5350 5300
Text Label 5200 5200 2    50   ~ 0
m2
Wire Wire Line
	5200 5200 5350 5200
Text Label 5200 5100 2    50   ~ 0
s2
Wire Wire Line
	5350 5100 5200 5100
Text Label 6900 3600 0    50   ~ 0
m3
Text Label 6900 3500 0    50   ~ 0
s3
Wire Wire Line
	6900 3500 6750 3500
Wire Wire Line
	6900 3600 6750 3600
Text Label 6900 3200 0    50   ~ 0
m4
Wire Wire Line
	6900 3200 6750 3200
Text Label 5200 5000 2    50   ~ 0
s4
Wire Wire Line
	5350 5000 5200 5000
Text Label 5200 4900 2    50   ~ 0
m5
Wire Wire Line
	5350 4900 5200 4900
Text Label 5200 4800 2    50   ~ 0
s5
Wire Wire Line
	5350 4800 5200 4800
Text Label 5200 4700 2    50   ~ 0
m6
Wire Wire Line
	5350 4700 5200 4700
Text Label 6900 5600 0    50   ~ 0
s6
Wire Wire Line
	6900 5600 6750 5600
Text Label 6900 5500 0    50   ~ 0
m7
Wire Wire Line
	6900 5500 6750 5500
Text Label 6900 5400 0    50   ~ 0
s7
Wire Wire Line
	6900 5400 6750 5400
Text Label 6900 5200 0    50   ~ 0
m8
Wire Wire Line
	6900 5200 6750 5200
Text Label 6900 5100 0    50   ~ 0
s8
Wire Wire Line
	6900 5100 6750 5100
Text Label 6900 4300 0    50   ~ 0
m9
Wire Wire Line
	6900 4300 6750 4300
Text Label 6900 4200 0    50   ~ 0
s9
Wire Wire Line
	6900 4200 6750 4200
Text Label 5200 4600 2    50   ~ 0
m10
Wire Wire Line
	5200 4600 5350 4600
Text Label 5200 4500 2    50   ~ 0
s10
Wire Wire Line
	5200 4500 5350 4500
Text Label 6900 2700 0    50   ~ 0
m11
Wire Wire Line
	6900 2700 6750 2700
Text Label 6900 2600 0    50   ~ 0
s11
Wire Wire Line
	6800 2400 6750 2400
Text Label 5200 4400 2    50   ~ 0
m12
Wire Wire Line
	5200 4400 5350 4400
Text Label 5200 4300 2    50   ~ 0
s12
Wire Wire Line
	5200 4300 5350 4300
Text Label 5200 4200 2    50   ~ 0
m13
Wire Wire Line
	5200 4200 5350 4200
Text Label 5200 4100 2    50   ~ 0
s13
Wire Wire Line
	5200 4100 5350 4100
Text Label 6900 5000 0    50   ~ 0
m14
Wire Wire Line
	6900 5000 6750 5000
Text Label 6900 4900 0    50   ~ 0
s14
Wire Wire Line
	6900 4900 6750 4900
Text Label 6900 4800 0    50   ~ 0
m15
Text Label 6900 4700 0    50   ~ 0
s15
Wire Wire Line
	6900 4800 6750 4800
Wire Wire Line
	6900 4700 6750 4700
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J9
U 1 1 5F7C296C
P 2150 4250
F 0 "J9" H 2200 4567 50  0000 C CNN
F 1 "stack[0..3]" H 2200 4476 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x04_P2.54mm_Vertical" H 2150 4250 50  0001 C CNN
F 3 "~" H 2150 4250 50  0001 C CNN
	1    2150 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J10
U 1 1 5F7D5386
P 2150 4900
F 0 "J10" H 2200 5217 50  0000 C CNN
F 1 "stack[0..3]" H 2200 5126 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x04_P2.54mm_Vertical" H 2150 4900 50  0001 C CNN
F 3 "~" H 2150 4900 50  0001 C CNN
	1    2150 4900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J11
U 1 1 5F7D6185
P 2150 5550
F 0 "J11" H 2200 5867 50  0000 C CNN
F 1 "stack[0..3]" H 2200 5776 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x04_P2.54mm_Vertical" H 2150 5550 50  0001 C CNN
F 3 "~" H 2150 5550 50  0001 C CNN
	1    2150 5550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J12
U 1 1 5F7D6C12
P 2150 6200
F 0 "J12" H 2200 6517 50  0000 C CNN
F 1 "stack[0..3]" H 2200 6426 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x04_P2.54mm_Vertical" H 2150 6200 50  0001 C CNN
F 3 "~" H 2150 6200 50  0001 C CNN
	1    2150 6200
	1    0    0    -1  
$EndComp
Text Label 2450 4150 0    50   ~ 0
m0
Text Label 2450 4250 0    50   ~ 0
m1
Text Label 2450 4350 0    50   ~ 0
m2
Text Label 2450 4450 0    50   ~ 0
m3
Text Label 2450 4800 0    50   ~ 0
m4
Text Label 2450 4900 0    50   ~ 0
m5
Text Label 2450 5000 0    50   ~ 0
m6
Text Label 2450 5100 0    50   ~ 0
m7
Text Label 2450 5450 0    50   ~ 0
m8
Text Label 2450 5550 0    50   ~ 0
m9
Text Label 2450 5650 0    50   ~ 0
m10
Text Label 2450 5750 0    50   ~ 0
m11
Text Label 2450 6100 0    50   ~ 0
m12
Text Label 2450 6200 0    50   ~ 0
m13
Text Label 2450 6300 0    50   ~ 0
m14
Text Label 2450 6400 0    50   ~ 0
m15
Text Label 1950 4150 2    50   ~ 0
s0
Text Label 1950 4250 2    50   ~ 0
s1
Text Label 1950 4350 2    50   ~ 0
s2
Text Label 1950 4450 2    50   ~ 0
s3
Text Label 1950 4800 2    50   ~ 0
s4
Text Label 1950 4900 2    50   ~ 0
s5
Text Label 1950 5000 2    50   ~ 0
s6
Text Label 1950 5100 2    50   ~ 0
s7
Text Label 1950 5450 2    50   ~ 0
s8
Text Label 1950 5550 2    50   ~ 0
s9
Text Label 1950 5650 2    50   ~ 0
s10
Text Label 1950 5750 2    50   ~ 0
s11
Text Label 1950 6100 2    50   ~ 0
s12
Text Label 1950 6200 2    50   ~ 0
s13
Text Label 1950 6300 2    50   ~ 0
s14
Text Label 1950 6400 2    50   ~ 0
s15
Wire Wire Line
	6250 5800 6150 5800
Connection ~ 6150 5800
Wire Wire Line
	6150 5800 6050 5800
Connection ~ 6050 5800
Wire Wire Line
	5950 5800 5850 5800
Wire Wire Line
	6050 5800 5950 5800
Connection ~ 5950 5800
Connection ~ 5850 5800
Wire Wire Line
	6150 2200 6250 2200
Connection ~ 6150 2200
Wire Wire Line
	6050 2200 6150 2200
Connection ~ 6050 2200
Wire Wire Line
	5850 2200 5950 2200
Wire Wire Line
	5950 2200 6050 2200
Connection ~ 5950 2200
Connection ~ 5850 2200
$Comp
L MCU_ST_STM32F3:STM32F303RETx U1
U 1 1 5F40C94C
P 6050 4000
F 0 "U1" H 6050 2800 50  0000 C CNN
F 1 "STM32F303RETx" H 6050 2700 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 5450 2300 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00118585.pdf" H 6050 4000 50  0001 C CNN
	1    6050 4000
	1    0    0    -1  
$EndComp
Text Label 6800 2800 0    50   ~ 0
PA4
Text Label 6800 2900 0    50   ~ 0
PA5
Text Label 6800 3000 0    50   ~ 0
PA6
Text Label 6800 3100 0    50   ~ 0
PA7
Wire Wire Line
	6800 3100 6750 3100
Wire Wire Line
	6800 3000 6750 3000
Wire Wire Line
	6800 2900 6750 2900
Wire Wire Line
	6800 2800 6750 2800
Text Label 8250 950  2    50   ~ 0
PA4
Text Label 8250 1050 2    50   ~ 0
PA5
Text Label 8250 1150 2    50   ~ 0
PA6
Text Label 8250 1250 2    50   ~ 0
PA7
Text Label 6800 4100 0    50   ~ 0
PB0
Wire Wire Line
	6800 4100 6750 4100
Text Label 8750 950  0    50   ~ 0
PB0
Text Label 6800 5300 0    50   ~ 0
PB12
Wire Wire Line
	6800 5300 6750 5300
Text Label 8750 1050 0    50   ~ 0
PB12
Text Label 5300 5500 2    50   ~ 0
PC14
Text Label 5300 5600 2    50   ~ 0
PC15
Wire Wire Line
	5300 5500 5350 5500
Wire Wire Line
	5300 5600 5350 5600
Text Label 8750 1150 0    50   ~ 0
PC14
Text Label 8750 1250 0    50   ~ 0
PC15
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 6017469B
P 2200 6700
F 0 "J4" H 2280 6692 50  0000 L CNN
F 1 "STACK_GND" H 2280 6601 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 2200 6700 50  0001 C CNN
F 3 "~" H 2200 6700 50  0001 C CNN
	1    2200 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 6700 1950 6700
Wire Wire Line
	1950 6700 1950 6800
Wire Wire Line
	2000 6800 1950 6800
Connection ~ 1950 6800
Wire Wire Line
	1950 6800 1950 6900
$Comp
L power:GND #PWR075
U 1 1 601818C2
P 1950 6900
F 0 "#PWR075" H 1950 6650 50  0001 C CNN
F 1 "GND" H 1955 6727 50  0000 C CNN
F 2 "" H 1950 6900 50  0001 C CNN
F 3 "" H 1950 6900 50  0001 C CNN
	1    1950 6900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Counter_Clockwise J13
U 1 1 601B22D7
P 8450 1050
F 0 "J13" H 8500 1367 50  0000 C CNN
F 1 "GPIO" H 8500 1276 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 8450 1050 50  0001 C CNN
F 3 "~" H 8450 1050 50  0001 C CNN
	1    8450 1050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
