EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Amplifier_Operational:MCP6001U U1
U 1 1 603BC9C0
P 4550 4050
F 0 "U1" H 4750 4350 50  0000 L CNN
F 1 "MCP6001U" H 4600 4250 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 4550 4050 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4550 4050 50  0001 C CNN
	1    4550 4050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 603BD37D
P 3250 3400
F 0 "#PWR01" H 3250 3250 50  0001 C CNN
F 1 "+3.3V" H 3265 3573 50  0000 C CNN
F 2 "" H 3250 3400 50  0001 C CNN
F 3 "" H 3250 3400 50  0001 C CNN
	1    3250 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT_TRIM RV1
U 1 1 603BE84F
P 3900 4150
F 0 "RV1" H 3831 4196 50  0000 R CNN
F 1 "trimpot" H 3831 4105 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Vishay_TS53YJ_Vertical" H 3900 4150 50  0001 C CNN
F 3 "~" H 3900 4150 50  0001 C CNN
	1    3900 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 4150 4250 4150
$Comp
L Device:Q_Photo_NPN Q1
U 1 1 603BFF21
P 3150 3700
F 0 "Q1" H 3340 3746 50  0000 L CNN
F 1 "Q_Photo_NPN" H 3340 3655 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x01_P2.54mm_Vertical" H 3350 3800 50  0001 C CNN
F 3 "~" H 3150 3700 50  0001 C CNN
	1    3150 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 603C1C44
P 3250 4150
F 0 "R1" H 3320 4196 50  0000 L CNN
F 1 "10k" H 3320 4105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3180 4150 50  0001 C CNN
F 3 "~" H 3250 4150 50  0001 C CNN
	1    3250 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4000 3250 3950
Wire Wire Line
	3250 3400 3250 3450
Wire Wire Line
	3250 3450 3900 3450
Wire Wire Line
	3900 3450 3900 4000
Connection ~ 3250 3450
Wire Wire Line
	3250 3450 3250 3500
Wire Wire Line
	3250 3950 4250 3950
Connection ~ 3250 3950
Wire Wire Line
	3250 3950 3250 3900
Wire Wire Line
	3900 4300 3900 4350
$Comp
L power:GND #PWR05
U 1 1 603C7992
P 4450 4350
F 0 "#PWR05" H 4450 4100 50  0001 C CNN
F 1 "GND" H 4455 4177 50  0000 C CNN
F 2 "" H 4450 4350 50  0001 C CNN
F 3 "" H 4450 4350 50  0001 C CNN
	1    4450 4350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 603C81C8
P 4450 3750
F 0 "#PWR04" H 4450 3600 50  0001 C CNN
F 1 "+3.3V" H 4465 3923 50  0000 C CNN
F 2 "" H 4450 3750 50  0001 C CNN
F 3 "" H 4450 3750 50  0001 C CNN
	1    4450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4050 5300 4050
Text Label 5700 4050 0    50   ~ 0
signal_out
Text Notes 3650 2400 0    50   ~ 0
Photo gate sensor board
Text Notes 4700 4500 0    50   ~ 0
opamp comparator\n* path free -> 3.3V\n* path obstructed -> 0V\ni.e. the logic output is inverted
Text Notes 3100 4850 0    50   ~ 0
threshold voltage can be set using trimpot
$Comp
L Device:LED D1
U 1 1 603C953D
P 5300 3800
F 0 "D1" V 5339 3683 50  0000 R CNN
F 1 "LED" V 5248 3683 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 5300 3800 50  0001 C CNN
F 3 "~" H 5300 3800 50  0001 C CNN
	1    5300 3800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 603CCCE3
P 5300 3500
F 0 "R2" H 5370 3546 50  0000 L CNN
F 1 "10k" H 5370 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5230 3500 50  0001 C CNN
F 3 "~" H 5300 3500 50  0001 C CNN
	1    5300 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4050 5300 3950
Connection ~ 5300 4050
Wire Wire Line
	5300 4050 5700 4050
$Comp
L power:+3.3V #PWR06
U 1 1 603CE764
P 5300 3350
F 0 "#PWR06" H 5300 3200 50  0001 C CNN
F 1 "+3.3V" H 5315 3523 50  0000 C CNN
F 2 "" H 5300 3350 50  0001 C CNN
F 3 "" H 5300 3350 50  0001 C CNN
	1    5300 3350
	1    0    0    -1  
$EndComp
Text Notes 5600 3600 0    50   ~ 0
sensor indicator LED
$Comp
L Connector:Conn_01x03_Female J1
U 1 1 603D0817
P 7350 4000
F 0 "J1" H 7378 4026 50  0000 L CNN
F 1 "motherboard connector" H 7378 3935 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 7350 4000 50  0001 C CNN
F 3 "~" H 7350 4000 50  0001 C CNN
	1    7350 4000
	1    0    0    1   
$EndComp
Text Label 7050 3900 2    50   ~ 0
signal_out
Wire Wire Line
	7100 4100 7150 4100
Wire Wire Line
	7150 3900 7050 3900
$Comp
L power:GND #PWR08
U 1 1 603D2FCE
P 7100 4100
F 0 "#PWR08" H 7100 3850 50  0001 C CNN
F 1 "GND" H 7105 3927 50  0000 C CNN
F 2 "" H 7100 4100 50  0001 C CNN
F 3 "" H 7100 4100 50  0001 C CNN
	1    7100 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4000 7150 4000
$Comp
L power:+3.3V #PWR07
U 1 1 603D300E
P 7100 4000
F 0 "#PWR07" H 7100 3850 50  0001 C CNN
F 1 "+3.3V" V 7115 4128 50  0000 L CNN
F 2 "" H 7100 4000 50  0001 C CNN
F 3 "" H 7100 4000 50  0001 C CNN
	1    7100 4000
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 603D8852
P 850 900
F 0 "#PWR09" H 850 750 50  0001 C CNN
F 1 "+3.3V" H 865 1073 50  0000 C CNN
F 2 "" H 850 900 50  0001 C CNN
F 3 "" H 850 900 50  0001 C CNN
	1    850  900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 603D916C
P 1250 900
F 0 "#PWR010" H 1250 650 50  0001 C CNN
F 1 "GND" H 1255 727 50  0000 C CNN
F 2 "" H 1250 900 50  0001 C CNN
F 3 "" H 1250 900 50  0001 C CNN
	1    1250 900 
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 603D9DC9
P 850 900
F 0 "#FLG01" H 850 975 50  0001 C CNN
F 1 "PWR_FLAG" H 850 1073 50  0000 C CNN
F 2 "" H 850 900 50  0001 C CNN
F 3 "~" H 850 900 50  0001 C CNN
	1    850  900 
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 603DA5F0
P 1250 900
F 0 "#FLG02" H 1250 975 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 1073 50  0000 C CNN
F 2 "" H 1250 900 50  0001 C CNN
F 3 "~" H 1250 900 50  0001 C CNN
	1    1250 900 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 603E18D9
P 1000 3400
F 0 "#PWR011" H 1000 3250 50  0001 C CNN
F 1 "+3.3V" H 1015 3573 50  0000 C CNN
F 2 "" H 1000 3400 50  0001 C CNN
F 3 "" H 1000 3400 50  0001 C CNN
	1    1000 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D2
U 1 1 603E2689
P 1000 3500
F 0 "D2" V 1046 3432 50  0000 R CNN
F 1 "LED_Small" V 955 3432 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" V 1000 3500 50  0001 C CNN
F 3 "~" V 1000 3500 50  0001 C CNN
	1    1000 3500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 603E3A4A
P 1000 3700
F 0 "R3" H 1059 3746 50  0000 L CNN
F 1 "10k" H 1059 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1000 3700 50  0001 C CNN
F 3 "~" H 1000 3700 50  0001 C CNN
	1    1000 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 603E441F
P 1000 3800
F 0 "#PWR012" H 1000 3550 50  0001 C CNN
F 1 "GND" H 1005 3627 50  0000 C CNN
F 2 "" H 1000 3800 50  0001 C CNN
F 3 "" H 1000 3800 50  0001 C CNN
	1    1000 3800
	1    0    0    -1  
$EndComp
Text Notes 750  3000 0    50   ~ 0
power indicator
$Comp
L power:+3.3V #PWR02
U 1 1 603F3F62
P 2550 3400
F 0 "#PWR02" H 2550 3250 50  0001 C CNN
F 1 "+3.3V" H 2565 3573 50  0000 C CNN
F 2 "" H 2550 3400 50  0001 C CNN
F 3 "" H 2550 3400 50  0001 C CNN
	1    2550 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 603F4E9C
P 2550 3600
F 0 "D3" V 2589 3483 50  0000 R CNN
F 1 "IR LED" V 2498 3483 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x01_P2.54mm_Vertical" H 2550 3600 50  0001 C CNN
F 3 "~" H 2550 3600 50  0001 C CNN
	1    2550 3600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 603F6B2D
P 2550 4100
F 0 "R4" H 2620 4146 50  0000 L CNN
F 1 "1k" H 2620 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2480 4100 50  0001 C CNN
F 3 "~" H 2550 4100 50  0001 C CNN
	1    2550 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 3950 2550 3750
Wire Notes Line
	3450 3900 2400 3900
Wire Notes Line
	2400 3900 2400 3400
Wire Notes Line
	2400 3400 3450 3400
Wire Notes Line
	3450 3400 3450 3900
Wire Wire Line
	2550 3400 2550 3450
$Comp
L power:GND #PWR03
U 1 1 603FD418
P 2550 4400
F 0 "#PWR03" H 2550 4150 50  0001 C CNN
F 1 "GND" H 2555 4227 50  0000 C CNN
F 2 "" H 2550 4400 50  0001 C CNN
F 3 "" H 2550 4400 50  0001 C CNN
	1    2550 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4250 2550 4400
Wire Wire Line
	3250 4300 3250 4350
Wire Wire Line
	3900 4350 3550 4350
Wire Wire Line
	3550 4350 3550 4400
Connection ~ 3550 4350
Wire Wire Line
	3550 4350 3250 4350
$Comp
L power:GND #PWR013
U 1 1 603FEAA3
P 3550 4400
F 0 "#PWR013" H 3550 4150 50  0001 C CNN
F 1 "GND" H 3555 4227 50  0000 C CNN
F 2 "" H 3550 4400 50  0001 C CNN
F 3 "" H 3550 4400 50  0001 C CNN
	1    3550 4400
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 604096DF
P 850 5300
F 0 "H1" H 950 5346 50  0000 L CNN
F 1 "MountingHole" H 950 5255 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 850 5300 50  0001 C CNN
F 3 "~" H 850 5300 50  0001 C CNN
	1    850  5300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 604099BB
P 850 5500
F 0 "H2" H 950 5546 50  0000 L CNN
F 1 "MountingHole" H 950 5455 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 850 5500 50  0001 C CNN
F 3 "~" H 850 5500 50  0001 C CNN
	1    850  5500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 60409CF1
P 850 5700
F 0 "H3" H 950 5746 50  0000 L CNN
F 1 "MountingHole" H 950 5655 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 850 5700 50  0001 C CNN
F 3 "~" H 850 5700 50  0001 C CNN
	1    850  5700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 6040A0E2
P 850 5900
F 0 "H4" H 950 5946 50  0000 L CNN
F 1 "MountingHole" H 950 5855 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 850 5900 50  0001 C CNN
F 3 "~" H 850 5900 50  0001 C CNN
	1    850  5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 6041AC1A
P 6700 2750
F 0 "C1" H 6792 2796 50  0000 L CNN
F 1 "100n" H 6792 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6700 2750 50  0001 C CNN
F 3 "~" H 6700 2750 50  0001 C CNN
	1    6700 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 6041B38E
P 6700 2650
F 0 "#PWR014" H 6700 2500 50  0001 C CNN
F 1 "+3.3V" H 6715 2823 50  0000 C CNN
F 2 "" H 6700 2650 50  0001 C CNN
F 3 "" H 6700 2650 50  0001 C CNN
	1    6700 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 6041BE09
P 6700 2850
F 0 "#PWR015" H 6700 2600 50  0001 C CNN
F 1 "GND" H 6705 2677 50  0000 C CNN
F 2 "" H 6700 2850 50  0001 C CNN
F 3 "" H 6700 2850 50  0001 C CNN
	1    6700 2850
	1    0    0    -1  
$EndComp
Text Notes 3500 3300 0    50   ~ 0
shorter leg of phototransistor\nconnects to 3.3V
$EndSCHEMATC
