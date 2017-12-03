v 20150930 2
C 37600 38200 0 0 0 title-A-cibolo.sym
{
T 58500 38200 5 10 1 1 0 0 1
file=pyflex_f401.sch
T 62600 37900 5 10 1 1 0 0 1
drawn-by=John Griessen
T 50800 38000 5 18 1 1 0 0 1
title=Culture Shock PYFLEX_F401
T 58700 37900 5 10 1 1 0 0 1
first-pagenum=1
T 59400 37900 5 10 1 1 0 0 1
last-pagenum=1
T 62600 38200 5 10 1 1 0 0 1
rev=2017-11-18 v0.4
T 64400 43400 5 18 1 1 270 0 1
title2=Culture Shock PYFLEX_F401
}
N 44200 51400 44200 54600 4
N 45700 51400 45700 52000 4
{
T 45650 51900 5 10 1 1 90 6 1
netname=PB9
}
N 45100 51400 45100 53000 4
N 45400 51400 45400 52000 4
{
T 45350 51900 5 10 1 1 90 6 1
netname=PB8
}
N 44200 54300 44500 54300 4
N 50600 44800 50600 50300 4
N 50600 49300 49300 49300 4
C 49700 48800 1 0 0 capacitor-1.sym
{
T 49900 49500 5 10 0 0 0 0 1
device=CAPACITOR
T 49900 49700 5 10 0 0 0 0 1
symversion=0.1
T 50200 48700 5 10 1 1 180 0 1
refdes=C3
T 49700 48800 5 10 0 0 0 0 1
footprint=1206.fp
}
N 49700 49000 49300 49000 4
N 40300 51400 44800 51400 4
N 44500 53000 46400 53000 4
C 47300 44000 1 0 0 capacitor-1.sym
{
T 47500 44700 5 10 0 0 0 0 1
device=CAPACITOR
T 47500 44900 5 10 0 0 0 0 1
symversion=0.1
T 47900 44300 5 10 1 1 0 0 1
refdes=C2
T 47600 44100 5 10 1 1 180 0 1
value=4.7 uF
T 47700 44400 5 10 0 1 180 0 1
footprint=CAPC1608N.lht
}
N 47300 44700 47300 44200 4
N 50600 44800 48200 44800 4
N 46000 51400 46000 54100 4
T 45200 54600 9 12 1 0 0 0 1
BOOT0
N 45600 54100 46000 54100 4
C 45700 53200 1 90 0 resistor-1.sym
{
T 45300 53500 5 10 0 0 90 0 1
device=RESISTOR
T 45500 54000 5 10 1 1 180 0 1
refdes=R5
T 45900 53400 5 10 1 1 90 0 1
value=100K
}
N 45600 53000 45600 53200 4
C 64200 42200 1 0 1 ffc-zif-20.sym
{
T 64100 48800 5 10 1 1 0 6 1
refdes=J1
}
N 44600 44300 40500 44300 4
{
T 44350 44350 5 10 1 1 0 6 1
netname=PA3_USART2_RX_TIM2.4_TIM5.4_TIM9.2
}
N 44900 44000 40500 44000 4
{
T 42850 44050 5 10 1 1 0 6 1
netname=PA4_ADC1.4_HV_sense
}
N 40500 43700 45200 43700 4
{
T 42600 43750 5 10 1 1 0 6 1
netname=PA5_ADC1.5_TIM2.1
}
T 39800 53000 9 12 1 0 0 0 3
VBAT could get a backup battery 
for implementing real time clock
if timestamping becomes popular.
L 41700 52900 43800 50500 3 10 1 0 -1 -1
B 39700 52900 3400 800 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
N 49300 47800 51100 47800 4
{
T 49400 47850 5 10 1 1 0 0 1
netname=PA11_OTG_FS_DM
}
N 49300 47500 51500 47500 4
{
T 49400 47550 5 10 1 1 0 0 1
netname=PA10_OTG_FS_ID
}
C 45600 52700 1 0 0 com.sym
C 49400 48700 1 0 0 com.sym
C 51600 46500 1 0 0 nc-right-1.sym
{
T 51700 47000 5 10 0 0 0 0 1
value=NoConnection
T 51700 47200 5 10 0 0 0 0 1
device=DRC_Directive
}
C 45800 52000 1 90 0 nc-right-1.sym
{
T 45300 52100 5 10 0 0 90 0 1
value=NoConnection
T 45100 52100 5 10 0 0 90 0 1
device=DRC_Directive
}
C 45500 52000 1 90 0 nc-right-1.sym
{
T 45000 52100 5 10 0 0 90 0 1
value=NoConnection
T 44800 52100 5 10 0 0 90 0 1
device=DRC_Directive
}
N 62900 43200 61100 43200 4
N 62900 43500 62400 43500 4
C 62200 42300 1 0 1 com.sym
N 62900 42600 62100 42600 4
N 62400 43500 62400 42600 4
N 62900 42900 61100 42900 4
N 51000 57300 51000 56700 4
C 51300 57700 1 0 1 Vdd.sym
{
T 51100 57950 5 10 1 1 0 3 1
net=USB_C_5V:1
}
C 49800 55700 1 0 0 USB_SMT_5p.sym
{
T 49800 56950 5 10 1 1 0 0 1
refdes=J4
T 50090 56010 5 16 0 1 90 0 1
device=microUSB_AB_antek
T 50100 68150 5 10 0 0 0 0 1
footprint=microUSB_AB_1.fp
}
N 52300 56500 54600 56500 4
{
T 52900 56550 5 10 1 1 0 0 1
netname=PA11_OTG_FS_DM
}
N 51000 56300 51400 56300 4
C 44000 54600 1 0 0 Vdd.sym
{
T 44050 54850 5 10 1 1 0 0 1
net=3.3V:1
}
N 42400 55600 42400 56200 4
N 41400 56200 43300 56200 4
C 42600 54700 1 90 0 capacitor-1.sym
{
T 41900 54900 5 10 0 0 90 0 1
device=MF-CAP-0603-1uF
T 41700 54900 5 10 0 0 90 0 1
symversion=0.1
T 42600 54700 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 42600 54700 5 10 0 0 0 0 1
description=1 uF X7R 25V
T 42000 55300 5 10 1 1 0 0 1
refdes=C11
T 42500 54900 5 10 1 1 0 0 1
value=1 uF
}
C 43500 54700 1 90 0 capacitor-1.sym
{
T 42800 54900 5 10 0 0 90 0 1
device=MF-CAP-0603-0.1uF
T 42600 54900 5 10 0 0 90 0 1
symversion=0.1
T 43500 54700 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 43500 54700 5 10 0 0 0 0 1
description=Capacitor MLCC 0603 0.1uF 10% 25V
T 42900 55300 5 10 1 1 0 0 1
refdes=C10
T 43400 54900 5 10 1 1 0 0 1
value=0.1 uF
}
N 43300 55600 43300 56200 4
C 42000 54700 1 90 0 capacitor-1.sym
{
T 41300 54900 5 10 0 0 90 0 1
device=MF-CAP-0603-1uF
T 41100 54900 5 10 0 0 90 0 1
symversion=0.1
T 42000 54700 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 42000 54700 5 10 0 0 0 0 1
description=1 uF X7R 25V
T 41400 55300 5 10 1 1 0 0 1
refdes=C12
T 41900 54900 5 10 1 1 0 0 1
value=1 uF
}
N 40900 54700 43300 54700 4
C 42700 56200 1 0 0 Vdd.sym
{
T 42700 56400 5 10 1 1 0 0 1
net=3.3V:1
}
N 41800 55600 42400 55600 4
C 42500 54400 1 0 0 com.sym
N 40900 54700 40900 55800 4
N 40400 56200 39800 56200 4
N 39800 56200 39800 57000 4
C 40400 55800 1 0 0 regulator_sot23.sym
{
T 40400 57900 5 8 0 0 0 0 1
symversion=3.0
T 41400 56400 5 10 1 1 0 0 1
value=3.3
T 40500 56900 5 10 1 1 0 0 1
refdes=U1
T 40900 55700 5 8 1 1 0 0 1
footprint=SOT23-123.fp
T 40480 56649 5 10 1 1 0 0 1
device=AP2120N-3.3TRG1
}
C 60500 55600 1 270 0 coil-1.sym
{
T 60900 55400 5 10 0 0 270 0 1
device=COIL
T 60700 55200 5 10 1 1 0 0 1
refdes=L31
T 61100 55400 5 10 0 0 270 0 1
symversion=0.1
}
C 57400 53700 1 0 0 dc-dc-sot23-5.sym
{
T 59800 56000 5 14 1 1 0 3 1
refdes=U3
T 57900 54900 5 10 1 1 0 0 1
device=TPS61040DBVR
T 58000 54200 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
C 55300 47700 1 0 0 dc-dc-sot23-5.sym
{
T 57700 50000 5 14 1 1 0 3 1
refdes=U4
T 55800 48900 5 10 1 1 0 0 1
device=G5111T11UF
T 55800 48300 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 60500 55600 60500 56250 4
N 60500 56250 59500 56250 4
N 59500 56250 59500 56200 4
N 60500 54600 60500 53400 4
N 53900 53400 60500 53400 4
N 59500 53400 59500 53700 4
C 60300 56250 1 270 1 schottky.sym
{
T 60900 56650 5 10 0 0 270 6 1
device=Schottky
T 61150 56700 5 10 1 1 0 6 1
refdes=D31
}
N 60500 57150 60500 57600 4
N 55500 57600 62300 57600 4
C 58400 57600 1 270 0 capacitor-1.sym
{
T 59100 57400 5 10 0 0 270 0 1
device=CAPACITOR
T 59300 57400 5 10 0 0 270 0 1
symversion=0.1
T 58100 57300 5 10 1 1 0 0 1
refdes=C32
T 58800 57000 5 10 1 1 0 0 1
value=1 uF
}
N 58600 56700 58600 56200 4
N 58600 56400 58200 56400 4
N 58200 52500 58200 56400 4
N 51000 52500 62200 52500 4
N 51000 52500 51000 55900 4
C 58900 52200 1 0 1 com.sym
N 52500 43200 52500 57700 4
N 53000 53400 52500 53400 4
N 54400 46500 57200 46500 4
N 52800 47300 57400 47300 4
C 55000 41200 1 0 0 dc-dc-sot23-5.sym
{
T 57400 43500 5 14 1 1 0 3 1
refdes=U5
T 55500 42400 5 10 1 1 0 0 1
device=AP3012KTR-G1
T 55500 41800 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 56800 57600 56800 57400 4
C 56700 57400 1 270 0 resistor-1.sym
{
T 57100 57100 5 10 0 0 270 0 1
device=RESISTOR
T 56600 56600 5 10 1 1 90 0 1
refdes=R31
T 57100 56800 5 10 1 1 90 0 1
value=2.2 M
T 56700 57400 5 10 0 0 90 0 1
footprint=0805_HV.fp
}
N 55500 56500 57700 56500 4
N 57700 56500 57700 56200 4
N 56800 56500 56800 55400 4
N 56800 54500 56800 52500 4
N 57700 53700 56200 53700 4
N 55300 53700 51900 53700 4
N 57700 53600 57700 53700 4
N 57700 52700 57700 52500 4
C 62500 57600 1 0 1 power_+.sym
{
T 63000 57800 5 10 1 1 0 6 1
net=20V_ONBOARD:1
}
C 55300 57600 1 270 0 capacitor-1.sym
{
T 56000 57400 5 10 0 0 270 0 1
device=CAPACITOR
T 56200 57400 5 10 0 0 270 0 1
symversion=0.1
T 54900 57300 5 10 1 1 0 0 1
refdes=C3FF
T 55800 56900 5 10 1 1 0 0 1
value=1 uF
}
N 55500 56700 55500 56500 4
C 58400 49600 1 270 0 coil-1.sym
{
T 58800 49400 5 10 0 0 270 0 1
device=COIL
T 59000 49400 5 10 0 0 270 0 1
symversion=0.1
T 58600 49200 5 10 1 1 0 0 1
refdes=L41
}
N 58400 49600 58400 50250 4
N 58400 50250 57400 50250 4
N 57400 50250 57400 50200 4
N 58400 48600 58400 48000 4
N 57400 47300 57400 47700 4
C 58200 50250 1 270 1 schottky.sym
{
T 58800 50650 5 10 0 0 270 6 1
device=Schottky
T 59050 50600 5 10 1 1 0 6 1
refdes=D41
}
N 58400 51150 58400 51600 4
N 53400 51600 60200 51600 4
C 56300 51600 1 270 0 capacitor-1.sym
{
T 57000 51400 5 10 0 0 270 0 1
device=CAPACITOR
T 57200 51400 5 10 0 0 270 0 1
symversion=0.1
T 56000 51200 5 10 1 1 0 0 1
refdes=C42
T 56700 51000 5 10 1 1 0 0 1
value=1 uF
}
N 56500 50700 56500 50200 4
N 56500 50400 56100 50400 4
N 56100 46500 56100 50400 4
C 57300 46200 1 0 1 com.sym
N 54700 51600 54700 51400 4
C 54600 51400 1 270 0 resistor-1.sym
{
T 55000 51100 5 10 0 0 270 0 1
device=2013M R 2.2 M 1%
T 54600 51400 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 54500 50600 5 10 1 1 90 0 1
refdes=R41
T 55000 50800 5 10 1 1 90 0 1
value=2.2 M
}
C 54600 49400 1 270 0 resistor-1.sym
{
T 55000 49100 5 10 0 0 270 0 1
device=2013M R 160 k 1%
T 54600 49400 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 54500 48600 5 10 1 1 90 0 1
refdes=R42
T 55000 48800 5 10 1 1 90 0 1
value=160 k
}
N 53400 50500 55600 50500 4
N 55600 50500 55600 50200 4
N 54700 50500 54700 49400 4
N 54700 48500 54700 46500 4
N 52800 47300 52800 48600 4
N 55600 47600 55600 47700 4
N 55600 46700 55600 46500 4
C 53200 51600 1 270 0 capacitor-1.sym
{
T 53900 51400 5 10 0 0 270 0 1
device=CAPACITOR
T 54100 51400 5 10 0 0 270 0 1
symversion=0.1
T 52900 51200 5 10 1 1 0 0 1
refdes=C4FF
T 53600 51000 5 10 1 1 0 0 1
value=1 uF
}
N 53400 50700 53400 50500 4
N 53100 40000 58100 40000 4
N 52500 40800 57100 40800 4
C 58100 43100 1 270 0 coil-1.sym
{
T 58500 42900 5 10 0 0 270 0 1
device=COIL
T 58700 42900 5 10 0 0 270 0 1
symversion=0.1
T 57700 42700 5 10 1 1 0 0 1
refdes=L51
}
N 58100 43100 58100 43750 4
N 58100 43750 57100 43750 4
N 57100 43750 57100 43700 4
N 58100 42100 58100 40900 4
N 57100 40800 57100 41200 4
C 57900 43750 1 270 1 schottky.sym
{
T 58500 44150 5 10 0 0 270 6 1
device=Schottky
T 58750 44200 5 10 1 1 0 6 1
refdes=D51
}
N 58100 44650 58100 45100 4
N 53100 45100 58600 45100 4
C 56000 45100 1 270 0 capacitor-1.sym
{
T 56700 44900 5 10 0 0 270 0 1
device=CAPACITOR
T 56900 44900 5 10 0 0 270 0 1
symversion=0.1
T 55700 44700 5 10 1 1 0 0 1
refdes=C52
T 56400 44500 5 10 1 1 0 0 1
value=1 uF
}
N 56200 44200 56200 43700 4
N 56200 43900 55800 43900 4
N 55800 40000 55800 43900 4
C 56500 39700 1 0 1 com.sym
N 54400 45100 54400 44900 4
C 54300 44900 1 270 0 resistor-1.sym
{
T 54700 44600 5 10 0 0 270 0 1
device=RESISTOR
T 54300 44900 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 54200 44100 5 10 1 1 90 0 1
refdes=R51
T 54700 44300 5 10 1 1 90 0 1
value=2.2 M
}
C 54300 42900 1 270 0 resistor-1.sym
{
T 54700 42600 5 10 0 0 270 0 1
device=RESISTOR
T 54300 42900 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 54200 42100 5 10 1 1 90 0 1
refdes=R52
T 54700 42300 5 10 1 1 90 0 1
value=160 k
}
N 53100 44000 55300 44000 4
N 55300 44000 55300 43700 4
N 54400 44000 54400 42900 4
N 54400 42000 54400 40000 4
N 55300 41200 53800 41200 4
C 53800 41300 1 180 0 resistor-1.sym
{
T 53500 40900 5 10 0 0 180 0 1
device=2013M R 220 1%
T 53800 41300 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 53000 41400 5 10 1 1 0 0 1
refdes=R53
T 53200 40900 5 10 1 1 0 0 1
value=220
}
N 52500 40800 52500 42300 4
C 55200 41100 1 270 0 resistor-1.sym
{
T 55600 40800 5 10 0 0 270 0 1
device=RESISTOR
T 55200 41100 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 55100 40300 5 10 1 1 90 0 1
refdes=R54
T 55600 40500 5 10 1 1 90 0 1
value=10 k
}
N 55300 41100 55300 41200 4
N 55300 40200 55300 40000 4
C 52900 45100 1 270 0 capacitor-1.sym
{
T 53600 44900 5 10 0 0 270 0 1
device=CAPACITOR
T 53800 44900 5 10 0 0 270 0 1
symversion=0.1
T 52600 44700 5 10 1 1 0 0 1
refdes=C5FF
T 53300 44500 5 10 1 1 0 0 1
value=1 uF
}
N 53100 44200 53100 44000 4
C 62200 45000 1 0 1 com.sym
N 62900 45300 62100 45300 4
N 62900 45000 62600 45000 4
N 62600 45000 62600 45300 4
C 62200 44100 1 0 1 com.sym
N 62900 44400 62100 44400 4
N 62900 44100 62600 44100 4
N 62600 44100 62600 44400 4
N 51600 46600 49300 46600 4
{
T 51300 46650 5 10 1 1 0 6 1
netname=PB15_RTC_REFIN
}
N 62900 44700 59500 44700 4
{
T 62500 44750 5 10 1 1 0 6 1
netname=PA1_TIM5_CH2
}
N 62900 43800 59500 43800 4
{
T 62500 43850 5 10 1 1 0 6 1
netname=PA0_TIM2_CH1/TIM2_ETR
}
N 62900 45600 59500 45600 4
{
T 62700 45650 5 10 1 1 0 6 1
netname=PA2_USART2_TX_TIM2.3_TIM5.3_TIM9.1
}
N 62900 45900 59500 45900 4
{
T 62700 45950 5 10 1 1 0 6 1
netname=PA3_USART2_RX_TIM2.4_TIM5.4_TIM9.2
}
N 62900 46200 59500 46200 4
{
T 62400 46250 5 10 1 1 0 6 1
netname=PA4_ADC1.4_HV_sense
}
N 62900 46500 59500 46500 4
{
T 62300 46550 5 10 1 1 0 6 1
netname=PA5_ADC1.5_TIM2.1
}
C 62200 48000 1 0 1 com.sym
N 62900 48300 62100 48300 4
N 62900 48000 62600 48000 4
N 62600 48000 62600 48300 4
N 62900 47700 61500 47700 4
N 62900 47400 62600 47400 4
N 62600 47400 62600 47700 4
C 62200 46800 1 0 1 com.sym
N 62900 47100 62100 47100 4
N 62900 46800 62600 46800 4
N 62600 46800 62600 47100 4
C 61100 43300 1 180 0 nc-right-1.sym
{
T 61000 42800 5 10 0 0 180 0 1
value=NoConnection
T 61000 42600 5 10 0 0 180 0 1
device=DRC_Directive
}
C 61100 43000 1 180 0 nc-right-1.sym
{
T 61000 42500 5 10 0 0 180 0 1
value=NoConnection
T 61000 42300 5 10 0 0 180 0 1
device=DRC_Directive
}
C 61700 47700 1 0 1 power_+.sym
{
T 61800 47900 5 10 1 1 0 6 1
net=20V_IN:1
}
N 58400 47100 58400 46900 4
N 57200 46900 58400 46900 4
N 57200 46900 57200 46500 4
N 52500 49500 52800 49500 4
N 49300 47200 51900 47200 4
{
T 49400 47250 5 10 1 1 0 0 1
netname=PA9_OTG_FS_VBUS
}
N 49300 46900 52200 46900 4
{
T 49400 46950 5 10 1 1 0 0 1
netname=PA8_MCO_1_TIM1.1
}
N 54100 47700 55600 47700 4
N 53000 47700 53200 47700 4
N 49300 45700 51900 45700 4
N 51900 41200 51900 45700 4
N 52900 41200 51900 41200 4
C 47800 39200 1 0 0 tp5000_charger.sym
{
T 52600 41200 5 10 0 0 0 0 1
footprint=QFN16-min
T 52600 41600 5 10 0 0 0 0 1
device=TP5000
T 50500 42700 5 14 1 1 0 3 1
refdes=U6
}
N 50700 41100 51700 41100 4
N 51700 38600 51700 41100 4
N 49300 39200 49300 38600 4
N 49000 38600 51700 38600 4
N 49000 39200 49000 38600 4
T 40800 57300 9 12 1 0 0 0 1
fix this with new regulator
C 51200 39900 1 270 0 capacitor-1.sym
{
T 51900 39700 5 10 0 0 270 0 1
device=CAPACITOR
T 52100 39700 5 10 0 0 270 0 1
symversion=0.1
T 50900 39500 5 10 1 1 0 0 1
refdes=C61
T 50800 39200 5 10 1 1 0 0 1
value=0.1 uF
T 51200 39900 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
N 51400 39000 51400 38600 4
N 50700 40500 51400 40500 4
N 51400 40500 51400 39900 4
C 44900 51900 1 90 0 capacitor-1.sym
{
T 44200 52100 5 10 0 0 90 0 1
device=CAPACITOR
T 44000 52100 5 10 0 0 90 0 1
symversion=0.1
T 45100 52200 5 10 1 1 180 0 1
refdes=C21
T 44600 52100 5 10 0 1 90 0 1
footprint=CAPC1608N.lht
}
N 44700 53000 44700 52800 4
N 47600 44500 47600 44700 4
N 48200 44100 48200 44500 4
N 44900 44000 44900 44700 4
N 45200 44700 45200 43700 4
N 44600 44700 44600 44300 4
C 39200 45600 1 180 1 ffc-zif-20.sym
{
T 39300 39000 5 10 1 1 180 6 1
refdes=J2
}
C 50400 50300 1 0 0 Vdd.sym
{
T 50400 50500 5 10 1 1 0 0 1
net=3.3V:1
}
C 45500 54300 1 0 1 switch-pushbutton-no-1.sym
{
T 45100 54900 5 10 0 0 180 2 1
device=SWITCH_PUSHBUTTON_NO
T 45100 54600 5 10 1 1 180 2 1
refdes=S1
}
N 45500 54300 45600 54300 4
N 45600 54300 45600 54100 4
C 47000 52900 1 0 0 JTAG_5x2.sym
{
T 46800 54400 5 10 0 1 0 0 1
device=HEADER10
T 48050 54550 5 10 1 1 180 0 1
refdes=J3
}
C 58200 48000 1 270 0 capacitor-1.sym
{
T 58900 47800 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 57900 47600 5 10 1 1 0 0 1
refdes=C41
T 58600 47400 5 10 1 1 0 0 1
value=4.7 uF
T 58200 48000 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
N 47500 51400 47500 52500 4
N 47500 52500 48700 52500 4
N 48700 52500 48700 53600 4
N 48300 53600 48700 53600 4
N 47200 51400 47200 52700 4
N 47200 52700 48300 52700 4
N 48300 52700 48300 53000 4
N 48100 51400 48100 52100 4
N 48100 52100 48900 52100 4
N 48900 52100 48900 53900 4
N 48900 53900 48300 53900 4
N 49300 48400 49600 48400 4
N 49600 48400 49600 52300 4
N 49100 54200 48300 54200 4
C 47000 53400 1 180 0 nc-right-1.sym
{
T 46900 52900 5 10 0 0 180 0 1
value=NoConnection
T 46900 52700 5 10 0 0 180 0 1
device=DRC_Directive
}
N 47000 53900 47000 53600 4
N 46400 53600 47000 53600 4
N 46400 53600 46400 53000 4
C 46800 54600 1 0 0 Vdd.sym
{
T 46800 54800 5 10 1 1 0 0 1
net=3.3V:1
}
N 47000 54600 47000 54200 4
N 46900 51400 46900 53000 4
N 52300 55600 54600 55600 4
{
T 53000 55650 5 10 1 1 0 0 1
netname=PA10_OTG_FS_ID
}
N 49100 52300 49600 52300 4
N 49100 52300 49100 54200 4
C 60300 53400 1 270 0 capacitor-1.sym
{
T 61000 53200 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 60000 53000 5 10 1 1 0 0 1
refdes=C31
T 60700 52800 5 10 1 1 0 0 1
value=4.7 uF
T 60300 53400 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
C 57900 40900 1 270 0 capacitor-1.sym
{
T 58600 40700 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 57600 40500 5 10 1 1 0 0 1
refdes=C51
T 58300 40300 5 10 1 1 0 0 1
value=4.7 uF
T 57900 40900 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
C 54100 47800 1 180 0 resistor-1.sym
{
T 53800 47400 5 10 0 0 180 0 1
device=2013M R 220 1%
T 54100 47800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 53300 47900 5 10 1 1 0 0 1
refdes=R43
T 53500 47400 5 10 1 1 0 0 1
value=220
}
C 56200 53800 1 180 0 resistor-1.sym
{
T 55900 53400 5 10 0 0 180 0 1
device=2013M R 220 1%
T 56200 53800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 55400 53900 5 10 1 1 0 0 1
refdes=R33
T 55600 53400 5 10 1 1 0 0 1
value=220
}
C 57600 53600 1 270 0 resistor-1.sym
{
T 58000 53300 5 10 0 0 270 0 1
device=2013M R 10k 1%
T 57600 53600 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 57500 52800 5 10 1 1 90 0 1
refdes=R34
T 58000 53000 5 10 1 1 90 0 1
value=10 k
}
C 55500 47600 1 270 0 resistor-1.sym
{
T 55900 47300 5 10 0 0 270 0 1
device=2013M R 10k 1%
T 55500 47600 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 55400 46800 5 10 1 1 90 0 1
refdes=R44
T 55900 47000 5 10 1 1 90 0 1
value=10 k
}
C 56700 55400 1 270 0 resistor-1.sym
{
T 57100 55100 5 10 0 0 270 0 1
device=2013M R 160k 1%
T 56700 55400 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 56600 54600 5 10 1 1 90 0 1
refdes=R32
T 57100 54800 5 10 1 1 90 0 1
value=160 k
}
C 53900 53200 1 90 0 jump-solder.sym
{
T 53500 53600 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 53500 53700 5 10 1 1 90 0 1
refdes=J5
T 53900 53200 5 10 0 0 0 0 1
footprint=jump-solder.lht
}
C 52600 48600 1 0 0 jump-solder.sym
{
T 53000 49000 5 8 0 0 0 0 1
device=JUMPER SOLDER
T 53100 49000 5 10 1 1 0 0 1
refdes=J6
T 52600 48600 5 10 0 0 270 0 1
footprint=jump-solder.lht
}
C 52300 42300 1 0 0 jump-solder.sym
{
T 52700 42700 5 8 0 0 0 0 1
device=JUMPER SOLDER
T 52800 42700 5 10 1 1 0 0 1
refdes=J7
T 52300 42300 5 10 0 0 270 0 1
footprint=jump-solder.lht
}
C 44700 53300 1 90 0 capacitor-1.sym
{
T 44000 53500 5 10 0 0 90 0 1
device=CAPACITOR
T 43800 53500 5 10 0 0 90 0 1
symversion=0.1
T 44900 53600 5 10 1 1 180 0 1
refdes=C22
T 44400 53800 5 10 0 1 90 0 1
footprint=CAPC1608N.lht
}
N 44500 53000 44500 53300 4
N 44500 54200 44500 54300 4
N 44700 51900 44700 51400 4
N 40500 44600 43700 44600 4
{
T 40850 44650 5 10 1 1 0 0 1
netname=RTC_TAMP1, RTC_OUT, RTC_TS
}
N 47600 44500 48200 44500 4
C 49100 44700 1 180 0 capacitor-1.sym
{
T 48900 44000 5 10 0 0 180 0 1
device=CAPACITOR
T 48900 43800 5 10 0 0 180 0 1
symversion=0.1
T 48800 44600 5 10 1 1 0 0 1
refdes=C4
T 49100 44700 5 10 0 0 90 0 1
footprint=CAPC1608N.lht
}
N 49100 44500 49100 44800 4
C 48100 43800 1 0 0 com.sym
N 48200 44800 48200 44700 4
N 48200 44700 47900 44700 4
N 45500 44700 45500 43400 4
N 45500 43400 40500 43400 4
{
T 41200 43450 5 10 1 1 0 6 1
netname=PA6
}
N 45800 44700 45800 43100 4
N 45800 43100 40500 43100 4
{
T 41200 43150 5 10 1 1 0 6 1
netname=PA7
}
N 46100 44700 46100 42800 4
N 46100 42800 40500 42800 4
{
T 41200 42850 5 10 1 1 0 6 1
netname=PB0
}
N 46400 44700 46400 42500 4
N 46400 42500 40500 42500 4
{
T 41200 42550 5 10 1 1 0 6 1
netname=PB1
}
N 40500 42200 46700 42200 4
{
T 40850 42250 5 10 1 1 0 0 1
netname=PB2
}
N 46700 42200 46700 44700 4
N 47000 44700 47000 41900 4
N 47000 41900 40500 41900 4
{
T 41300 41950 5 10 1 1 0 6 1
netname=PB10
}
C 43400 44700 1 0 0 stm32f401ce.sym
{
T 49300 52200 5 10 0 0 0 0 1
device=STM32F401CE
T 48500 51200 5 20 1 1 0 3 1
refdes=U2
}
C 42200 47300 1 0 0 capacitor-1.sym
{
T 42400 48000 5 10 0 0 0 0 1
device=CAPACITOR
T 42400 48200 5 10 0 0 0 0 1
symversion=0.1
T 42500 47700 5 10 1 1 180 0 1
refdes=C25
T 42200 47300 5 10 1 1 0 0 1
footprint=CAPC1608N.lht
}
N 43400 47500 43100 47500 4
N 40300 47200 43400 47200 4
N 42900 49600 43400 49600 4
N 42200 49300 43400 49300 4
N 42200 47200 42200 47500 4
C 41200 48900 1 0 0 capacitor-1.sym
{
T 41400 49600 5 10 0 0 0 0 1
device=CAPACITOR
T 41400 49800 5 10 0 0 0 0 1
symversion=0.1
T 41200 48900 5 10 0 0 0 0 1
footprint=0805.fp
T 41500 49100 5 10 1 1 180 0 1
refdes=C6
T 41700 49200 5 10 1 1 0 0 1
value=7 pF
}
N 43000 48400 43400 48400 4
N 43400 48700 43400 49000 4
N 43400 49000 43000 49000 4
C 41200 48300 1 0 0 capacitor-1.sym
{
T 41400 49000 5 10 0 0 0 0 1
device=CAPACITOR
T 41400 49200 5 10 0 0 0 0 1
symversion=0.1
T 41200 48300 5 10 0 0 0 0 1
footprint=0805.fp
T 41500 48400 5 10 1 1 180 0 1
refdes=C7
T 41100 48600 5 10 1 1 0 0 1
value=7 pF
}
N 43400 48100 41600 48100 4
{
T 42900 47950 5 10 1 1 0 6 1
netname=NRESET
}
N 42100 48200 42100 48500 4
N 41200 48500 40900 48500 4
N 40900 48500 40900 50100 4
N 40900 49100 41200 49100 4
N 42000 50100 42600 50100 4
C 41100 49400 1 0 0 capacitor-1.sym
{
T 41300 50100 5 10 0 0 0 0 1
device=CAPACITOR
T 41300 50300 5 10 0 0 0 0 1
symversion=0.1
T 41100 49400 5 10 0 0 0 0 1
footprint=0805.fp
T 41400 49500 5 10 1 1 180 0 1
refdes=C8
T 41000 49700 5 10 1 1 0 0 1
value=9 pF
}
C 41100 49900 1 0 0 capacitor-1.sym
{
T 41300 50600 5 10 0 0 0 0 1
device=CAPACITOR
T 41300 50800 5 10 0 0 0 0 1
symversion=0.1
T 41100 49900 5 10 0 0 0 0 1
footprint=0805.fp
T 41400 50000 5 10 1 1 180 0 1
refdes=C9
T 41000 50200 5 10 1 1 0 0 1
value=9 pF
}
N 40900 49600 41100 49600 4
N 43400 46600 40000 46600 4
{
T 43000 46650 5 10 1 1 0 6 1
netname=PA1_TIM5_CH2
}
N 43400 46900 40000 46900 4
{
T 43000 46950 5 10 1 1 0 6 1
netname=PA0_TIM2_CH1/TIM2_ETR
}
N 43400 46300 39500 46300 4
{
T 43200 46350 5 10 1 1 0 6 1
netname=PA2_USART2_TX_TIM2.3_TIM5.3_TIM9.1
}
N 41900 47800 43100 47800 4
N 43100 47800 43100 47500 4
C 41300 47400 1 0 0 com.sym
N 41900 47700 41900 47800 4
C 40600 48600 1 0 0 com.sym
N 40600 47700 41900 47700 4
C 41600 47900 1 0 1 switch-pushbutton-no-1.sym
{
T 41200 48500 5 10 0 0 180 2 1
device=SWITCH_PUSHBUTTON_NO
T 40800 48300 5 10 1 1 0 2 1
refdes=S2
}
N 41600 48100 41600 47900 4
N 40600 47700 40600 47900 4
C 40300 49000 1 270 0 coil-1.sym
{
T 40700 48800 5 10 0 0 270 0 1
device=COIL
T 40900 48800 5 10 0 0 270 0 1
symversion=0.1
T 40000 48500 5 10 1 1 0 0 1
refdes=L1
}
N 40300 47200 40300 48000 4
C 42400 49900 1 270 0 crystal-2.sym
{
T 43100 49800 5 10 0 0 270 0 1
device=CRYSTAL
T 42400 49900 5 10 0 0 270 0 1
footprint=1812.fp
T 42426 49544 5 10 1 1 180 0 1
refdes=X2
T 42600 49900 5 10 1 1 180 0 1
value=32.78kHz
}
N 42600 49900 42600 50100 4
C 43300 48400 1 90 0 crystal-gnd-4.sym
{
T 42500 48500 5 10 0 0 90 0 1
device=FA-20H
T 43300 48400 5 10 0 1 0 0 1
footprint=XTALCC2520.lht
T 42626 48744 5 10 1 1 180 0 1
refdes=X1
}
N 40700 48900 40900 48900 4
N 43000 48400 43000 48200 4
N 42100 48200 43000 48200 4
N 42100 49100 43000 49100 4
N 43000 49100 43000 49000 4
N 42900 49900 42900 49600 4
N 42900 49900 42600 49900 4
N 42700 49000 42200 49000 4
N 42200 49000 42200 48800 4
N 40900 48800 42300 48800 4
N 42700 48400 42300 48400 4
N 42300 48400 42300 48800 4
N 40900 50100 41100 50100 4
N 42200 49300 42200 49600 4
N 42200 49600 42000 49600 4
N 40100 50900 43100 50900 4
{
T 40200 50950 5 10 1 1 0 0 1
netname=RTC_TAMP1, RTC_OUT, RTC_TS
}
N 43100 49900 43400 49900 4
N 43100 49900 43100 50900 4
N 43400 50200 43400 51400 4
N 40300 49000 40300 51400 4
N 51000 56500 51400 56500 4
N 51000 56100 51200 56100 4
N 51200 56100 51200 55600 4
N 49300 48100 51100 48100 4
{
T 49400 48150 5 10 1 1 0 0 1
netname=PA12_OTG_FS_DP
}
N 51400 56000 51400 56300 4
N 51000 57300 51900 57300 4
{
T 50200 57350 5 10 1 1 0 0 1
netname=PA9_OTG_FS_VBUS
}
C 40000 57000 1 0 1 Vdd.sym
{
T 39800 57250 5 10 1 1 0 3 1
net=USB_C_5V:1
}
C 60400 51600 1 0 1 power_+.sym
{
T 60900 51800 5 10 1 1 0 6 1
net=20V_ONBOARD:1
}
C 56000 45100 1 0 1 power_+.sym
{
T 56500 45300 5 10 1 1 0 6 1
net=20V_ONBOARD:1
}
C 52300 55700 1 180 0 resistor-1.sym
{
T 52000 55300 5 10 0 0 180 0 1
device=2013M R 22 1%
T 52300 55700 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 51400 55800 5 10 1 1 0 0 1
refdes=R7
T 51700 55300 5 10 1 1 0 0 1
value=22
}
C 52800 57400 1 180 0 resistor-1.sym
{
T 52500 57000 5 10 0 0 180 0 1
device=2013M R 1.5 k 1%
T 52800 57400 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 52000 57500 5 10 1 1 0 0 1
refdes=R9
T 52200 57000 5 10 1 1 0 0 1
value=1.5 k
}
N 53500 56000 53500 57300 4
N 52800 57300 53500 57300 4
C 52300 56600 1 180 0 resistor-1.sym
{
T 52000 56200 5 10 0 0 180 0 1
device=2013M R 22 1%
T 52300 56600 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 51500 56700 5 10 1 1 0 0 1
refdes=R8
T 51700 56200 5 10 1 1 0 0 1
value=22
}
N 51200 55600 51400 55600 4
N 53000 56000 55600 56000 4
{
T 53900 56050 5 10 1 1 0 0 1
netname=PA12_OTG_FS_DP
}
C 53000 56100 1 180 0 resistor-1.sym
{
T 52700 55700 5 10 0 0 180 0 1
device=2013M R 22 1%
T 53000 56100 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 52200 56200 5 10 1 1 0 0 1
refdes=R6
T 52400 55700 5 10 1 1 0 0 1
value=22
}
N 52100 56000 51400 56000 4
N 52500 57700 51100 57700 4
N 40500 39800 43400 39800 4
{
T 40600 39850 5 10 1 1 0 0 1
netname=PA8_MCO_1_TIM1.1
}
N 53000 47700 53000 46300 4
N 53000 46300 49300 46300 4
N 47800 51400 47800 52300 4
N 48300 53300 48500 53300 4
N 48500 52300 48500 53300 4
N 48500 52300 47800 52300 4
T 43900 57400 9 18 1 0 0 0 1
Cortex Debug Connector J3
L 45150 57100 45150 56700 3 10 1 0 -1 -1
L 45750 57100 45750 55700 3 10 1 0 -1 -1
V 45300 57000 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45300 56700 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45600 57000 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45600 56700 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45600 56400 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45300 56400 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45600 56100 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45300 56100 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45600 55800 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 45300 55800 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
L 45150 56100 45150 55700 3 10 1 0 -1 -1
C 46500 54600 1 270 0 resistor-1.sym
{
T 46900 54300 5 10 0 0 270 0 1
device=2013M R 22 1%
T 46500 54600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 46400 53800 5 10 1 1 90 0 1
refdes=R8
T 46900 54000 5 10 1 1 90 0 1
value=22
}
N 47000 54600 46600 54600 4
N 46600 53700 46600 53000 4
N 46600 53000 47000 53000 4
T 43800 56400 9 15 1 0 0 1 5
      VCC
      GND
      GND
      KEY
GNDDet
L 45150 57100 45000 57100 3 10 1 0 -1 -1
L 45000 57100 45000 56700 3 10 1 0 -1 -1
L 45150 56700 45000 56700 3 10 1 0 -1 -1
L 45150 56100 45000 56100 3 10 1 0 -1 -1
L 45000 56100 45000 55700 3 10 1 0 -1 -1
L 45000 55700 45150 55700 3 10 1 0 -1 -1
L 45750 55700 45900 55700 3 10 1 0 -1 -1
L 45900 55700 45900 57100 3 10 1 0 -1 -1
L 45900 57100 45750 57100 3 10 1 0 -1 -1
T 46100 56400 9 15 1 0 0 1 5
SWDIO/TMS
SWDCLK/TCK
SWO/TDO
NC/TDI
nRESET
