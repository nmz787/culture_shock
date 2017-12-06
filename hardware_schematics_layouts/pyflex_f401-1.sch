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
N 45100 51400 45100 53000 4
N 44200 54300 44500 54300 4
N 50600 49000 50600 50300 4
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
N 49700 44800 48200 44800 4
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
C 64500 45400 1 0 1 ffc-zif-20.sym
{
T 64400 52000 5 10 1 1 0 6 1
refdes=J1
}
N 44600 44300 42800 44300 4
{
T 43750 44350 5 10 1 1 0 6 1
netname=PA3_TIM5.4
}
N 44900 44000 42800 44000 4
{
T 44750 44050 5 10 1 1 0 6 1
netname=PA4_ADC1.4_HV_sense
}
T 39700 51900 9 12 1 0 0 0 3
VBAT could get a backup battery 
for implementing real time clock
if timestamping becomes popular.
L 43000 51800 43700 51100 3 10 1 0 -1 -1
B 39600 51800 3400 800 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
N 49300 47800 51500 47800 4
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
C 49300 46500 1 0 0 nc-right-1.sym
{
T 49400 47000 5 10 0 0 0 0 1
value=NoConnection
T 49400 47200 5 10 0 0 0 0 1
device=DRC_Directive
}
N 63200 46700 62700 46700 4
C 62500 45500 1 0 1 com.sym
N 63200 45800 62400 45800 4
N 62700 46700 62700 45800 4
N 50500 57300 50500 56700 4
C 49300 55700 1 0 0 USB_SMT_5p.sym
{
T 49300 56950 5 10 1 1 0 0 1
refdes=J4
T 49590 56010 5 16 0 1 90 0 1
device=microUSB_AB_antek
T 49600 68150 5 10 0 0 0 0 1
footprint=microUSB_AB_1.fp
}
N 51800 56500 54100 56500 4
{
T 52400 56550 5 10 1 1 0 0 1
netname=PA11_OTG_FS_DM
}
N 50500 56300 50900 56300 4
C 44000 54600 1 0 0 Vdd.sym
{
T 44050 54850 5 10 1 1 0 0 1
net=3V3:1
}
N 42000 56100 42000 56700 4
N 41000 56700 42900 56700 4
C 42200 55200 1 90 0 capacitor-1.sym
{
T 41500 55400 5 10 0 0 90 0 1
device=MF-CAP-0603-1uF
T 41300 55400 5 10 0 0 90 0 1
symversion=0.1
T 42200 55200 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 42200 55200 5 10 0 0 0 0 1
description=1 uF X7R 25V
T 41600 55800 5 10 1 1 0 0 1
refdes=C11
T 42100 55400 5 10 1 1 0 0 1
value=1 uF
}
C 43100 55200 1 90 0 capacitor-1.sym
{
T 42400 55400 5 10 0 0 90 0 1
device=MF-CAP-0603-0.1uF
T 42200 55400 5 10 0 0 90 0 1
symversion=0.1
T 43100 55200 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 43100 55200 5 10 0 0 0 0 1
description=Capacitor MLCC 0603 0.1uF 10% 25V
T 42500 55800 5 10 1 1 0 0 1
refdes=C10
T 43000 55400 5 10 1 1 0 0 1
value=0.1 uF
}
N 42900 56100 42900 56700 4
C 41600 55200 1 90 0 capacitor-1.sym
{
T 40900 55400 5 10 0 0 90 0 1
device=MF-CAP-0603-1uF
T 40700 55400 5 10 0 0 90 0 1
symversion=0.1
T 41600 55200 5 10 0 0 0 0 1
footprint=chip_1608_0603_N.fp
T 41600 55200 5 10 0 0 0 0 1
description=1 uF X7R 25V
T 41000 55800 5 10 1 1 0 0 1
refdes=C12
T 41500 55400 5 10 1 1 0 0 1
value=1 uF
}
N 40500 55200 42900 55200 4
C 42300 56700 1 0 0 Vdd.sym
{
T 42300 56900 5 10 1 1 0 0 1
net=3V3:1
}
N 41400 56100 42000 56100 4
C 42100 54900 1 0 0 com.sym
N 40500 55200 40500 56300 4
N 40000 56700 39400 56700 4
N 39400 56700 39400 57100 4
C 40000 56300 1 0 0 regulator_sot23.sym
{
T 40000 58400 5 8 0 0 0 0 1
symversion=3.0
T 41000 56900 5 10 1 1 0 0 1
value=3.3
T 40100 57400 5 10 1 1 0 0 1
refdes=U1
T 40500 56200 5 8 1 1 0 0 1
footprint=SOT23-123.fp
T 40080 57149 5 10 1 1 0 0 1
device=AP2120N-3.3TRG1
}
C 55300 48700 1 0 0 dc-dc-sot23-5.sym
{
T 57700 51000 5 14 1 1 0 3 1
refdes=U4
T 55800 49900 5 10 1 1 0 0 1
device=G5111T11UF
T 55800 49300 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 50500 55500 50500 55900 4
N 54400 47500 58400 47500 4
N 54600 48000 57400 48000 4
C 58400 50300 1 270 0 coil-1.sym
{
T 58800 50100 5 10 0 0 270 0 1
device=COIL
T 59000 50100 5 10 0 0 270 0 1
symversion=0.1
T 58600 49900 5 10 1 1 0 0 1
refdes=L41
}
N 58400 50300 58400 51400 4
N 58400 51400 57400 51400 4
N 58400 49300 58400 49000 4
N 57400 48000 57400 48700 4
C 58200 51400 1 270 1 schottky.sym
{
T 58800 51800 5 10 0 0 270 6 1
device=Schottky
T 59050 51750 5 10 1 1 0 6 1
refdes=D41
}
N 53400 52300 58400 52300 4
{
T 56600 52350 5 10 1 1 0 0 1
netname=18V_ONBOARD
}
C 56300 52300 1 270 0 capacitor-1.sym
{
T 57000 52100 5 10 0 0 270 0 1
device=CAPACITOR
T 57200 52100 5 10 0 0 270 0 1
symversion=0.1
T 56000 51900 5 10 1 1 0 0 1
refdes=C42
T 56700 51700 5 10 1 1 0 0 1
value=1 uF
}
N 56500 51400 56500 51200 4
N 56500 51400 56100 51400 4
N 56100 47500 56100 51400 4
C 57300 47200 1 0 1 com.sym
N 54700 52300 54700 52100 4
C 54600 52100 1 270 0 resistor-1.sym
{
T 55000 51800 5 10 0 0 270 0 1
device=2013M R 2.2 M 1%
T 54600 52100 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 54500 51300 5 10 1 1 90 0 1
refdes=R41
T 55000 51500 5 10 1 1 90 0 1
value=2.2 M
}
C 54600 50100 1 270 0 resistor-1.sym
{
T 55000 49800 5 10 0 0 270 0 1
device=2013M R 160 k 1%
T 54600 50100 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 54500 49300 5 10 1 1 90 0 1
refdes=R42
T 55000 49500 5 10 1 1 90 0 1
value=160 k
}
N 53400 51200 55600 51200 4
N 54700 51200 54700 50100 4
N 54700 49200 54700 47500 4
N 55600 48600 55600 48700 4
N 55600 47700 55600 47500 4
C 53200 52300 1 270 0 capacitor-1.sym
{
T 53900 52100 5 10 0 0 270 0 1
device=CAPACITOR
T 54100 52100 5 10 0 0 270 0 1
symversion=0.1
T 52900 51200 5 10 1 1 0 0 1
refdes=C4FF
T 53600 51700 5 10 1 1 0 0 1
value=1 uF
}
N 53400 51400 53400 51200 4
C 62500 48200 1 0 1 com.sym
N 63200 48500 62400 48500 4
N 63200 48200 62900 48200 4
N 62900 48200 62900 48500 4
C 62500 47300 1 0 1 com.sym
N 63200 47600 62400 47600 4
N 63200 47300 62900 47300 4
N 62900 47300 62900 47600 4
N 63200 47900 60300 47900 4
{
T 62800 47950 5 10 1 1 0 6 1
netname=PA1_TIM5_CH2
}
N 63200 47000 60300 47000 4
{
T 62800 47050 5 10 1 1 0 6 1
netname=PA0_TIM2_CH1/TIM2_ETR
}
N 63200 49400 60300 49400 4
{
T 62700 49450 5 10 1 1 0 6 1
netname=PA4_ADC1.4_HV_sense
}
N 63200 49700 60300 49700 4
{
T 63000 49750 5 10 1 1 0 6 1
netname=PA5_ADC1.5_current_sense
}
C 62500 51200 1 0 1 com.sym
N 63200 51500 62400 51500 4
N 63200 51200 62900 51200 4
N 62900 51200 62900 51500 4
N 63200 50900 61300 50900 4
N 63200 50600 62900 50600 4
N 62900 50600 62900 50900 4
C 62500 50000 1 0 1 com.sym
N 63200 50300 62400 50300 4
N 63200 50000 62900 50000 4
N 62900 50000 62900 50300 4
N 53000 48000 53700 48000 4
N 49300 47200 51500 47200 4
{
T 49400 47250 5 10 1 1 0 0 1
netname=PA9_OTG_FS_VBUS
}
N 49300 46900 51500 46900 4
{
T 49400 46950 5 10 1 1 0 0 1
netname=MCO_1_PA8
}
N 54300 48700 55600 48700 4
N 56500 40700 57700 40700 4
{
T 57400 40750 5 10 1 1 0 6 1
netname=PB12
}
C 50500 39800 1 0 0 tp5000_charger.sym
{
T 55300 41800 5 10 0 0 0 0 1
footprint=QFN16-min
T 55300 42200 5 10 0 0 0 0 1
device=TP5000
T 53200 43300 5 14 1 1 0 3 1
refdes=U6
}
N 53400 41700 54400 41700 4
N 54400 39200 54400 41700 4
N 52000 39800 52000 39200 4
N 51700 39200 54400 39200 4
N 51700 39800 51700 39200 4
C 53900 40500 1 270 0 capacitor-1.sym
{
T 54600 40300 5 10 0 0 270 0 1
device=CAPACITOR
T 54800 40300 5 10 0 0 270 0 1
symversion=0.1
T 53600 40100 5 10 1 1 0 0 1
refdes=C61
T 53500 39800 5 10 1 1 0 0 1
value=0.1 uF
T 53900 40500 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
N 54100 39600 54100 39200 4
N 53400 41100 54100 41100 4
N 54100 41100 54100 40500 4
C 44700 51900 1 90 0 capacitor-1.sym
{
T 44000 52100 5 10 0 0 90 0 1
device=CAPACITOR
T 43800 52100 5 10 0 0 90 0 1
symversion=0.1
T 44900 52200 5 10 1 1 180 0 1
refdes=C21
T 44400 52100 5 10 0 1 90 0 1
footprint=CAPC1608N.lht
}
N 44500 52800 44500 53300 4
N 47600 44500 47600 44700 4
N 48200 43400 48200 44500 4
N 44900 44000 44900 44700 4
N 45200 44700 45200 43700 4
N 44600 44700 44600 44300 4
C 39200 45600 1 180 1 ffc-zif-20.sym
{
T 39600 45800 5 10 1 1 180 6 1
refdes=J2
}
C 50400 50300 1 0 0 Vdd.sym
{
T 50400 50500 5 10 1 1 0 0 1
net=3V3:1
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
T 47000 52900 5 10 0 0 0 0 1
footprint=2x5hdr_1.27.fp
}
C 58200 49000 1 270 0 capacitor-1.sym
{
T 58900 48800 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 57900 48600 5 10 1 1 0 0 1
refdes=C41
T 58600 48400 5 10 1 1 0 0 1
value=4.7 uF
T 58200 49000 5 10 0 0 0 0 1
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
net=3V3:1
}
N 47000 54600 47000 54200 4
N 46900 51400 46900 53000 4
N 51800 55600 54100 55600 4
{
T 52500 55650 5 10 1 1 0 0 1
netname=PA10_OTG_FS_ID
}
N 49100 52300 49600 52300 4
N 49100 52300 49100 54200 4
C 54300 48800 1 180 0 resistor-1.sym
{
T 54000 48400 5 10 0 0 180 0 1
device=2013M R 220 1%
T 54300 48800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 53500 48900 5 10 1 1 0 0 1
refdes=R43
T 53700 48400 5 10 1 1 0 0 1
value=220
}
C 55500 48600 1 270 0 resistor-1.sym
{
T 55900 48300 5 10 0 0 270 0 1
device=2013M R 10k 1%
T 55500 48600 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 55400 47800 5 10 1 1 90 0 1
refdes=R44
T 55900 48000 5 10 1 1 90 0 1
value=10 k
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
N 44500 54200 44500 54300 4
N 44500 51900 44500 51400 4
N 40500 44600 43700 44600 4
{
T 40850 44650 5 10 1 1 0 0 1
netname=PC13
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
C 48400 43500 1 0 0 com.sym
N 48200 44800 48200 44700 4
N 48200 44700 47900 44700 4
N 45500 44700 45500 43400 4
N 45500 43400 40500 43400 4
{
T 41900 43450 5 10 1 1 0 6 1
netname=PA6_ADC1.6
}
N 45800 44700 45800 43100 4
N 45800 43100 40500 43100 4
{
T 41900 43150 5 10 1 1 0 6 1
netname=PA7_ADC1.7
}
N 46100 44700 46100 42500 4
N 46100 42500 40500 42500 4
{
T 41200 42550 5 10 1 1 0 6 1
netname=PB0
}
N 46400 44700 46400 42200 4
N 46400 42200 40500 42200 4
{
T 41200 42250 5 10 1 1 0 6 1
netname=PB1
}
N 40500 41900 46700 41900 4
{
T 40850 41950 5 10 1 1 0 0 1
netname=PB2
}
N 46700 41900 46700 44700 4
N 47000 44700 47000 43400 4
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
T 41500 49000 5 10 1 1 180 0 1
refdes=C6
T 41800 49200 5 10 1 1 0 0 1
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
T 41800 48600 5 10 1 1 0 0 1
value=7 pF
}
N 43400 48100 41600 48100 4
{
T 42900 47950 5 10 1 1 0 6 1
netname=NRST
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
N 43400 46300 40000 46300 4
{
T 43200 46350 5 10 1 1 0 6 1
netname=PA2_USART2_TX_TIM5.3
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
N 42300 50500 43100 50500 4
{
T 42400 50550 5 10 1 1 0 0 1
netname=PC13
}
N 43100 49900 43400 49900 4
N 43100 49900 43100 50500 4
N 43400 50200 43400 50800 4
N 40300 49000 40300 51400 4
N 50500 56500 50900 56500 4
N 50500 56100 50700 56100 4
N 50700 56100 50700 55600 4
N 49300 48100 51500 48100 4
{
T 49400 48150 5 10 1 1 0 0 1
netname=PA12_OTG_FS_DP
}
N 50900 56000 50900 56300 4
N 50500 57300 51400 57300 4
{
T 49700 57350 5 10 1 1 0 0 1
netname=PA9_OTG_FS_VBUS
}
C 39600 57100 1 0 1 Vdd.sym
{
T 39400 57350 5 10 1 1 0 3 1
net=USB_C_5V:1
}
C 51800 55700 1 180 0 resistor-1.sym
{
T 51500 55300 5 10 0 0 180 0 1
device=2013M R 22 1%
T 51800 55700 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 50900 55800 5 10 1 1 0 0 1
refdes=R7
T 51200 55300 5 10 1 1 0 0 1
value=22
}
C 52300 57400 1 180 0 resistor-1.sym
{
T 52000 57000 5 10 0 0 180 0 1
device=2013M R 1.5 k 1%
T 52300 57400 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 51500 57500 5 10 1 1 0 0 1
refdes=R9
T 51700 57000 5 10 1 1 0 0 1
value=1.5 k
}
N 53000 56000 53000 57300 4
N 52300 57300 53000 57300 4
C 51800 56600 1 180 0 resistor-1.sym
{
T 51500 56200 5 10 0 0 180 0 1
device=2013M R 22 1%
T 51800 56600 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 51000 56700 5 10 1 1 0 0 1
refdes=R8
T 51200 56200 5 10 1 1 0 0 1
value=22
}
N 50700 55600 50900 55600 4
N 52500 56000 55100 56000 4
{
T 53400 56050 5 10 1 1 0 0 1
netname=PA12_OTG_FS_DP
}
C 52500 56100 1 180 0 resistor-1.sym
{
T 52200 55700 5 10 0 0 180 0 1
device=2013M R 22 1%
T 52500 56100 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 51700 56200 5 10 1 1 0 0 1
refdes=R6
T 51900 55700 5 10 1 1 0 0 1
value=22
}
N 51600 56000 50900 56000 4
N 50500 46300 49300 46300 4
{
T 50200 46350 5 10 1 1 0 6 1
netname=PB14
}
N 47800 51400 47800 52300 4
N 48300 53300 48500 53300 4
N 48500 52300 48500 53300 4
N 48500 52300 47800 52300 4
T 44600 57400 9 18 1 0 0 0 1
Cortex Debug Connector J3
L 45850 57100 45850 56700 3 10 1 0 -1 -1
L 46450 57100 46450 55700 3 10 1 0 -1 -1
V 46000 57000 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46000 56700 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46300 57000 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46300 56700 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46300 56400 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46000 56400 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46300 56100 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46000 56100 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46300 55800 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
V 46000 55800 50 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
L 45850 56100 45850 55700 3 10 1 0 -1 -1
C 46500 54600 1 270 0 resistor-1.sym
{
T 46900 54300 5 10 0 0 270 0 1
device=2013M R 22 1%
T 46500 54600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 46400 53800 5 10 1 1 90 0 1
refdes=R10
T 46900 54000 5 10 1 1 90 0 1
value=22
}
N 47000 54600 46600 54600 4
N 46600 53700 46600 53000 4
N 46600 53000 47000 53000 4
T 44500 56400 9 15 1 0 0 1 5
      VCC
      GND
      GND
      KEY
GNDDet
L 45850 57100 45700 57100 3 10 1 0 -1 -1
L 45700 57100 45700 56700 3 10 1 0 -1 -1
L 45850 56700 45700 56700 3 10 1 0 -1 -1
L 45850 56100 45700 56100 3 10 1 0 -1 -1
L 45700 56100 45700 55700 3 10 1 0 -1 -1
L 45700 55700 45850 55700 3 10 1 0 -1 -1
L 46450 55700 46600 55700 3 10 1 0 -1 -1
L 46600 55700 46600 57100 3 10 1 0 -1 -1
L 46600 57100 46450 57100 3 10 1 0 -1 -1
T 46800 56400 9 15 1 0 0 1 5
SWDIO/TMS
SWDCLK/TCK
SWO/TDO
NC/TDI
nRESET
N 45200 43700 42800 43700 4
{
T 45100 43750 5 10 1 1 0 6 1
netname=PA5_ADC1.5_current_sense
}
C 49500 44800 1 0 0 Vdd.sym
{
T 49500 45000 5 10 1 1 0 0 1
net=3V3:1
}
C 61500 50900 1 0 1 power_+.sym
{
T 62000 51100 5 10 1 1 0 6 1
net=18V_ONBOARD:1
}
N 61700 46100 63200 46100 4
{
T 62200 46150 5 10 1 1 0 6 1
netname=PB9
}
N 45400 51400 45400 52000 4
{
T 45350 51900 5 10 1 1 90 6 1
netname=PB9
}
N 45700 51400 45700 52000 4
{
T 45650 51900 5 10 1 1 90 6 1
netname=LED
}
N 61700 46400 63200 46400 4
N 40500 41300 42300 41300 4
{
T 42200 41350 5 10 1 1 0 6 1
netname=I2C1_SDA_PB7
}
N 40500 41000 42300 41000 4
{
T 42200 41050 5 10 1 1 0 6 1
netname=I2C1_SCL_PB6
}
C 41800 44900 1 0 0 com.sym
N 40500 45200 41900 45200 4
N 40500 44900 41100 44900 4
N 41100 44900 41100 45200 4
N 40500 40700 42500 40700 4
C 42400 40400 1 0 0 com.sym
N 57400 51200 57400 51400 4
N 58400 48100 58400 47500 4
C 59800 40700 1 0 0 dc-dc-sot23-5.sym
{
T 62200 43000 5 14 1 1 0 3 1
refdes=U5
T 60300 41900 5 10 1 1 0 0 1
device=AP3012KTR-G1
T 60300 41300 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 59200 39500 62900 39500 4
N 58900 40000 61900 40000 4
C 62900 42600 1 270 0 coil-1.sym
{
T 63300 42400 5 10 0 0 270 0 1
device=COIL
T 63500 42400 5 10 0 0 270 0 1
symversion=0.1
T 62500 42200 5 10 1 1 0 0 1
refdes=L51
}
N 62900 42600 62900 43250 4
N 62900 43250 61900 43250 4
N 61900 43250 61900 43200 4
N 62900 41600 62900 40400 4
N 61900 40000 61900 40700 4
C 62700 43250 1 270 1 schottky.sym
{
T 63300 43650 5 10 0 0 270 6 1
device=Schottky
T 63550 43700 5 10 1 1 0 6 1
refdes=D51
}
N 62900 44150 62900 44600 4
C 60800 44600 1 270 0 capacitor-1.sym
{
T 61500 44400 5 10 0 0 270 0 1
device=CAPACITOR
T 61700 44400 5 10 0 0 270 0 1
symversion=0.1
T 60500 44200 5 10 1 1 0 0 1
refdes=C52
T 61200 44000 5 10 1 1 0 0 1
value=1 uF
}
N 61000 43700 61000 43200 4
N 61000 43400 60600 43400 4
N 60600 39500 60600 43400 4
C 61300 39200 1 0 1 com.sym
N 59200 44600 59200 44400 4
C 59100 44400 1 270 0 resistor-1.sym
{
T 59500 44100 5 10 0 0 270 0 1
device=RESISTOR
T 59100 44400 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 59000 43600 5 10 1 1 90 0 1
refdes=R51
T 59500 43800 5 10 1 1 90 0 1
value=2.2 M
}
C 59100 42400 1 270 0 resistor-1.sym
{
T 59500 42100 5 10 0 0 270 0 1
device=RESISTOR
T 59100 42400 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 59000 41600 5 10 1 1 90 0 1
refdes=R52
T 59500 41800 5 10 1 1 90 0 1
value=160 k
}
N 58300 43500 60100 43500 4
N 60100 43500 60100 43200 4
N 59200 43500 59200 42400 4
N 59200 41500 59200 39500 4
N 60100 40700 58600 40700 4
C 58600 40800 1 180 0 resistor-1.sym
{
T 58300 40400 5 10 0 0 180 0 1
device=2013M R 220 1%
T 58600 40800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 57800 40900 5 10 1 1 0 0 1
refdes=R53
T 58000 40400 5 10 1 1 0 0 1
value=220
}
C 60000 40600 1 270 0 resistor-1.sym
{
T 60400 40300 5 10 0 0 270 0 1
device=RESISTOR
T 60000 40600 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 59900 39800 5 10 1 1 90 0 1
refdes=R54
T 60400 40000 5 10 1 1 90 0 1
value=10 k
}
N 60100 40600 60100 40700 4
N 60100 39700 60100 39500 4
C 58100 44600 1 270 0 capacitor-1.sym
{
T 58800 44400 5 10 0 0 270 0 1
device=CAPACITOR
T 59000 44400 5 10 0 0 270 0 1
symversion=0.1
T 57800 44200 5 10 1 1 0 0 1
refdes=C5FF
T 58500 44000 5 10 1 1 0 0 1
value=1 uF
}
N 58300 43700 58300 43500 4
C 62700 40400 1 270 0 capacitor-1.sym
{
T 63400 40200 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 62700 40400 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 62400 40000 5 10 1 1 0 0 1
refdes=C51
T 63100 39800 5 10 1 1 0 0 1
value=4.7 uF
}
C 58900 39800 1 90 0 jump-solder.sym
{
T 58500 40200 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 58900 39800 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 58600 40400 5 10 1 1 180 0 1
refdes=J7
}
N 58300 44600 62900 44600 4
{
T 61100 44650 5 10 1 1 0 0 1
netname=18V_ONBOARD
}
C 54600 47800 1 90 0 jump-solder.sym
{
T 54200 48200 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 54200 48400 5 10 1 1 180 0 1
refdes=J6
T 54600 47800 5 10 0 0 0 0 1
footprint=jump-solder.lht
}
C 57500 40100 1 0 1 Vdd.sym
{
T 57300 40350 5 10 1 1 0 3 1
net=USB_C_5V:1
}
N 57300 40100 57300 40000 4
N 57300 40000 58000 40000 4
C 48200 43400 1 0 1 switch-pushbutton-no-1.sym
{
T 47800 44000 5 10 0 0 180 2 1
device=SWITCH_PUSHBUTTON_NO
T 47400 43800 5 10 1 1 0 2 1
refdes=S3
}
N 47000 43400 47200 43400 4
N 48500 43800 48200 43800 4
C 43500 54000 1 270 0 resistor-1.sym
{
T 43900 53700 5 10 0 0 270 0 1
device=2013M R 1.5 k 1%
T 43500 54000 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 43400 53200 5 10 1 1 90 0 1
refdes=R11
T 43900 53400 5 10 1 1 90 0 1
value=1.680R
}
N 43600 54000 44200 54000 4
N 43600 52400 43600 53100 4
{
T 43550 52800 5 10 1 1 90 6 1
netname=LED
}
C 61700 46500 1 180 0 nc-right-1.sym
{
T 61600 46000 5 10 0 0 180 0 1
value=NoConnection
T 61600 45800 5 10 0 0 180 0 1
device=DRC_Directive
}
T 39700 53600 9 12 1 0 0 0 2
other regulator
goes here.
B 39500 53100 1700 1100 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
N 63200 48800 60300 48800 4
{
T 62900 48850 5 10 1 1 0 6 1
netname=PA2_USART2_TX_TIM5.3
}
N 43400 50800 42500 50800 4
{
T 43300 50850 5 10 1 1 0 6 1
netname=VBAT
}
N 63200 49100 60300 49100 4
{
T 61750 49150 5 10 1 1 0 6 1
netname=PA3_TIM5.4
}
N 40500 44300 41700 44300 4
{
T 41300 44350 5 10 1 1 0 6 1
netname=VBAT
}
N 46300 51400 46300 52800 4
{
T 46250 52800 5 10 1 1 90 6 1
netname=I2C1_SDA_PB7
}
N 46600 51400 46600 52900 4
{
T 46550 52800 5 10 1 1 90 6 1
netname=I2C1_SCL_PB6
}
N 40500 41600 41600 41600 4
{
T 40900 41650 5 10 1 1 0 0 1
netname=3V3
}
N 40500 43700 42200 43700 4
C 42100 43400 1 0 0 com.sym
N 40500 42800 42200 42800 4
C 42100 42500 1 0 0 com.sym
N 40500 39500 41900 39500 4
C 41800 39200 1 0 0 com.sym
N 40500 44000 41700 44000 4
{
T 40900 44150 5 10 1 1 180 6 1
netname=NRST
}
N 40500 40400 41900 40400 4
N 40500 40100 41900 40100 4
N 41900 40100 41900 40700 4
N 40500 39800 42300 39800 4
{
T 40900 39850 5 10 1 1 0 0 1
netname=MCO_1_PA8
}
C 50600 55200 1 0 1 com.sym
N 53400 48700 52700 48700 4
{
T 53300 48750 5 10 1 1 0 6 1
netname=PB14
}
N 50500 45700 49300 45700 4
{
T 50200 45750 5 10 1 1 0 6 1
netname=PB12
}
C 53200 48100 1 0 1 Vdd.sym
{
T 53000 48350 5 10 1 1 0 3 1
net=USB_C_5V:1
}
N 53000 48100 53000 48000 4
N 50500 46000 49300 46000 4
{
T 50200 46050 5 10 1 1 0 6 1
netname=PB13
}