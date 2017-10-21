v 20150930 2
N 58800 48300 59800 48300 4
{
T 59300 48400 5 10 1 1 0 0 1
netname=doubler_out
}
C 57900 48100 1 0 0 diode-1.sym
{
T 58300 48700 5 10 0 0 0 0 1
device=R3000-TP
T 58500 48700 5 10 1 1 180 0 1
refdes=D1
T 58800 48000 5 10 1 1 180 0 1
value=3KV
T 57900 48100 5 10 0 0 0 0 1
description=DIODE GEN PURP 3KV 200MA DO15 
T 57900 48100 5 10 0 0 0 0 1
footprint=diode_10mm.fp
}
C 59600 47100 1 270 0 capacitor-1.sym
{
T 60300 46900 5 10 0 0 270 0 1
device=472-3KV
T 60000 46800 5 10 1 1 0 0 1
refdes=C3
T 60500 46900 5 10 0 0 270 0 1
symversion=0.1
T 59300 46400 5 10 1 1 0 0 1
value=4.7nF
T 59600 47100 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat.fp
}
C 56300 48100 1 0 0 capacitor-1.sym
{
T 56500 48800 5 10 0 0 0 0 1
device=472-3KV
T 56800 48700 5 10 1 1 180 0 1
refdes=C1
T 56500 49000 5 10 0 0 0 0 1
symversion=0.1
T 57100 48000 5 10 1 1 180 0 1
value=4.7nF
T 56300 48100 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat.fp
}
N 57200 48300 57900 48300 4
{
T 57400 48400 5 10 1 1 0 0 1
netname=v2
}
N 57600 47400 57600 48300 4
N 57600 45200 57600 46500 4
N 59800 47100 59800 48300 4
C 56400 49000 1 0 0 capacitor-1.sym
{
T 56600 49700 5 10 0 0 0 0 1
device=472-3KV
T 56900 49600 5 10 1 1 180 0 1
refdes=C2
T 56600 49900 5 10 0 0 0 0 1
symversion=0.1
T 57200 48900 5 10 1 1 180 0 1
value=4.7nF
T 56400 49000 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat_flip.fp
}
N 59400 41000 62700 41000 4
N 59800 45200 59800 46200 4
N 59800 48000 62700 48000 4
N 57500 48300 57500 49200 4
N 56400 49200 56200 49200 4
N 56200 49200 56200 48300 4
N 57500 49200 57300 49200 4
C 37600 38200 0 0 0 title-A-cibolo.sym
{
T 58500 38200 5 10 1 1 0 0 1
file=kvboard.sch
T 62600 37900 5 10 1 1 0 0 1
drawn-by=John Griessen
T 50900 38000 5 18 1 1 0 0 1
title=Culture Shock kvboard PCB
T 58700 37900 5 10 1 1 0 0 1
first-pagenum=1
T 59400 37900 5 10 1 1 0 0 1
last-pagenum=1
T 62600 38200 5 10 1 1 0 0 1
rev=0.2    2017-8-11
T 64400 43100 5 18 1 1 270 0 1
title2=Culture Shock kvboard PCB
}
C 55500 46800 1 0 1 transformer-epc19.sym
{
T 55350 48400 5 10 1 1 0 6 1
refdes=T1
T 55500 48600 5 10 0 1 0 6 1
device=CS1-HVT-EPC19-03
T 55500 46800 5 10 0 0 0 0 1
description=1500V transformer
T 55500 46800 5 10 0 0 0 0 1
footprint=xfmr_epc19_10.fp
T 55500 46800 5 10 0 0 0 0 1
value=xfmr
}
C 60400 47500 1 0 0 resistor-1.sym
{
T 60700 47900 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 60500 47800 5 10 1 1 0 0 1
refdes=R1
T 60800 47300 5 10 1 1 0 0 1
value=475K
T 60400 47500 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 60400 47500 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 47500 1 0 0 resistor-1.sym
{
T 61900 47900 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 47800 5 10 1 1 0 0 1
refdes=R2
T 62000 47300 5 10 1 1 0 0 1
value=475K
T 61600 47500 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 61600 47500 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
N 54000 47900 54000 47300 4
T 58000 39600 9 12 1 0 0 0 3
2500V divided by 800 attenuation = 
3.125V, which is compatible with 
the 3.3V supply to the MCUs ADCs.
N 60400 46800 60400 46000 4
C 60500 41300 1 90 0 resistor-1.sym
{
T 60100 41600 5 10 0 0 90 0 1
device=RC0805FR-074K75L
T 60200 41500 5 10 1 1 90 0 1
refdes=R9
T 60700 41700 5 10 1 1 90 0 1
value=4.75K
T 60500 41300 5 10 0 0 0 0 1
description=4.75k Ohm ±1% 0.125W, 1/8W Chip Resistor 0805 (2012 Metric) 
T 60500 41300 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
}
N 55400 42600 60400 42600 4
C 55400 42700 1 180 0 resistor-1.sym
{
T 55100 42300 5 10 0 0 180 0 1
device=MF-RES-0805-220
T 53100 42900 5 10 1 1 180 0 1
refdes=R14
T 55400 42700 5 10 0 1 0 0 1
footprint=chip_2012_0805_N.fp
T 55000 42800 5 10 1 1 0 0 1
value=220
T 55400 42700 5 10 0 0 0 0 1
description=220 Ohm 1/8 Watt 0805
}
N 54500 42600 43300 42600 4
C 63600 47900 1 0 1 terminal-1.sym
{
T 63290 48650 5 10 0 0 0 6 1
device=terminal
T 63290 48500 5 10 0 0 0 6 1
footprint=spring_coin_contact.fp
T 63350 47950 5 10 1 1 0 0 1
refdes=J1
T 63600 47900 5 10 0 0 0 0 1
value=connect
}
C 63600 40900 1 0 1 terminal-1.sym
{
T 63290 41650 5 10 0 0 0 6 1
device=terminal
T 63290 41500 5 10 0 0 0 6 1
footprint=spring_coin_contact.fp
T 63350 40950 5 10 1 1 0 0 1
refdes=J2
T 63600 40900 5 10 0 0 0 0 1
value=connect
}
N 61400 40800 61400 41000 4
N 55500 46900 55500 45200 4
N 55500 45200 59800 45200 4
N 54000 46900 53950 46900 4
N 50150 48300 54000 48300 4
T 62250 48450 9 12 1 0 0 0 2
spring contacts
hold cuvette
T 62250 41450 9 12 1 0 0 0 2
spring contacts
hold cuvette
N 53950 46900 53950 46100 4
N 49600 46700 49600 46100 4
N 49600 45300 49600 43600 4
N 53950 43600 53950 45300 4
C 58900 47400 1 270 0 capacitor-1.sym
{
T 59600 47200 5 10 0 0 270 0 1
device=472-3KV
T 59300 47100 5 10 1 1 0 0 1
refdes=C4
T 59800 47200 5 10 0 0 270 0 1
symversion=0.1
T 58600 46700 5 10 1 1 0 0 1
value=4.7nF
T 58900 47400 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat_flip.fp
}
N 59100 47400 59100 48300 4
N 59100 45200 59100 46500 4
N 60400 41300 60400 41000 4
L 54800 49400 54800 48800 3 10 1 0 -1 -1
L 54800 50200 55400 50200 3 10 1 0 -1 -1
L 55700 50200 56500 50200 3 10 1 0 -1 -1
L 63800 50200 63800 49500 3 10 1 0 -1 -1
L 63800 44800 62800 44800 3 10 1 0 -1 -1
T 61000 49500 9 22 1 0 0 0 1
HV Zone
L 54800 50200 54800 49700 3 10 1 0 -1 -1
L 57900 50200 58700 50200 3 10 1 0 -1 -1
L 56800 50200 57600 50200 3 10 1 0 -1 -1
L 59000 50200 59800 50200 3 10 1 0 -1 -1
L 60100 50200 60900 50200 3 10 1 0 -1 -1
L 61200 50200 62000 50200 3 10 1 0 -1 -1
L 62300 50200 63100 50200 3 10 1 0 -1 -1
L 63400 50200 63700 50200 3 10 1 0 -1 -1
L 63800 48400 63800 49200 3 10 1 0 -1 -1
L 63800 47300 63800 48100 3 10 1 0 -1 -1
L 63800 46200 63800 47000 3 10 1 0 -1 -1
L 63800 45100 63800 45900 3 10 1 0 -1 -1
L 57400 44800 58200 44800 3 10 1 0 -1 -1
L 58500 44800 59300 44800 3 10 1 0 -1 -1
L 59600 44800 60300 44800 3 10 1 0 -1 -1
L 60600 44800 61400 44800 3 10 1 0 -1 -1
L 61700 44800 62500 44800 3 10 1 0 -1 -1
T 55700 46300 9 22 1 0 0 0 1
HV Zone
L 54800 44800 54800 45500 3 10 1 0 -1 -1
L 54800 45800 54800 46600 3 10 1 0 -1 -1
C 61600 46700 1 0 0 resistor-1.sym
{
T 61900 47100 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 47000 5 10 1 1 0 0 1
refdes=R3
T 62000 46500 5 10 1 1 0 0 1
value=475K
T 61600 46700 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 61600 46700 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 60400 46700 1 0 0 resistor-1.sym
{
T 60700 47100 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 60500 47000 5 10 1 1 0 0 1
refdes=R4
T 60800 46500 5 10 1 1 0 0 1
value=475K
T 60400 46700 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 60400 46700 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 60400 45900 1 0 0 resistor-1.sym
{
T 60700 46300 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 60500 46200 5 10 1 1 0 0 1
refdes=R5
T 60800 45700 5 10 1 1 0 0 1
value=475K
T 60400 45900 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 60400 45900 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 45900 1 0 0 resistor-1.sym
{
T 61900 46300 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 46200 5 10 1 1 0 0 1
refdes=R6
T 62000 45700 5 10 1 1 0 0 1
value=475K
T 61600 45900 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 61600 45900 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 45100 1 0 0 resistor-1.sym
{
T 61900 45500 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 45400 5 10 1 1 0 0 1
refdes=R7
T 62000 44900 5 10 1 1 0 0 1
value=475K
T 61600 45100 5 10 0 0 0 0 1
footprint=1206_HV.fp
T 61600 45100 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 60400 45100 1 0 0 resistor-1.sym
{
T 60700 45500 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 60500 45400 5 10 1 1 0 0 1
refdes=R8
T 60800 44900 5 10 1 1 0 0 1
value=475K
T 60400 45100 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
T 60400 45100 5 10 0 0 0 0 1
footprint=1206_HV.fp
}
N 60400 42200 60400 45200 4
N 60400 47600 60400 48000 4
N 51350 45500 53450 45500 4
N 46750 45500 49100 45500 4
N 46750 43600 46750 44250 4
N 51350 43600 51350 44250 4
N 46750 45150 46750 45850 4
N 51350 45150 51350 45900 4
C 57800 46500 1 90 0 diode-1.sym
{
T 57200 46900 5 10 0 0 90 0 1
device=R3000-TP
T 57100 47000 5 10 1 1 0 0 1
refdes=D2
T 58200 47000 5 10 1 1 180 0 1
value=3KV
T 57800 46500 5 10 0 0 90 0 1
description=DIODE GEN PURP 3KV 200MA DO15 
T 57800 46500 5 10 0 0 0 0 1
footprint=diode_10mm.fp
}
L 56300 44800 57100 44800 3 10 1 0 -1 -1
L 54800 44800 54900 44800 3 10 1 0 -1 -1
L 55200 44800 56000 44800 3 10 1 0 -1 -1
N 61600 47600 61300 47600 4
N 61300 46800 61600 46800 4
N 61300 46000 61600 46000 4
N 61300 45200 61600 45200 4
N 62500 46000 62500 45200 4
N 62500 46800 62500 47600 4
N 59400 45200 59400 41000 4
N 46750 43600 53950 43600 4
{
T 51550 43655 5 10 1 1 0 3 1
netname=com
}
N 43300 42600 43300 46400 4
T 39100 40500 9 12 1 0 0 0 7
ZIF connector pin numbering 
is not connected 1 to 1,
but by FFC cable with same 
side up on both boards.
Connector left pin connects to 
other end connecto right pin 
as a signal goes through FFC cable.
N 55500 48300 56300 48300 4
C 49350 43300 1 0 0 com.sym
C 61300 40500 1 0 0 com.sym
C 49100 45300 1 0 0 nmos-4pad.sym
{
T 49800 45650 5 10 1 1 0 0 1
device=BUK98150-55
T 49800 45850 5 10 1 1 0 0 1
refdes=Q11
T 49800 45450 5 10 1 1 0 0 1
footprint=SOT223_min.lht
T 49100 45300 5 10 0 0 0 0 1
description=MOSFET N-CH 55V 5.5A SOT223 enhanced mode
T 49100 45300 5 10 0 0 0 0 1
value=NFET
}
N 49700 46100 49700 46700 4
C 53450 45300 1 0 0 nmos-4pad.sym
{
T 54050 45750 5 10 1 1 0 0 1
device=BUK98150-55
T 54150 46000 5 10 1 1 0 0 1
refdes=Q12
T 54050 45500 5 10 0 1 0 0 1
footprint=SOT223_min.lht
T 53450 45300 5 10 0 0 0 0 1
description=MOSFET N-CH 55V 5.5A SOT223 enhanced mode
T 53450 45300 5 10 0 0 0 0 1
value=NFET
}
N 54050 46100 54050 46300 4
N 54050 46300 53950 46300 4
C 39000 48900 1 180 1 ffc-zif-20.sym
{
T 39100 42300 5 10 1 1 180 6 1
refdes=J3
T 39292 46442 5 10 1 1 270 6 1
footprint=ffc_zif_20.fp
T 40442 44308 5 10 0 1 180 6 1
device=zif_connector
T 39000 48900 5 10 0 0 0 0 1
value=connect
T 39000 48900 5 10 0 0 0 0 1
net=com:21
T 39000 48900 5 10 0 0 0 0 1
net=com:22
}
N 40300 42800 42000 42800 4
N 42000 42800 42000 42700 4
N 40300 45200 41800 45200 4
{
T 41300 45255 5 10 1 1 0 3 1
netname=com
}
N 40300 45500 40800 45500 4
N 40300 48500 41000 48500 4
N 40800 48200 40800 48500 4
N 40300 48200 43000 48200 4
{
T 41300 48255 5 10 1 1 0 3 1
netname=com
}
C 42100 42400 1 0 1 com.sym
N 46300 44000 46300 48250 4
C 45950 48500 1 0 0 p-chan-dual-so6.sym
{
T 46550 50300 5 14 1 1 0 3 1
refdes=U1
T 46350 48400 5 10 1 1 0 0 1
footprint=SOT563-6-most.fp
T 46750 50350 5 10 1 1 0 0 1
device=NTZD3152PT1G
}
N 40300 47600 41200 47600 4
N 41200 47600 41200 47900 4
N 40300 47900 42300 47900 4
{
T 41550 48100 5 11 1 1 180 6 1
netname=20V_IN
}
N 40800 47300 40300 47300 4
N 40300 47000 41800 47000 4
{
T 41300 47055 5 10 1 1 0 3 1
netname=com
}
N 40800 47000 40800 47300 4
N 40300 46700 42650 46700 4
N 40300 46400 43300 46400 4
{
T 40400 46550 5 10 1 1 180 6 1
netname=MCU_PA4_HV_sense
}
N 40300 46100 42650 46100 4
N 40300 45800 42650 45800 4
N 40300 44300 41800 44300 4
{
T 41300 44355 5 10 1 1 0 3 1
netname=com
}
N 40800 44300 40800 44600 4
N 40300 43100 42500 43100 4
{
T 40600 43250 5 10 1 1 180 6 1
netname=MCU_PC0_ADC10_IN
}
N 40300 43400 42500 43400 4
{
T 40600 43550 5 10 1 1 180 6 1
netname=MCU_PC1_ADC11_IN
}
N 40300 43700 41800 43700 4
{
T 41300 43755 5 10 1 1 0 3 1
netname=com
}
N 40300 44600 40800 44600 4
N 45400 44900 45400 50050 4
N 40800 45200 40800 45500 4
C 42400 51600 1 270 0 capacitor-1.sym
{
T 43100 51400 5 10 0 0 270 0 1
device=CL31A106MBHNNNE
T 43300 51400 5 10 0 0 270 0 1
symversion=0.1
T 42400 51600 5 10 0 0 0 0 1
description=CAP CER 10UF 50V X5R 1206 
T 42400 51600 5 10 0 0 0 0 1
footprint=chip_3216_1206_N.fp
T 42500 51000 5 10 1 1 180 0 1
refdes=C6
T 42800 51300 5 10 1 1 90 0 1
value=10 uF
T 42200 51500 5 10 1 1 0 0 1
comment=X5R
}
C 40200 54400 1 0 0 Vdd.sym
{
T 40100 54700 5 10 1 1 0 0 1
net=20V_IN:1
}
C 42900 47150 1 0 0 com.sym
N 43000 50200 43000 47450 4
N 52950 47600 54000 47600 4
{
T 53050 47750 5 11 1 1 180 6 1
netname=20V_IN
}
N 53950 46400 53000 46400 4
N 52500 45800 51350 45800 4
N 53000 45600 53000 43600 4
N 47600 45800 47600 45500 4
N 48350 45600 48350 43600 4
N 40300 44000 46300 44000 4
{
T 40600 44150 5 10 1 1 180 6 1
netname=PA0_TIM_pulser
}
N 40300 44900 45400 44900 4
{
T 40600 45050 5 10 1 1 180 6 1
netname=PA1_TIM_pulser
}
C 43100 51600 1 270 0 capacitor-2.sym
{
T 43800 51400 5 10 0 0 270 0 1
device=UUX1E221MNL1GS
T 44000 51400 5 10 0 0 270 0 1
symversion=0.1
T 43100 51600 5 10 0 0 0 0 1
description=CAP ALUM 220UF 20% 25V SMD
T 43100 51600 5 10 0 0 0 0 1
footprint=cap_10mm_alum_smt.fp
T 43000 50800 5 10 1 1 0 0 1
refdes=C7
T 43200 51300 5 10 1 1 90 0 1
value=220 uF
}
C 43700 51600 1 270 0 capacitor-2.sym
{
T 44400 51400 5 10 0 0 270 0 1
device=UUX1E221MNL1GS
T 44600 51400 5 10 0 0 270 0 1
symversion=0.1
T 43700 51600 5 10 0 0 0 0 1
description=CAP ALUM 220UF 20% 25V SMD
T 43700 51600 5 10 0 0 0 0 1
footprint=cap_10mm_alum_smt.fp
T 43600 50800 5 10 1 1 0 0 1
refdes=C8
T 43800 51300 5 10 1 1 90 0 1
value=220 uF
}
C 40800 51600 1 270 0 capacitor-1.sym
{
T 41500 51400 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 41700 51400 5 10 0 0 270 0 1
symversion=0.1
T 40800 51600 5 10 0 0 0 0 1
description=CAP CER 1UF 50V X7R
T 40800 51600 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
T 40900 51000 5 10 1 1 180 0 1
refdes=C10
T 41200 51300 5 10 1 1 90 0 1
value=1 uF
T 40600 51500 5 10 1 1 0 0 1
comment=X5R
}
C 41600 51600 1 270 0 capacitor-1.sym
{
T 42300 51400 5 10 0 0 270 0 1
device=CL31A106MBHNNNE
T 42500 51400 5 10 0 0 270 0 1
symversion=0.1
T 41600 51600 5 10 0 0 0 0 1
description=CAP CER 10UF 50V X5R 1206 
T 41600 51600 5 10 0 0 0 0 1
footprint=chip_3216_1206_N.fp
T 41700 51000 5 10 1 1 180 0 1
refdes=C11
T 42000 51300 5 10 1 1 90 0 1
value=10 uF
T 41400 51500 5 10 1 1 0 0 1
comment=X5R
}
C 52500 45600 1 0 0 NFET-gsd.sym
{
T 51850 46650 5 10 1 1 0 0 1
device=NTR5198NL
T 52500 46400 5 10 1 1 0 0 1
refdes=Q2
T 51250 46000 5 10 0 1 0 0 1
footprint=SOT23-123.fp
}
N 45400 51100 49000 51100 4
{
T 45500 51150 5 10 1 1 0 0 1
netname=v4
}
N 45400 50950 45400 51100 4
N 45950 49700 45950 51100 4
N 45950 49400 45400 49400 4
N 48250 49100 48550 49100 4
N 48550 49100 48550 51100 4
N 48250 49400 49000 49400 4
N 49000 50950 49000 51100 4
C 46850 44250 1 90 0 resistor-1.sym
{
T 46450 44550 5 10 0 0 90 0 1
device=RC0805FR-074K75L
T 46550 44450 5 10 1 1 90 0 1
refdes=R11
T 47050 44650 5 10 1 1 90 0 1
value=4.75K
T 46850 44250 5 10 0 0 0 0 1
description=4.75k Ohm ±1% 0.125W, 1/8W Chip Resistor 0805 (2012 Metric) 
T 46850 44250 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
}
C 51450 44250 1 90 0 resistor-1.sym
{
T 51050 44550 5 10 0 0 90 0 1
device=RC0805FR-074K75L
T 51150 44450 5 10 1 1 90 0 1
refdes=R10
T 51650 44650 5 10 1 1 90 0 1
value=4.75K
T 51450 44250 5 10 0 0 0 0 1
description=4.75k Ohm ±1% 0.125W, 1/8W Chip Resistor 0805 (2012 Metric) 
T 51450 44250 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
}
C 46650 46750 1 270 0 resistor-1.sym
{
T 47050 46450 5 10 0 0 270 0 1
device=RES-0805
T 46550 45750 5 10 1 1 90 0 1
refdes=R13
T 46650 46750 5 10 0 1 90 0 1
footprint=chip_2012_0805_N.fp
T 46550 46350 5 10 1 1 90 0 1
value=2.49K
T 46650 46750 5 10 0 0 90 0 1
description=220 Ohm 1/8 Watt 0805
}
N 45800 47050 45800 49100 4
C 45500 50050 1 90 0 resistor-1.sym
{
T 45100 50350 5 10 0 0 90 0 1
device=RC0805FR-074K75L
T 45200 50250 5 10 1 1 90 0 1
refdes=R15
T 45700 50450 5 10 1 1 90 0 1
value=4.75K
T 45500 50050 5 10 0 0 0 0 1
description=4.75k Ohm ±1% 0.125W, 1/8W Chip Resistor 0805 (2012 Metric) 
T 45500 50050 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
}
C 49100 50050 1 90 0 resistor-1.sym
{
T 48700 50350 5 10 0 0 90 0 1
device=RC0805FR-074K75L
T 48800 50250 5 10 1 1 90 0 1
refdes=R16
T 49300 50450 5 10 1 1 90 0 1
value=4.75K
T 49100 50050 5 10 0 0 0 0 1
description=4.75k Ohm ±1% 0.125W, 1/8W Chip Resistor 0805 (2012 Metric) 
T 49100 50050 5 10 0 0 0 0 1
footprint=chip_2012_0805_N.fp
}
N 48250 49700 49950 49700 4
N 49000 48250 49000 50050 4
N 49000 48250 46300 48250 4
C 47850 45600 1 0 0 NFET-gsd.sym
{
T 47300 46600 5 10 1 1 0 0 1
device=NTR5198NL
T 47850 46400 5 10 1 1 0 0 1
refdes=Q1
T 47100 46850 5 10 1 1 0 0 1
footprint=SOT23-123.fp
}
T 52900 46500 9 12 1 0 0 0 2
Use small
transistor.
N 53350 43600 53350 44100 4
N 53350 45000 53350 46400 4
N 53350 46400 53950 46400 4
T 48450 46800 9 12 1 0 0 0 2
Use small
transistor
N 48350 46700 48350 46400 4
N 48800 44100 48800 43600 4
N 48800 45000 48800 46700 4
C 51250 46800 1 270 0 resistor-1.sym
{
T 51650 46500 5 10 0 0 270 0 1
device=RES-0805
T 51150 45800 5 10 1 1 90 0 1
refdes=R12
T 51250 46800 5 10 0 1 90 0 1
footprint=chip_2012_0805_N.fp
T 51150 46400 5 10 1 1 90 0 1
value=2.49K
T 51250 46800 5 10 0 0 90 0 1
description=220 Ohm 1/8 Watt 0805
}
N 49950 49700 49950 49300 4
N 49950 49300 51350 49300 4
N 51350 49300 51350 46800 4
N 47850 45800 47600 45800 4
N 50150 48300 50150 46700 4
N 50150 46700 48350 46700 4
N 45800 47050 46750 47050 4
N 46750 46750 46750 47050 4
N 45950 49100 45800 49100 4
T 40400 45850 9 10 1 0 0 0 1
MCU_PA2_PWM5_OUT
T 40400 46150 9 10 1 0 0 0 1
MCU_PA3_PWM6_OUT
T 40400 46700 9 10 1 0 0 0 1
MCU_PA5_ADC5_IN
C 42650 45700 1 0 0 nc-right-1.sym
{
T 42750 46200 5 10 0 0 0 0 1
value=NoConnection
T 42750 46400 5 10 0 0 0 0 1
device=DRC_Directive
}
C 42650 46000 1 0 0 nc-right-1.sym
{
T 42750 46500 5 10 0 0 0 0 1
value=NoConnection
T 42750 46700 5 10 0 0 0 0 1
device=DRC_Directive
}
C 42650 46600 1 0 0 nc-right-1.sym
{
T 42750 47100 5 10 0 0 0 0 1
value=NoConnection
T 42750 47300 5 10 0 0 0 0 1
device=DRC_Directive
}
C 42500 43000 1 0 0 nc-right-1.sym
{
T 42600 43500 5 10 0 0 0 0 1
value=NoConnection
T 42600 43700 5 10 0 0 0 0 1
device=DRC_Directive
}
C 42500 43300 1 0 0 nc-right-1.sym
{
T 42600 43800 5 10 0 0 0 0 1
value=NoConnection
T 42600 44000 5 10 0 0 0 0 1
device=DRC_Directive
}
N 42300 47900 42300 49200 4
N 42300 49200 40400 49200 4
N 40400 54400 40400 49200 4
N 40400 51900 44500 51900 4
N 41000 51900 41000 51600 4
N 41800 51600 41800 51900 4
N 42600 51600 42600 51900 4
N 43300 51600 43300 51900 4
N 43900 51600 43900 51900 4
N 43900 50700 43900 50200 4
N 41000 50200 44500 50200 4
N 43300 50700 43300 50200 4
N 41000 50700 41000 50200 4
N 41800 50700 41800 50200 4
N 42600 50700 42600 50200 4
N 47000 51100 47000 52500 4
N 47000 53300 47000 53700 4
N 40400 53700 47000 53700 4
C 44300 51600 1 270 0 capacitor-2.sym
{
T 45000 51400 5 10 0 0 270 0 1
device=UUX1E221MNL1GS
T 45200 51400 5 10 0 0 270 0 1
symversion=0.1
T 44300 51600 5 10 0 0 0 0 1
description=CAP ALUM 220UF 20% 25V SMD
T 44300 51600 5 10 0 0 0 0 1
footprint=cap_10mm_alum_smt.fp
T 44200 50800 5 10 1 1 0 0 1
refdes=C9
T 44400 51300 5 10 1 1 90 0 1
value=220 uF
}
N 44500 51600 44500 51900 4
N 44500 50700 44500 50200 4
C 39300 53200 1 0 0 terminal-1.sym
{
T 39610 53950 5 10 0 0 0 0 1
device=terminal
T 39610 53800 5 10 0 0 0 0 1
footprint=term-wire-042.fp
T 39550 53350 5 10 1 1 0 6 1
refdes=J4
T 39400 53000 5 10 1 1 0 0 1
description=20V_IN
}
N 40200 53300 40400 53300 4
C 39300 52400 1 0 0 terminal-1.sym
{
T 39610 53150 5 10 0 0 0 0 1
device=terminal
T 39610 53000 5 10 0 0 0 0 1
footprint=term-wire-042.fp
T 39550 52550 5 10 1 1 0 6 1
refdes=J5
T 39400 52200 5 10 1 1 0 0 1
description=20V_GND
}
N 40200 52500 40200 49000 4
N 41000 49000 41000 48500 4
N 40200 49000 41000 49000 4
C 47100 52500 1 90 0 zener-2.sym
{
T 46600 52900 5 10 0 0 90 0 1
device=MMSZ4680T1G
T 46700 53100 5 10 1 1 0 0 1
refdes=Z1
T 47100 52600 5 10 1 1 0 0 1
value=2.2V
T 47200 53100 5 10 0 1 0 0 1
description=DIODE ZENER 2.2V 500MW SOD123
T 47200 52300 5 10 0 1 0 0 1
footprint=SOD123.fp
}
T 51700 41900 9 12 1 0 0 0 1
put two vias to gnd near TXFMR pads
T 51700 41600 9 12 1 0 0 0 1
to heatsink the drive FETs
L 54400 46600 54400 42100 3 10 1 0 -1 -1
L 54400 46600 54300 46400 3 10 1 0 -1 -1
L 54500 46400 54300 46400 3 10 1 0 -1 -1
L 54500 46400 54400 46600 3 10 1 0 -1 -1
T 48600 42800 9 12 1 0 0 0 2
D3, D4 reduce STPS2L25U footprint tails,
move into subc around TXFMR
B 51600 41000 4800 1100 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
C 49000 44100 1 90 0 schottky.sym
{
T 48400 44500 5 10 0 0 90 0 1
device=Schottky
T 48300 44500 5 10 1 1 0 0 1
refdes=D3
T 49000 44100 5 10 1 1 0 0 1
value=.375V
T 49050 44300 5 10 0 1 0 0 1
footprint=SMB-min.lht
}
C 53550 44100 1 90 0 schottky.sym
{
T 52950 44500 5 10 0 0 90 0 1
device=Schottky
T 52850 44500 5 10 1 1 0 0 1
refdes=D4
T 53550 44100 5 10 1 1 0 0 1
value=.375V
T 53600 44300 5 10 0 1 0 0 1
footprint=SMB-min.lht
}
