v 20150930 2
C 40000 40000 0 0 0 title-A-cibolo.sym
{
T 61300 40000 5 10 1 1 0 0 1
file=pyflex_f401_power.sch
T 64900 39700 5 10 1 1 0 0 1
drawn-by=John Griessen
T 53400 39650 5 16 1 1 0 0 1
title=PYFLEX_F401 Power
T 61000 39700 5 10 1 1 0 0 1
first-pagenum=2
T 61900 39700 5 10 1 1 0 0 1
last-pagenum=2
T 64900 40000 5 10 1 1 0 0 1
rev=2018-08-25 v0.8
T 66800 45200 5 18 1 1 270 0 1
title2=PYFLEX_F401 Power
}
T 53400 39950 5 16 1 1 0 0 1
title=Culture Shock
N 60900 55800 61600 55800 4
{
T 60900 55850 5 10 1 1 0 0 1
netname=U3_SW
}
C 61600 55500 1 270 0 coil-1.sym
{
T 62000 55300 5 10 0 0 270 0 1
device=NRH2410T100MN
T 61600 55500 5 10 0 0 0 0 1
footprint=INDC2424N.lht
T 61800 54800 5 10 1 1 0 0 1
refdes=L31
T 61800 55100 5 10 1 1 0 0 1
value=10 uH
}
N 61600 54500 61600 53300 4
N 61600 55800 61600 55500 4
N 60900 53300 61600 53300 4
N 62800 53600 62800 57500 4
N 62800 53600 61600 53600 4
N 60900 52300 60900 53300 4
C 61600 57400 1 0 0 resistor-1.sym
{
T 61900 57800 5 10 0 0 0 0 1
device=R_180K_1%_1608
T 62600 58000 5 10 0 1 180 0 1
footprint=RESC1608N.lht
T 61800 57800 5 10 1 1 180 0 1
refdes=R35
T 62400 57300 5 10 1 1 180 0 1
value=180k
}
N 62800 57500 62500 57500 4
N 60900 55800 60900 56000 4
N 60000 56000 59700 56000 4
C 57200 56700 1 270 0 resistor-1.sym
{
T 57600 56400 5 10 0 0 270 0 1
device=R_1M0_1%_1608
T 57800 55500 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 57200 56500 5 10 1 1 180 0 1
refdes=R31
T 57800 56300 5 10 1 1 180 0 1
value=1M0
}
N 58200 55800 59100 55800 4
{
T 58300 55650 5 10 1 1 0 0 1
netname=U3_FB
}
C 60700 56000 1 270 1 schottky.sym
{
T 62000 56400 5 10 1 1 0 6 1
device=B5819WS
T 61450 56650 5 10 1 1 0 6 1
refdes=D31
T 61200 56200 5 10 1 1 0 0 1
value=9Apulse .5W
T 61000 56100 5 10 0 0 0 0 1
footprint=SOD-323N.lht
}
N 60000 55800 60000 56000 4
C 58800 53300 1 0 0 dc-dc-sot23-5.sym
{
T 58800 53300 5 10 0 0 0 0 1
value=IC
T 60500 55600 5 14 1 1 0 3 1
refdes=U3
T 59300 54500 5 10 1 1 0 0 1
device=TPS61040DBVR
T 59400 53700 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 59700 55900 59700 56200 4
N 60000 51200 61600 51200 4
C 59100 51100 1 0 0 resistor-1.sym
{
T 59400 51500 5 10 0 0 0 0 1
device=R_7.50k_1%_1608
T 59100 51100 5 10 0 0 180 0 1
footprint=RESC1608N.lht
T 59300 51400 5 10 1 1 0 0 1
refdes=R34
T 59500 50900 5 10 1 1 0 0 1
value=7.50k
}
C 59700 56300 1 180 0 resistor-1.sym
{
T 59400 55900 5 10 0 0 180 0 1
device=R_180K_1%_1608
T 58700 55700 5 10 0 1 0 0 1
footprint=RESC1608N.lht
T 58900 56400 5 10 1 1 0 0 1
refdes=R32
T 59100 55900 5 10 1 1 0 0 1
value=180k
}
N 58200 52300 60900 52300 4
N 58800 56200 58800 55800 4
C 58700 57500 1 270 0 pnp-bec-123.sym
{
T 58700 57500 5 10 0 0 0 0 1
footprint=SOT323N.lht
T 58700 57500 5 10 0 0 0 0 1
description=BJT PNP Hfe=220 45V 0.5A 
T 58700 57500 5 10 0 0 0 0 1
value=BJT PNP
T 59700 57000 5 10 1 1 0 0 1
device=BC807-40L
T 59500 57200 5 10 1 1 0 0 1
refdes=Q31
}
N 59200 57500 61600 57500 4
C 61800 52400 1 90 0 capacitor-1.sym
{
T 61700 53000 5 10 1 1 0 0 1
value=10 uF
T 61100 52600 5 10 0 0 90 0 1
device=CC1206KFX5R8BB106
T 61800 52400 5 10 0 0 180 0 1
footprint=CAPC3216N.lht
T 61700 52500 5 10 1 1 0 0 1
refdes=C31
}
N 61600 51200 61600 52400 4
N 58500 56900 58700 56900 4
N 60900 56900 59700 56900 4
C 58000 56700 1 270 0 capacitor-1.sym
{
T 58700 56500 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 58000 56700 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 58000 56700 5 10 0 0 0 0 1
description=MLCC 3.3pF 50V COG
T 58300 56400 5 10 1 1 0 0 1
refdes=C33
T 58300 56000 5 10 1 1 0 0 1
value=3.3pF
}
N 58500 56700 57300 56700 4
N 58500 56700 58500 57900 4
N 58500 57900 58800 57900 4
C 60900 50900 1 0 0 vss.sym
N 55100 51100 54400 51100 4
{
T 54900 51150 5 10 1 1 0 6 1
netname=PB14
}
N 59700 57900 61200 57900 4
{
T 61200 57950 5 10 1 1 0 6 1
netname=ONBOARD_18V
}
C 60200 56000 1 90 0 EMBEDDEDcapacitor-1.sym
[
P 60000 56000 60000 56200 1 0 0
{
T 59950 56150 5 8 0 1 90 6 1
pinnumber=1
T 60050 56150 5 8 0 1 90 8 1
pinseq=1
T 60000 56200 9 8 0 1 90 0 1
pinlabel=1
T 60000 56200 5 8 0 1 90 2 1
pintype=pas
}
P 60000 56900 60000 56700 1 0 0
{
T 59950 56750 5 8 0 1 90 0 1
pinnumber=2
T 60050 56750 5 8 0 1 90 2 1
pinseq=2
T 60000 56700 9 8 0 1 90 6 1
pinlabel=2
T 60000 56700 5 8 0 1 90 8 1
pintype=pas
}
L 59800 56400 60200 56400 3 0 0 0 -1 -1
L 59800 56500 60200 56500 3 0 0 0 -1 -1
L 60000 56700 60000 56500 3 0 0 0 -1 -1
L 60000 56400 60000 56200 3 0 0 0 -1 -1
T 59500 56200 5 10 0 0 90 0 1
device=CAPACITOR
T 59700 56200 8 10 0 1 90 0 1
refdes=C?
T 58900 56200 5 10 0 0 90 0 1
description=capacitor
T 59100 56200 5 10 0 0 90 0 1
numslots=0
T 59300 56200 5 10 0 0 90 0 1
symversion=0.1
]
{
T 59500 56200 5 10 0 0 90 0 1
device=CL10A105KA5LNNC
T 59300 56200 5 10 0 0 90 0 1
symversion=0.1
T 60200 56000 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 60200 56000 5 10 0 0 0 0 1
description=Cap MLCC 1uF 25V X5R 10%
T 59600 56600 5 10 1 1 0 0 1
refdes=C32
T 60100 56200 5 10 1 1 0 0 1
value=1 uF
}
N 56000 51100 56900 51100 4
N 56200 51100 56200 51600 4
C 56300 51600 1 90 0 resistor-1.sym
{
T 55900 51900 5 10 0 0 90 0 1
device=R_1.8k_1%_1608
T 56300 51600 5 10 0 0 0 0 1
description=Res thick 1.80k 1% 1608
T 56300 51600 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 56000 51800 5 10 1 1 90 0 1
refdes=R37
T 56500 51800 5 10 1 1 90 0 1
value=1.80K
}
C 56000 51200 1 180 0 resistor-1.sym
{
T 55700 50800 5 10 0 0 180 0 1
device=R_7.50k_1%_1608
T 56000 51200 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 55200 51300 5 10 1 1 0 0 1
refdes=R33
T 55600 50800 5 10 1 1 0 0 1
value=7.50k
}
C 58800 58100 1 270 0 jump-solder.sym
{
T 59200 57700 5 8 0 0 270 0 1
device=JUMPER_SOLDER
T 58800 58100 5 10 0 0 180 0 1
footprint=jump-solder1.lht
T 58800 58100 5 10 0 0 180 0 1
value=layout-shape
T 58700 58100 5 10 1 1 0 0 1
refdes=J32
}
N 56200 52700 58200 52700 4
{
T 56300 52750 5 10 1 1 0 0 1
netname=U3_VIN
}
C 55200 52700 1 0 1 Vdd.sym
{
T 55000 52950 5 10 1 1 0 3 1
net=5VUNREG:1
}
C 56200 52500 1 90 0 jump-solder.sym
{
T 55800 52900 5 8 0 0 90 0 1
device=JUMPER_SOLDER
T 56200 52500 5 10 0 0 0 0 1
footprint=jump-solder1.lht
T 56200 52500 5 10 0 0 0 0 1
value=layout-shape
T 55500 52600 5 10 1 1 180 0 1
refdes=J31
}
N 55000 52700 55300 52700 4
N 56200 52500 56200 52700 4
C 59600 55600 1 0 0 vss.sym
N 59100 50200 59100 53300 4
N 58200 52300 58200 52700 4
C 57200 50200 1 0 0 PNP-PNP-so6-half1.sym
{
T 58600 52250 5 14 1 1 180 3 1
refdes=U8
T 58000 50700 5 10 1 1 180 0 1
device=BC857BDW1T1
T 56800 50800 5 10 1 1 0 0 1
footprint=SOT363M.lht
}
C 53900 54900 1 0 0 PNP-PNP-so6-half2.sym
{
T 55200 56850 5 14 1 1 0 3 1
refdes=U8
T 53500 55400 5 10 1 1 0 0 1
device=BC857BDW1T1
T 53500 55700 5 10 1 1 0 0 1
footprint=SOT363M.lht
}
N 56900 51100 56900 52300 4
N 56900 52300 57600 52300 4
N 58200 50200 59100 50200 4
{
T 59200 50250 5 10 1 1 0 6 1
netname=U3_!SHDN
}
C 57200 55300 1 270 0 resistor-1.sym
{
T 57600 55000 5 10 0 0 270 0 1
device=R_1M3_1%_1608
T 57800 54100 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 57200 55100 5 10 1 1 180 0 1
refdes=R36
T 57800 54900 5 10 1 1 180 0 1
value=1M3
}
N 57300 55800 57300 55300 4
N 56200 53900 58200 53900 4
N 56200 55700 57300 55700 4
N 54300 57000 54300 57500 4
N 58200 55800 58200 53900 4
C 54400 57500 1 90 0 resistor-1.sym
{
T 54000 57800 5 10 0 0 90 0 1
device=R_1.8k_1%_1608
T 54400 57500 5 10 0 0 0 0 1
description=Res thick 1.80k 1% 1608
T 54400 57500 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 54100 58000 5 10 1 1 180 0 1
refdes=R39
T 54900 58000 5 10 1 1 180 0 1
value=1.80K
}
N 54100 57300 54300 57300 4
C 54100 57400 1 180 0 resistor-1.sym
{
T 53800 57000 5 10 0 0 180 0 1
device=R_7.50k_1%_1608
T 54100 57400 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 53300 57500 5 10 1 1 0 0 1
refdes=R38
T 53700 57000 5 10 1 1 0 0 1
value=7.50k
}
N 53200 57300 52500 57300 4
{
T 52900 57350 5 10 1 1 0 6 1
netname=PB13
}
N 53300 58400 54900 58400 4
{
T 53400 58450 5 10 1 1 0 0 1
netname=U3_VIN
}
N 54900 58400 54900 57000 4
T 63000 56100 9 12 1 0 0 0 3
D31 Add 
more PCB
Cu (heat)
T 61800 53700 9 10 1 0 0 0 3
2424 
footprt

T 56800 57100 9 12 1 0 0 0 2
Choose Cff 
for 180k R35
T 59600 54000 9 10 1 0 0 0 2
1.0 MHz 
switcher
T 54500 50200 9 12 1 0 0 0 3
PB14 LO
==>
switcher ON
N 46300 45100 46300 44800 4
N 44500 44800 49400 44800 4
N 46000 45100 46000 44800 4
C 48500 45900 1 90 1 EMBEDDEDcapacitor-1.sym
[
P 48300 45900 48300 45700 1 0 0
{
T 48250 45750 5 8 0 1 90 0 1
pinnumber=1
T 48350 45750 5 8 0 1 90 2 1
pinseq=1
T 48300 45700 9 8 0 1 90 6 1
pinlabel=1
T 48300 45700 5 8 0 1 90 8 1
pintype=pas
}
P 48300 45000 48300 45200 1 0 0
{
T 48250 45150 5 8 0 1 90 6 1
pinnumber=2
T 48350 45150 5 8 0 1 90 8 1
pinseq=2
T 48300 45200 9 8 0 1 90 0 1
pinlabel=2
T 48300 45200 5 8 0 1 90 2 1
pintype=pas
}
L 48100 45500 48500 45500 3 0 0 0 -1 -1
L 48100 45400 48500 45400 3 0 0 0 -1 -1
L 48300 45200 48300 45400 3 0 0 0 -1 -1
L 48300 45500 48300 45700 3 0 0 0 -1 -1
T 47800 45700 5 10 0 0 90 6 1
device=CAPACITOR
T 48000 45700 8 10 0 1 90 6 1
refdes=C?
T 47200 45700 5 10 0 0 90 6 1
description=capacitor
T 47400 45700 5 10 0 0 90 6 1
numslots=0
T 47600 45700 5 10 0 0 90 6 1
symversion=0.1
]
{
T 47800 45700 5 10 0 0 270 2 1
device=06033C104KAT2A
T 47600 45700 5 10 0 0 270 2 1
symversion=0.1
T 48500 45900 5 10 0 0 0 6 1
footprint=CAPC1608N.lht
T 48700 45600 5 10 1 1 0 6 1
refdes=C61
T 48900 45200 5 10 1 1 0 6 1
value=0.1 uF
}
N 48300 45000 48300 44800 4
N 47700 46400 48300 46400 4
N 48300 46400 48300 45900 4
C 44700 44300 1 180 1 EMBEDDEDcoil-1.sym
[
P 44900 44300 44700 44300 1 0 1
{
T 44850 44250 5 8 0 1 180 0 1
pinnumber=1
T 44850 44350 5 8 0 1 180 2 1
pinseq=1
T 44950 44300 9 8 0 1 180 6 1
pinlabel=1
T 44950 44300 5 8 0 1 180 8 1
pintype=pas
}
P 45500 44300 45700 44300 1 0 1
{
T 45550 44250 5 8 0 1 180 6 1
pinnumber=2
T 45550 44350 5 8 0 1 180 8 1
pinseq=2
T 45450 44300 9 8 0 1 180 0 1
pinlabel=2
T 45450 44300 5 8 0 1 180 2 1
pintype=pas
}
A 45000 44300 100 180 180 3 0 0 0 -1 -1
A 45200 44300 100 180 180 3 0 0 0 -1 -1
A 45400 44300 100 180 180 3 0 0 0 -1 -1
T 44900 43900 5 10 0 0 180 6 1
device=COIL
T 44900 44100 8 10 0 1 180 6 1
refdes=L?
T 44900 43300 5 10 0 0 180 6 1
description=incuctor
T 44900 43500 5 10 0 0 180 6 1
numslots=0
T 44900 43700 5 10 0 0 180 6 1
symversion=0.1
]
{
T 44900 43900 5 10 0 0 0 2 1
device=NRS2012T4R7MGJ
T 44700 44300 5 10 0 0 90 6 1
footprint=INDC2020N.lht
T 45200 44400 5 10 1 1 0 6 1
refdes=L61
T 45400 44000 5 10 1 1 0 6 1
value=4.7 uH
}
N 44800 46100 44200 46100 4
N 44200 44300 44200 46400 4
N 46600 45100 46600 44300 4
N 47200 44300 45700 44300 4
N 44700 44300 44200 44300 4
N 47700 46700 48000 46700 4
N 48000 46700 48000 44800 4
N 44800 45100 44800 46100 4
N 44800 45100 45700 45100 4
N 44800 46400 44200 46400 4
N 47700 46100 49400 46100 4
{
T 48800 46150 5 10 1 1 0 0 1
netname=VBAT
}
C 46800 44500 1 0 0 vss.sym
C 50100 46150 1 180 0 EMBEDDEDterminal-1.sym
[
P 49400 46100 49650 46100 1 0 0
{
T 49450 46150 5 10 0 0 0 0 1
pintype=pas
T 49708 46092 9 10 0 1 0 0 1
pinlabel=terminal
T 49558 45942 5 10 0 1 0 6 1
pinnumber=1
T 49450 46150 5 10 0 0 0 0 1
pinseq=1
}
V 49700 46100 50 3 15 1 0 -1 -1 0 -1 -1 -1 -1 -1
T 49900 46150 8 10 0 1 180 6 1
refdes=T?
]
{
T 49790 45400 5 10 0 0 180 0 1
device=terminal-layout
T 49790 45550 5 10 0 0 180 0 1
footprint=term-pad-3x4.lht
T 50100 46150 5 10 0 0 0 0 1
pintype=pwr
T 50100 46150 5 10 0 0 0 0 1
value=layout
T 49850 46100 5 10 1 1 180 6 1
refdes=JE
}
L 49700 46000 49700 45600 3 10 1 0 -1 -1
L 49400 45600 50000 45600 3 10 1 0 -1 -1
L 49800 45500 49600 45500 3 10 1 0 -1 -1
L 49600 45500 49600 45400 3 10 1 0 -1 -1
L 49600 45400 49800 45400 3 10 1 0 -1 -1
L 49800 45400 49800 45500 3 10 1 0 -1 -1
L 49700 45400 49700 44900 3 10 1 0 -1 -1
C 50100 44850 1 180 0 EMBEDDEDterminal-1.sym
[
P 49400 44800 49650 44800 1 0 0
{
T 49450 44850 5 10 0 0 0 0 1
pintype=pas
T 49708 44792 9 10 0 1 0 0 1
pinlabel=terminal
T 49558 44642 5 10 0 1 0 6 1
pinnumber=1
T 49450 44850 5 10 0 0 0 0 1
pinseq=1
}
V 49700 44800 50 3 15 1 0 -1 -1 0 -1 -1 -1 -1 -1
T 49900 44850 8 10 0 1 180 6 1
refdes=T?
]
{
T 49790 44100 5 10 0 0 180 0 1
device=terminal-layout
T 49790 44250 5 10 0 0 180 0 1
footprint=term-pad-3x4.lht
T 50100 44850 5 10 0 0 0 0 1
pintype=pwr
T 50100 44850 5 10 0 0 0 0 1
value=layout
T 49850 44800 5 10 1 1 180 6 1
refdes=JF
}
N 48000 46600 48800 46600 4
C 48100 44400 1 180 0 EMBEDDEDresistor-1.sym
[
L 47500 44200 47600 44400 3 0 0 0 -1 -1
L 47600 44400 47700 44200 3 0 0 0 -1 -1
L 47700 44200 47800 44400 3 0 0 0 -1 -1
L 47800 44400 47900 44200 3 0 0 0 -1 -1
L 47500 44200 47400 44400 3 0 0 0 -1 -1
L 47400 44400 47350 44300 3 0 0 0 -1 -1
P 47200 44300 47350 44300 1 0 0
{
T 47300 44250 5 8 0 1 180 0 1
pinnumber=2
T 47300 44250 5 8 0 0 180 0 1
pinseq=2
T 47300 44250 5 8 0 1 180 0 1
pinlabel=2
T 47300 44250 5 8 0 1 180 0 1
pintype=pas
}
P 48100 44300 47948 44300 1 0 0
{
T 48000 44250 5 8 0 1 180 0 1
pinnumber=1
T 48000 44250 5 8 0 0 180 0 1
pinseq=1
T 48000 44250 5 8 0 1 180 0 1
pinlabel=1
T 48000 44250 5 8 0 1 180 0 1
pintype=pas
}
L 47899 44200 47950 44300 3 0 0 0 -1 -1
T 47800 44000 5 10 0 0 180 0 1
device=RESISTOR
T 47900 44100 8 10 0 1 180 0 1
refdes=R?
T 48100 44400 8 10 0 1 180 0 1
pins=2
T 48100 44400 8 10 0 1 180 0 1
class=DISCRETE
]
{
T 47800 44000 5 10 0 0 180 0 1
device=R_0R22_1%_1608
T 48100 44400 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 47200 44500 5 10 1 1 0 0 1
refdes=R65
T 47500 44000 5 10 1 1 0 0 1
value=.22
}
N 49000 46100 49000 44300 4
N 49000 44300 48100 44300 4
N 47700 47000 47900 47000 4
N 48800 47000 48800 46600 4
N 44500 44800 44500 47600 4
C 48800 47100 1 180 0 EMBEDDEDresistor-1.sym
[
L 48200 46900 48300 47100 3 0 0 0 -1 -1
L 48300 47100 48400 46900 3 0 0 0 -1 -1
L 48400 46900 48500 47100 3 0 0 0 -1 -1
L 48500 47100 48600 46900 3 0 0 0 -1 -1
L 48200 46900 48100 47100 3 0 0 0 -1 -1
L 48100 47100 48050 47000 3 0 0 0 -1 -1
P 47900 47000 48050 47000 1 0 0
{
T 48000 46950 5 8 0 1 180 0 1
pinnumber=2
T 48000 46950 5 8 0 0 180 0 1
pinseq=2
T 48000 46950 5 8 0 1 180 0 1
pinlabel=2
T 48000 46950 5 8 0 1 180 0 1
pintype=pas
}
P 48800 47000 48648 47000 1 0 0
{
T 48700 46950 5 8 0 1 180 0 1
pinnumber=1
T 48700 46950 5 8 0 0 180 0 1
pinseq=1
T 48700 46950 5 8 0 1 180 0 1
pinlabel=1
T 48700 46950 5 8 0 1 180 0 1
pintype=pas
}
L 48599 46900 48650 47000 3 0 0 0 -1 -1
T 48500 46700 5 10 0 0 180 0 1
device=RESISTOR
T 48600 46800 8 10 0 1 180 0 1
refdes=R?
T 48800 47100 8 10 0 1 180 0 1
pins=2
T 48800 47100 8 10 0 1 180 0 1
class=DISCRETE
]
{
T 48500 46700 5 10 0 0 180 0 1
device=R_62k_1%_1608
T 48800 47100 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 47900 47200 5 10 1 1 0 0 1
refdes=R63
T 48200 46700 5 10 1 1 0 0 1
value=62.0k
}
C 44300 47600 1 270 1 EMBEDDEDcapacitor-1.sym
[
P 44500 47600 44500 47800 1 0 0
{
T 44550 47750 5 8 0 1 270 0 1
pinnumber=1
T 44450 47750 5 8 0 1 270 2 1
pinseq=1
T 44500 47800 9 8 0 1 270 6 1
pinlabel=1
T 44500 47800 5 8 0 1 270 8 1
pintype=pas
}
P 44500 48500 44500 48300 1 0 0
{
T 44550 48350 5 8 0 1 270 6 1
pinnumber=2
T 44450 48350 5 8 0 1 270 8 1
pinseq=2
T 44500 48300 9 8 0 1 270 0 1
pinlabel=2
T 44500 48300 5 8 0 1 270 2 1
pintype=pas
}
L 44700 48000 44300 48000 3 0 0 0 -1 -1
L 44700 48100 44300 48100 3 0 0 0 -1 -1
L 44500 48300 44500 48100 3 0 0 0 -1 -1
L 44500 48000 44500 47800 3 0 0 0 -1 -1
T 45000 47800 5 10 0 0 270 6 1
device=CAPACITOR
T 44800 47800 8 10 0 1 270 6 1
refdes=C?
T 45600 47800 5 10 0 0 270 6 1
description=capacitor
T 45400 47800 5 10 0 0 270 6 1
numslots=0
T 45200 47800 5 10 0 0 270 6 1
symversion=0.1
]
{
T 44400 47700 5 10 1 1 0 6 1
value=4.7uF
T 45000 47800 5 10 0 0 90 2 1
device=CL10A475MO8NNNC
T 45200 47800 5 10 0 0 90 2 1
symversion=0.1
T 44300 47600 5 10 0 0 90 2 1
footprint=CAPC1608N.lht
T 44300 47600 5 10 0 0 0 0 1
description=Cap MLCC 4.7uF 16V X5R 20% 1608
T 44400 48200 5 10 1 1 0 6 1
refdes=C62
}
N 45700 48500 44500 48500 4
{
T 45500 48550 5 10 1 1 0 6 1
netname=5VUNREG
}
C 44800 45100 1 0 0 tp5000_charger.sym
{
T 49600 47100 5 10 0 0 0 0 1
footprint=QFN16_4040N.lht
T 49600 47500 5 10 0 0 0 0 1
device=TP5000
T 47100 48400 5 14 1 1 0 3 1
refdes=U6
T 44800 45100 5 10 0 0 0 0 1
value=IC
}
N 44800 48500 44800 46700 4
N 46000 48500 46000 48900 4
C 46100 48900 1 90 0 EMBEDDEDnc-right-1.sym
[
P 46000 48900 46000 49100 1 0 0
{
T 46000 49500 5 10 0 0 270 8 1
pinseq=1
T 45800 49500 5 10 0 0 270 8 1
pinnumber=1
}
L 46100 49100 45900 49100 3 0 0 0 -1 -1
T 45600 49000 8 10 0 0 90 0 1
value=NoConnection
T 46000 49150 9 10 1 0 90 1 1
NC
T 45500 48900 8 10 0 0 90 0 1
documentation=nc.pdf
T 45400 49000 8 10 0 0 90 0 1
device=DRC_Directive
T 45200 49000 8 10 0 0 90 0 1
graphical=1
]
{
T 45600 49000 5 10 0 0 90 0 1
value=NoConnection
T 45400 49000 5 10 0 0 90 0 1
device=DRC_Directive
}
C 46600 48800 1 0 0 EMBEDDEDnc-right-1.sym
[
P 46600 48900 46800 48900 1 0 0
{
T 47200 48900 5 10 0 0 180 8 1
pinseq=1
T 47200 49100 5 10 0 0 180 8 1
pinnumber=1
}
L 46800 48800 46800 49000 3 0 0 0 -1 -1
T 46700 49300 8 10 0 0 0 0 1
value=NoConnection
T 46850 48900 9 10 1 0 0 1 1
NC
T 46600 49400 8 10 0 0 0 0 1
documentation=nc.pdf
T 46700 49500 8 10 0 0 0 0 1
device=DRC_Directive
T 46700 49700 8 10 0 0 0 0 1
graphical=1
]
{
T 46700 49300 5 10 0 0 0 0 1
value=NoConnection
T 46700 49500 5 10 0 0 0 0 1
device=DRC_Directive
}
N 46600 48500 46600 48900 4
N 46300 48500 46300 49800 4
C 47500 49900 1 180 0 EMBEDDEDresistor-1.sym
[
L 46900 49700 47000 49900 3 0 0 0 -1 -1
L 47000 49900 47100 49700 3 0 0 0 -1 -1
L 47100 49700 47200 49900 3 0 0 0 -1 -1
L 47200 49900 47300 49700 3 0 0 0 -1 -1
L 46900 49700 46800 49900 3 0 0 0 -1 -1
L 46800 49900 46750 49800 3 0 0 0 -1 -1
P 46600 49800 46750 49800 1 0 0
{
T 46700 49750 5 8 0 1 180 0 1
pinnumber=2
T 46700 49750 5 8 0 0 180 0 1
pinseq=2
T 46700 49750 5 8 0 1 180 0 1
pinlabel=2
T 46700 49750 5 8 0 1 180 0 1
pintype=pas
}
P 47500 49800 47348 49800 1 0 0
{
T 47400 49750 5 8 0 1 180 0 1
pinnumber=1
T 47400 49750 5 8 0 0 180 0 1
pinseq=1
T 47400 49750 5 8 0 1 180 0 1
pinlabel=1
T 47400 49750 5 8 0 1 180 0 1
pintype=pas
}
L 47299 49700 47350 49800 3 0 0 0 -1 -1
T 47200 49500 5 10 0 0 180 0 1
device=RESISTOR
T 47300 49600 8 10 0 1 180 0 1
refdes=R?
T 47500 49900 8 10 0 1 180 0 1
pins=2
T 47500 49900 8 10 0 1 180 0 1
class=DISCRETE
]
{
T 47200 49500 5 10 0 0 180 0 1
device=R_1.80k_1%_1608
T 47500 49900 5 10 0 0 0 0 1
footprint=RESC1608N.lht
T 47000 50100 5 10 1 1 180 0 1
refdes=R64
T 47300 49600 5 10 1 1 180 0 1
value=1.80k
}
N 48300 49800 49600 49800 4
{
T 48500 49850 5 10 1 1 0 0 1
netname=5VUNREG
}
N 46600 49800 46300 49800 4
N 47600 49800 47500 49800 4
C 48300 49900 1 180 0 LEDSCN.sym
{
T 48300 49300 5 10 0 0 180 0 1
device=LED-0805
T 48300 49900 5 10 0 1 0 0 1
footprint=DIOC2013N.lht
T 48000 50100 5 10 1 1 180 0 1
refdes=D11
T 48000 49600 5 10 1 1 0 0 1
value=GRN LED
}
T 50200 45200 9 12 1 0 90 0 1
battery 
T 49500 44200 9 12 1 0 0 0 2
IDC 
conn
T 49600 46300 9 12 1 0 0 0 2
IDC 
conn
T 48000 48400 9 12 1 0 0 0 3
CS pin not 
connected 

C 55700 54600 1 0 0 NFET_enh_DPAK.sym
{
T 56300 55200 5 10 1 1 0 0 1
refdes=Q32
T 55700 54600 5 10 0 1 90 2 1
description=NFET 30V 0.1A SOT416 SC75 VGS=20
T 55700 54600 5 10 0 0 90 2 1
value=NFET
T 55700 54600 5 10 0 1 90 2 1
footprint=SOT416N.lht
T 55600 55400 5 10 1 1 0 0 1
device=SSM3K15AFS 
}
N 56200 54600 56200 53900 4
N 56200 55700 56200 55400 4
N 57300 54400 57300 53900 4
N 55700 54800 54900 54800 4
N 54900 54800 54900 54900 4
L 47200 48900 47900 48900 3 10 1 0 -1 -1
T 49500 48600 9 12 1 0 0 0 2
LiFePO4 
3.6V cutoff
T 48700 48500 9 12 1 0 0 0 2
      ==> 

