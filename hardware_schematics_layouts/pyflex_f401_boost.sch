v 20130925 2
C 40000 40000 0 0 0 title-A-cibolo.sym
{
T 61300 40000 5 10 1 1 0 0 1
file=pyflex_f401_boost.sch
T 64900 39700 5 10 1 1 0 0 1
drawn-by=John Griessen
T 53400 39650 5 16 1 1 0 0 1
title=PYFLEX_F401 Boost PS
T 61000 39700 5 10 1 1 0 0 1
first-pagenum=2
T 61900 39700 5 10 1 1 0 0 1
last-pagenum=2
T 64900 40000 5 10 1 1 0 0 1
rev=2017-12-08 v0.4
T 66800 45200 5 18 1 1 270 0 1
title2=PYFLEX_F401 Boost PS
}
C 52900 59100 1 270 0 capacitor-1.sym
{
T 53600 58900 5 10 0 0 270 0 1
device=CAPACITOR
T 53800 58900 5 10 0 0 270 0 1
symversion=0.1
T 52600 58800 5 10 1 1 0 0 1
refdes=C32
T 53300 58500 5 10 1 1 0 0 1
value=1 uF
}
N 53100 58200 52700 58200 4
N 51300 59100 51300 58900 4
C 51200 58900 1 270 0 resistor-1.sym
{
T 51600 58600 5 10 0 0 270 0 1
device=R_2M2_1%_1608
T 51800 57700 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 51100 58100 5 10 1 1 90 0 1
refdes=R31
T 51600 58300 5 10 1 1 90 0 1
value=2M4
}
N 50000 58000 52200 58000 4
C 49800 59100 1 270 0 capacitor-1.sym
{
T 50500 58900 5 10 0 0 270 0 1
device=C_22PF_1%_1608
T 50700 58900 5 10 0 0 270 0 1
symversion=0.1
T 49400 58800 5 10 1 1 0 0 1
refdes=C3FF
T 50300 58400 5 10 1 1 0 0 1
value=22pF
T 49800 59100 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
N 50000 58200 50000 58000 4
C 54800 58200 1 270 1 schottky.sym
{
T 55400 58600 5 10 0 0 270 6 1
device=B5819WS
T 55650 58550 5 10 1 1 0 6 1
refdes=D31
T 55100 58300 5 10 1 1 0 0 1
footprint=SOD323_N.lht
}
N 54000 58000 54000 58200 4
N 54000 58200 55000 58200 4
N 50000 59100 55000 59100 4
{
T 54200 59150 5 10 1 1 0 0 1
netname=18V
}
N 53100 58000 53100 58200 4
C 55000 57100 1 270 0 coil-1.sym
{
T 55400 56900 5 10 0 0 270 0 1
device=COIL
T 55600 56900 5 10 0 0 270 0 1
symversion=0.1
T 55200 56700 5 10 1 1 0 0 1
refdes=L31
T 55100 57000 5 10 1 1 0 0 1
value=10 uH
T 55000 57100 5 10 0 0 0 0 1
footprint=chipind_3012_taiyoy.lht
}
C 51900 55500 1 0 0 dc-dc-sot23-5.sym
{
T 54300 57800 5 14 1 1 0 3 1
refdes=U3
T 52400 56700 5 10 1 1 0 0 1
device=TPS61040DBVR
T 52500 56000 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 55000 56100 55000 55300 4
N 54000 54900 54000 55500 4
N 52700 54300 52700 58200 4
N 48500 54300 55000 54300 4
N 51300 58000 51300 56900 4
N 51300 56000 51300 54300 4
N 52200 55400 52200 55500 4
N 52200 54500 52200 54300 4
C 54800 55200 1 270 0 capacitor-1.sym
{
T 55500 55000 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 54800 55200 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 54500 54800 5 10 1 1 0 0 1
refdes=C31
T 55200 54600 5 10 1 1 0 0 1
value=4.7 uF
}
C 52100 55400 1 270 0 resistor-1.sym
{
T 52500 55100 5 10 0 0 270 0 1
device=R_10K_1%_2013
T 52100 55400 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 52000 54600 5 10 1 1 90 0 1
refdes=R34
T 52500 54800 5 10 1 1 90 0 1
value=10k
}
C 51200 56900 1 270 0 resistor-1.sym
{
T 51600 56600 5 10 0 0 270 0 1
device=R_180K_1%_1608
T 51800 55900 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 51100 56100 5 10 1 1 90 0 1
refdes=R32
T 51600 56300 5 10 1 1 90 0 1
value=180k
}
N 55000 58200 55000 57100 4
N 54000 55300 55000 55300 4
N 52200 55500 50700 55500 4
C 50700 55600 1 180 0 resistor-1.sym
{
T 50400 55200 5 10 0 0 180 0 1
device=2013M R 220 1%
T 50700 55600 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 49900 55700 5 10 1 1 0 0 1
refdes=R33
T 50100 55200 5 10 1 1 0 0 1
value=220
}
C 49900 54700 1 90 0 jump-solder.sym
{
T 49500 55100 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 49900 54700 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 49500 55200 5 10 1 1 90 0 1
refdes=J5
}
N 54000 54900 49900 54900 4
N 49800 55500 47500 55500 4
C 45400 50300 1 0 0 input-hier.sym
{
T 45700 51000 5 10 0 0 0 0 1
device=none
T 45400 50300 5 10 1 1 0 0 1
refdes=5V
}
C 46400 55400 1 0 0 input-hier.sym
{
T 46700 56100 5 10 0 0 0 0 1
device=none
T 46500 55400 5 10 1 1 0 0 1
refdes=!SHDN1
}
C 45400 49400 1 0 0 input-hier.sym
{
T 45700 50100 5 10 0 0 0 0 1
device=none
T 45600 49400 5 10 1 1 0 0 1
refdes=GND
}
C 51200 48800 1 0 0 dc-dc-sot23-5.sym
{
T 53600 51100 5 14 1 1 0 3 1
refdes=U4
T 51700 50000 5 10 1 1 0 0 1
device=G5111T11UF
T 51700 49400 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 48500 47600 54300 47600 4
N 50500 48100 53300 48100 4
C 54300 50400 1 270 0 coil-1.sym
{
T 54700 50200 5 10 0 0 270 0 1
device=COIL
T 54900 50200 5 10 0 0 270 0 1
symversion=0.1
T 54500 50000 5 10 1 1 0 0 1
refdes=L41
}
N 54300 50400 54300 51500 4
N 54300 51500 53300 51500 4
N 54300 49400 54300 49100 4
N 53300 48100 53300 48800 4
C 54100 51500 1 270 1 schottky.sym
{
T 54700 51900 5 10 0 0 270 6 1
device=Schottky
T 54950 51850 5 10 1 1 0 6 1
refdes=D41
}
N 49600 52400 55800 52400 4
{
T 53500 52450 5 10 1 1 0 0 1
netname=18V
}
C 52200 52400 1 270 0 capacitor-1.sym
{
T 52900 52200 5 10 0 0 270 0 1
device=CAPACITOR
T 53100 52200 5 10 0 0 270 0 1
symversion=0.1
T 51900 52000 5 10 1 1 0 0 1
refdes=C42
T 52600 51800 5 10 1 1 0 0 1
value=1 uF
}
N 52400 51500 52400 51300 4
N 52400 51500 52000 51500 4
N 52000 47600 52000 51500 4
N 50600 52400 50600 52200 4
C 50500 52200 1 270 0 resistor-1.sym
{
T 50900 51900 5 10 0 0 270 0 1
device=2013M R 2.2 M 1%
T 50500 52200 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 50400 51400 5 10 1 1 90 0 1
refdes=R41
T 50900 51600 5 10 1 1 90 0 1
value=2.2 M
}
C 50500 50200 1 270 0 resistor-1.sym
{
T 50900 49900 5 10 0 0 270 0 1
device=2013M R 160 k 1%
T 50500 50200 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 50400 49400 5 10 1 1 90 0 1
refdes=R42
T 50900 49600 5 10 1 1 90 0 1
value=160 k
}
N 49600 51300 51500 51300 4
N 50600 51300 50600 50200 4
N 50600 49300 50600 47600 4
N 51500 48600 51500 48800 4
N 51500 47700 51500 47600 4
C 49400 52400 1 270 0 capacitor-1.sym
{
T 50100 52200 5 10 0 0 270 0 1
device=CAPACITOR
T 50300 52200 5 10 0 0 270 0 1
symversion=0.1
T 49100 51300 5 10 1 1 0 0 1
refdes=C4FF
T 49800 51800 5 10 1 1 0 0 1
value=1 uF
}
N 49600 51500 49600 51300 4
N 48100 48100 49600 48100 4
N 50200 48800 51500 48800 4
C 54100 49100 1 270 0 capacitor-1.sym
{
T 54800 48900 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 54100 49100 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 53800 48700 5 10 1 1 0 0 1
refdes=C41
T 54500 48500 5 10 1 1 0 0 1
value=4.7 uF
}
C 50200 48900 1 180 0 resistor-1.sym
{
T 49900 48500 5 10 0 0 180 0 1
device=2013M R 220 1%
T 50200 48900 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 49400 49000 5 10 1 1 0 0 1
refdes=R43
T 49600 48500 5 10 1 1 0 0 1
value=220
}
C 51400 48600 1 270 0 resistor-1.sym
{
T 51800 48300 5 10 0 0 270 0 1
device=2013M R 10k 1%
T 51400 48600 5 10 0 0 90 0 1
footprint=RESC2013N.lht
T 51300 47800 5 10 1 1 90 0 1
refdes=R44
T 51800 48000 5 10 1 1 90 0 1
value=10 k
}
N 53300 51300 53300 51500 4
N 54300 48200 54300 47600 4
C 50500 47900 1 90 0 jump-solder.sym
{
T 50100 48300 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 50500 47900 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 50100 48500 5 10 1 1 180 0 1
refdes=J6
}
N 48500 41500 54000 41500 4
N 48100 42000 49100 42000 4
N 48100 54900 48100 42000 4
N 48100 54900 49000 54900 4
C 46400 48700 1 0 0 input-hier.sym
{
T 46700 49400 5 10 0 0 0 0 1
device=none
T 46400 48700 5 10 1 1 0 0 1
refdes=!SHDN2
}
N 49300 48800 47500 48800 4
N 48800 42700 47500 42700 4
C 46400 42600 1 0 0 input-hier.sym
{
T 46700 43300 5 10 0 0 0 0 1
device=none
T 46500 42600 5 10 1 1 0 0 1
refdes=!SHDN3
}
C 55800 52300 1 0 0 output-hier.sym
{
T 56000 52800 5 10 0 0 0 0 1
device=none
T 56100 52300 5 10 1 1 0 0 1
refdes=18V
}
N 48500 54300 48500 41500 4
C 50900 42700 1 0 0 dc-dc-sot23-5.sym
{
T 53300 45000 5 14 1 1 0 3 1
refdes=U5
T 51400 43900 5 10 1 1 0 0 1
device=AP3012KTR-G1
T 51400 43300 5 10 1 1 0 0 1
footprint=SOT23-5.lht
}
N 50000 42000 53000 42000 4
C 54000 44600 1 270 0 coil-1.sym
{
T 54400 44400 5 10 0 0 270 0 1
device=COIL
T 54600 44400 5 10 0 0 270 0 1
symversion=0.1
T 53600 44200 5 10 1 1 0 0 1
refdes=L51
}
N 54000 44600 54000 45250 4
N 54000 45250 53000 45250 4
N 53000 45250 53000 45200 4
N 54000 43600 54000 42400 4
N 53000 42000 53000 42700 4
C 53800 45250 1 270 1 schottky.sym
{
T 54400 45650 5 10 0 0 270 6 1
device=Schottky
T 54650 45700 5 10 1 1 0 6 1
refdes=D51
}
N 54000 46150 54000 46600 4
C 51900 46600 1 270 0 capacitor-1.sym
{
T 52600 46400 5 10 0 0 270 0 1
device=CAPACITOR
T 52800 46400 5 10 0 0 270 0 1
symversion=0.1
T 51600 46200 5 10 1 1 0 0 1
refdes=C52
T 52300 46000 5 10 1 1 0 0 1
value=1 uF
}
N 52100 45700 52100 45200 4
N 52100 45400 51700 45400 4
N 51700 41500 51700 45400 4
N 50300 46600 50300 46400 4
C 50200 46400 1 270 0 resistor-1.sym
{
T 50600 46100 5 10 0 0 270 0 1
device=RESISTOR
T 50200 46400 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 50100 45600 5 10 1 1 90 0 1
refdes=R51
T 50600 45800 5 10 1 1 90 0 1
value=2.2 M
}
C 50200 44400 1 270 0 resistor-1.sym
{
T 50600 44100 5 10 0 0 270 0 1
device=RESISTOR
T 50200 44400 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 50100 43600 5 10 1 1 90 0 1
refdes=R52
T 50600 43800 5 10 1 1 90 0 1
value=160 k
}
N 49400 45500 51200 45500 4
N 51200 45500 51200 45200 4
N 50300 45500 50300 44400 4
N 50300 43500 50300 41500 4
N 51200 42700 49700 42700 4
C 49700 42800 1 180 0 resistor-1.sym
{
T 49400 42400 5 10 0 0 180 0 1
device=2013M R 220 1%
T 49700 42800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 48900 42900 5 10 1 1 0 0 1
refdes=R53
T 49100 42400 5 10 1 1 0 0 1
value=220
}
C 51100 42600 1 270 0 resistor-1.sym
{
T 51500 42300 5 10 0 0 270 0 1
device=RESISTOR
T 51100 42600 5 10 0 0 90 0 1
footprint=0805_HV.fp
T 51000 41800 5 10 1 1 90 0 1
refdes=R54
T 51500 42000 5 10 1 1 90 0 1
value=10 k
}
N 51200 42600 51200 42700 4
N 51200 41700 51200 41500 4
C 49200 46600 1 270 0 capacitor-1.sym
{
T 49900 46400 5 10 0 0 270 0 1
device=CAPACITOR
T 50100 46400 5 10 0 0 270 0 1
symversion=0.1
T 48900 46200 5 10 1 1 0 0 1
refdes=C5FF
T 49600 46000 5 10 1 1 0 0 1
value=1 uF
}
N 49400 45700 49400 45500 4
C 53800 42400 1 270 0 capacitor-1.sym
{
T 54500 42200 5 10 0 0 270 0 1
device=CL10A475MO8NNNC
T 53800 42400 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 53500 42000 5 10 1 1 0 0 1
refdes=C51
T 54200 41800 5 10 1 1 0 0 1
value=4.7 uF
}
C 50000 41800 1 90 0 jump-solder.sym
{
T 49600 42200 5 8 0 0 90 0 1
device=JUMPER SOLDER
T 50000 41800 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 49700 42400 5 10 1 1 180 0 1
refdes=J7
}
N 49400 46600 54000 46600 4
{
T 51600 46650 5 10 1 1 0 0 1
netname=18V
}
N 46500 49500 48500 49500 4
N 46500 50400 48100 50400 4
T 53400 39950 5 16 1 1 0 0 1
title=Culture Shock
T 51200 59600 9 12 1 0 0 0 1
Choose Cff for 390k R2
T 51100 59300 9 12 1 0 0 0 1
Cff =  3pF X (200k/R2 -1)
