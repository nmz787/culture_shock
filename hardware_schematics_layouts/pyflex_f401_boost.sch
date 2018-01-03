v 20150930 2
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
rev=2017-12-18 v0.4
T 66800 45200 5 18 1 1 270 0 1
title2=PYFLEX_F401 Boost PS
}
C 52900 58800 1 270 0 capacitor-1.sym
{
T 53600 58600 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 53200 58500 5 10 1 1 0 0 1
refdes=C32
T 53200 58100 5 10 1 1 0 0 1
value=1 uF
T 52900 58800 5 10 0 0 0 0 1
footprint=CAPC2013N.lht
T 52900 58800 5 10 0 0 0 0 1
description=MLCC 1uF 50V X7R 10% 2013
}
N 53100 57900 52800 57900 4
N 51200 58800 51200 58600 4
C 51100 58600 1 270 0 resistor-1.sym
{
T 51500 58300 5 10 0 0 270 0 1
device=R_2M2_1%_1608
T 51700 57400 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 51000 57800 5 10 1 1 90 0 1
refdes=R31
T 51500 58000 5 10 1 1 90 0 1
value=2M4
}
N 50100 57700 52200 57700 4
C 54800 57900 1 270 1 schottky.sym
{
T 56100 58100 5 10 1 1 0 6 1
device=B5819WS
T 55650 58350 5 10 1 1 0 6 1
refdes=D31
T 55100 58000 5 10 0 0 0 0 1
footprint=SOD323_N.lht
T 54800 57900 5 10 0 0 0 0 1
value=semicon
}
N 54000 57700 54000 57900 4
N 54000 57900 55000 57900 4
N 53100 57700 53100 57900 4
C 55000 56800 1 270 0 coil-1.sym
{
T 55400 56600 5 10 0 0 270 0 1
device=NRH2410T100MN
T 55200 56100 5 10 1 1 0 0 1
refdes=L31
T 55200 56400 5 10 1 1 0 0 1
value=10 uH
T 55000 56800 5 10 0 0 0 0 1
footprint=INDC2424N.lht
}
C 51900 55200 1 0 0 dc-dc-sot23-5.sym
{
T 54300 57500 5 14 1 1 0 3 1
refdes=U3
T 52400 56400 5 10 1 1 0 0 1
device=TPS61040DBVR
T 52500 55700 5 10 1 1 0 0 1
footprint=SOT23-5.lht
T 51900 55200 5 10 0 0 0 0 1
value=IC
}
N 55000 55800 55000 55200 4
N 52800 56700 52800 58100 4
N 52200 54000 55000 54000 4
N 52200 55100 52200 55200 4
N 52200 54200 52200 54000 4
C 52100 55100 1 270 0 resistor-1.sym
{
T 52500 54800 5 10 0 0 270 0 1
device=R_7.5K_1%_1608
T 52100 55100 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 52000 54300 5 10 1 1 90 0 1
refdes=R34
T 52500 54500 5 10 1 1 90 0 1
value=7.5k
}
C 52800 58200 1 180 0 resistor-1.sym
{
T 52500 57800 5 10 0 0 180 0 1
device=R_180K_1%_1608
T 51800 57600 5 10 0 1 0 0 1
footprint=RESC1608N.lht
T 52000 58300 5 10 1 1 0 0 1
refdes=R32
T 52200 57800 5 10 1 1 0 0 1
value=180k
}
N 55000 57900 55000 56800 4
N 49900 53500 54000 53500 4
N 52200 55200 50500 55200 4
C 50500 55300 1 180 0 resistor-1.sym
{
T 50200 54900 5 10 0 0 180 0 1
device=R 200 1% 2013
T 50500 55300 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 49700 55400 5 10 1 1 0 0 1
refdes=R33
T 49900 54900 5 10 1 1 0 0 1
value=200
}
C 49900 53300 1 90 0 jump-solder.sym
{
T 49500 53700 5 8 0 0 90 0 1
device=JUMPER_SOLDER
T 49900 53300 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 49500 53800 5 10 1 1 90 0 1
refdes=J31
T 49900 53300 5 10 0 0 0 0 1
value=conn
}
N 49600 55200 47500 55200 4
C 45400 50500 1 0 0 input-hier.sym
{
T 45700 51100 5 10 0 0 0 0 1
value=pageconn
T 45700 51000 5 10 0 0 0 0 1
footprint=none
T 45400 50500 5 10 1 1 0 0 1
refdes=5V
T 45400 50500 5 10 0 0 0 0 1
device=pageconn
}
C 46400 55100 1 0 0 input-hier.sym
{
T 46700 55700 5 10 0 0 0 0 1
value=pageconn
T 46700 55600 5 10 0 0 0 0 1
footprint=none
T 46500 55100 5 10 1 1 0 0 1
refdes=!SHDN1
T 46400 55100 5 10 0 0 0 0 1
device=pageconn
}
C 45400 49600 1 0 0 input-hier.sym
{
T 45700 50200 5 10 0 0 0 0 1
value=pageconn
T 45700 50100 5 10 0 0 0 0 1
footprint=none
T 45600 49600 5 10 1 1 0 0 1
refdes=GND
T 45400 49600 5 10 0 0 0 0 1
device=pageconn
}
N 48100 47200 49600 47200 4
C 50500 47000 1 90 0 jump-solder.sym
{
T 50100 47400 5 8 0 0 90 0 1
device=JUMPER_SOLDER
T 50500 47000 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 50100 47600 5 10 1 1 180 0 1
refdes=J41
T 50500 47000 5 10 0 0 0 0 1
value=conn
}
N 48100 40500 48300 40500 4
N 48100 53500 48100 40500 4
N 48100 53500 49000 53500 4
C 46400 48600 1 0 0 input-hier.sym
{
T 46700 49200 5 10 0 0 0 0 1
value=pageconn
T 46400 48600 5 10 1 1 0 0 1
refdes=!SHDN2
T 46400 48600 5 10 0 0 0 0 1
device=pageconn
T 46400 48600 5 10 0 0 0 0 1
footprint=none
}
N 51300 48700 47500 48700 4
N 52100 42300 47500 42300 4
C 46400 42200 1 0 0 input-hier.sym
{
T 46700 42800 5 10 0 0 0 0 1
value=pageconn
T 46500 42200 5 10 1 1 0 0 1
refdes=!SHDN3
T 46400 42200 5 10 0 0 0 0 1
device=pageconn
T 46400 42200 5 10 0 0 0 0 1
footprint=none
}
C 62000 54200 1 0 0 output-hier.sym
{
T 62200 54800 5 10 0 0 0 0 1
value=pageconn
T 62300 54200 5 10 1 1 0 0 1
refdes=18V
T 62000 54200 5 10 0 0 0 0 1
device=pageconn
T 62000 54200 5 10 0 0 0 0 1
footprint=none
}
N 49200 40500 55400 40500 4
C 49200 40300 1 90 0 jump-solder.sym
{
T 48800 40700 5 8 0 0 90 0 1
device=JUMPER_SOLDER
T 49200 40300 5 10 0 0 0 0 1
footprint=jump-solder.lht
T 48900 40900 5 10 1 1 180 0 1
refdes=J51
T 49200 40300 5 10 0 0 0 0 1
value=conn
}
N 46500 49700 47400 49700 4
N 46500 50600 48100 50600 4
T 53400 39950 5 16 1 1 0 0 1
title=Culture Shock
T 47100 58300 9 12 1 0 0 0 1
Choose Cff for 180k R2
T 46400 58000 9 12 1 0 0 0 1
Cff =  3pF X (200k/R2 -1) ~=> zero
N 51900 58100 51900 57700 4
N 52800 56700 53100 56700 4
N 53100 56700 53100 54000 4
N 54000 55200 55000 55200 4
T 55700 54700 9 10 1 0 0 0 1
substituted 10uF for 4.7uF (min)
C 51800 59400 1 270 0 pnp-bec-123.sym
{
T 52800 58900 5 10 1 1 0 0 1
device=BC857B
T 52600 59100 5 10 1 1 0 0 1
refdes=Q31
T 51800 59400 5 10 0 0 0 0 1
footprint=SOT23-3N.lht
T 51800 59400 5 10 0 0 0 0 1
description=Hfe=220 Vcemax=45 Vbemax=5V
T 51800 59400 5 10 0 0 0 0 1
value=semicon
}
N 52300 59400 56000 59400 4
N 56000 59400 56000 57800 4
C 55900 57800 1 270 0 resistor-1.sym
{
T 56300 57500 5 10 0 0 270 0 1
device=R_180K_1%_1608
T 56500 56800 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 55800 57000 5 10 1 1 90 0 1
refdes=R35
T 56300 57200 5 10 1 1 90 0 1
value=180k
}
N 56000 56900 56000 55500 4
N 56000 55500 55000 55500 4
C 55200 54300 1 90 0 capacitor-1.sym
{
T 54500 54500 5 10 0 0 90 0 1
device=CL10A475MO8NNNC
T 55200 54300 5 10 0 0 180 0 1
footprint=CAPC3216N.lht
T 55100 54400 5 10 1 1 0 0 1
refdes=C31
T 55100 54900 5 10 1 1 0 0 1
value=10 uF
}
N 54000 53500 54000 55200 4
N 55000 54000 55000 54300 4
N 50100 58800 51800 58800 4
N 55000 58800 52800 58800 4
C 53500 52300 1 270 0 capacitor-1.sym
{
T 54200 52100 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 53500 52300 5 10 0 0 0 0 1
footprint=CAPC2013N.lht
T 53500 52300 5 10 0 0 0 0 1
description=MLCC 1uF 50V X7R 10% 2013
T 53800 52000 5 10 1 1 0 0 1
refdes=C42
T 53800 51600 5 10 1 1 0 0 1
value=1 uF
}
N 53700 51400 53400 51400 4
N 51800 52300 51800 52100 4
C 51700 52100 1 270 0 resistor-1.sym
{
T 52100 51800 5 10 0 0 270 0 1
device=R_2M4_1%_1608
T 52300 50900 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 51600 51300 5 10 1 1 90 0 1
refdes=R41
T 52100 51500 5 10 1 1 90 0 1
value=2M4
}
N 50700 51200 52800 51200 4
N 54600 51200 54600 51400 4
N 54600 51400 55600 51400 4
N 53700 51200 53700 51400 4
C 53400 51700 1 180 0 resistor-1.sym
{
T 53100 51300 5 10 0 0 180 0 1
device=R_169K_1%_1608
T 52400 51100 5 10 0 1 0 0 1
footprint=RESC1608N.lht
T 52600 51800 5 10 1 1 0 0 1
refdes=R42
T 52800 51300 5 10 1 1 0 0 1
value=169k
}
N 52500 51600 52500 51200 4
C 52400 52900 1 270 0 pnp-bec-123.sym
{
T 53400 52400 5 10 1 1 0 0 1
device=BC857B
T 52400 52900 5 10 0 0 0 0 1
footprint=SOT23-3N.lht
T 52400 52900 5 10 0 0 0 0 1
description=Hfe=220 Vcemax=45 Vbemax=5V
T 53200 52600 5 10 1 1 0 0 1
refdes=Q41
T 52400 52900 5 10 0 0 0 0 1
value=semicon
}
N 52900 52900 56600 52900 4
N 56600 52900 56600 51300 4
N 50700 52300 52400 52300 4
N 55600 52300 53400 52300 4
C 55600 50300 1 270 0 coil-1.sym
{
T 56000 50100 5 10 0 0 270 0 1
device=NRS2012T4R7MGJ
T 55600 50300 5 10 0 0 0 0 1
footprint=INDC2020N.lht
T 55800 49600 5 10 1 1 0 0 1
refdes=L41
T 55800 49900 5 10 1 1 0 0 1
value=4.7 uH
}
N 55600 49300 55600 48700 4
N 55600 51400 55600 50300 4
N 54600 48700 55600 48700 4
C 56500 51300 1 270 0 resistor-1.sym
{
T 56900 51000 5 10 0 0 270 0 1
device=R_180K_1%_1608
T 57100 50300 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 56400 50500 5 10 1 1 90 0 1
refdes=R45
T 56900 50700 5 10 1 1 90 0 1
value=180k
}
N 56600 48700 56600 50400 4
N 56600 49000 55600 49000 4
C 55800 47800 1 90 0 capacitor-1.sym
{
T 55100 48000 5 10 0 0 90 0 1
device=CL10A475MO8NNNC
T 55800 47800 5 10 0 0 180 0 1
footprint=CAPC3216N.lht
T 55700 47900 5 10 1 1 0 0 1
refdes=C41
T 55700 48400 5 10 1 1 0 0 1
value=10 uF
}
N 54600 47200 54600 48700 4
N 55600 47500 55600 47800 4
N 52800 47500 56600 47500 4
N 52800 48600 52800 48700 4
N 52800 47700 52800 47500 4
C 52700 48600 1 270 0 resistor-1.sym
{
T 53100 48300 5 10 0 0 270 0 1
device=R_7.5K_1%_1608
T 52700 48600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 52600 47800 5 10 1 1 90 0 1
refdes=R44
T 53100 48000 5 10 1 1 90 0 1
value=7.5k
}
N 50500 47200 54600 47200 4
N 52800 48700 52200 48700 4
C 52200 48800 1 180 0 resistor-1.sym
{
T 51900 48400 5 10 0 0 180 0 1
device=R 200 1% 2013
T 52200 48800 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 51400 48900 5 10 1 1 0 0 1
refdes=R43
T 51600 48400 5 10 1 1 0 0 1
value=200
}
N 53400 50200 53400 51600 4
N 53400 50200 53700 50200 4
N 53700 50200 53700 47500 4
T 57600 48200 9 10 1 0 0 0 1
substituted 10uF for  22uF (suggested)
C 52500 48700 1 0 0 dc-dc-sot23-5.sym
{
T 54900 51000 5 14 1 1 0 3 1
refdes=U4
T 53000 49900 5 10 1 1 0 0 1
device=G5111T11UF
T 53000 49300 5 10 1 1 0 0 1
footprint=SOT23-5.lht
T 52500 48700 5 10 0 0 0 0 1
value=IC
}
N 61400 54300 62000 54300 4
C 54300 45900 1 270 0 capacitor-1.sym
{
T 55000 45700 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 54300 45900 5 10 0 0 0 0 1
footprint=CAPC2013N.lht
T 54300 45900 5 10 0 0 0 0 1
description=MLCC 1uF 50V X7R 10% 2013
T 54600 45600 5 10 1 1 0 0 1
refdes=C52
T 54600 45200 5 10 1 1 0 0 1
value=1 uF
}
N 54500 45000 54200 45000 4
N 52600 45900 52600 45700 4
C 52500 45700 1 270 0 resistor-1.sym
{
T 52900 45400 5 10 0 0 270 0 1
device=R_2M2_1%_1608
T 53100 44500 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 52400 44900 5 10 1 1 90 0 1
refdes=R51
T 52900 45100 5 10 1 1 90 0 1
value=2M4
}
N 52600 44800 53600 44800 4
C 56200 45000 1 270 1 schottky.sym
{
T 57500 45200 5 10 1 1 0 6 1
device=B5819WS
T 57050 45450 5 10 1 1 0 6 1
refdes=D51
T 56500 45100 5 10 0 1 0 0 1
footprint=SOD323_N.lht
T 56200 45000 5 10 0 0 0 0 1
value=semicon
}
N 55400 44800 55400 45000 4
N 55400 45000 56400 45000 4
N 54500 44800 54500 45000 4
C 54200 45300 1 180 0 resistor-1.sym
{
T 53900 44900 5 10 0 0 180 0 1
device=R_180K_1%_1608
T 53200 44700 5 10 0 1 0 0 1
footprint=RESC1608N.lht
T 53400 45400 5 10 1 1 0 0 1
refdes=R52
T 53600 44900 5 10 1 1 0 0 1
value=180k
}
N 53300 45200 53300 44800 4
N 53700 46500 57400 46500 4
N 57400 46500 57400 44900 4
N 52600 45900 53200 45900 4
N 56400 45900 54200 45900 4
N 56400 42900 56400 42300 4
N 56400 45000 56400 43900 4
N 55400 42300 56400 42300 4
C 57300 44900 1 270 0 resistor-1.sym
{
T 57700 44600 5 10 0 0 270 0 1
device=R_180K_1%_1608
T 57900 43900 5 10 0 1 90 0 1
footprint=RESC1608N.lht
T 57200 44100 5 10 1 1 90 0 1
refdes=R55
T 57700 44300 5 10 1 1 90 0 1
value=180k
}
N 57400 44000 57400 42600 4
N 57400 42600 56400 42600 4
C 56600 41400 1 90 0 capacitor-1.sym
{
T 55900 41600 5 10 0 0 90 0 1
device=CL10A475MO8NNNC
T 56600 41400 5 10 0 0 180 0 1
footprint=CAPC3216N.lht
T 56500 41500 5 10 1 1 0 0 1
refdes=C51
T 56500 42000 5 10 1 1 0 0 1
value=10 uF
}
N 55400 40500 55400 42300 4
N 56400 41100 56400 41400 4
N 53600 41100 56400 41100 4
N 53600 42200 53600 42300 4
N 53600 41300 53600 41100 4
C 53500 42200 1 270 0 resistor-1.sym
{
T 53900 41900 5 10 0 0 270 0 1
device=R_7.5K_1%_1608
T 53500 42200 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 53400 41400 5 10 1 1 90 0 1
refdes=R54
T 53900 41600 5 10 1 1 90 0 1
value=7.5k
}
N 53600 42300 53000 42300 4
C 53000 42400 1 180 0 resistor-1.sym
{
T 52700 42000 5 10 0 0 180 0 1
device=R 200 1% 2013
T 53000 42400 5 10 0 0 0 0 1
footprint=RESC2013N.lht
T 52200 42500 5 10 1 1 0 0 1
refdes=R53
T 52400 42000 5 10 1 1 0 0 1
value=200
}
N 54200 43800 54200 45200 4
N 54200 43800 54500 43800 4
N 54500 43800 54500 41100 4
T 57100 41800 9 10 1 0 0 0 1
substituted 10uF for 4.7uF
C 53300 42300 1 0 0 dc-dc-sot23-5.sym
{
T 55700 44600 5 14 1 1 0 3 1
refdes=U5
T 53800 43500 5 10 1 1 0 0 1
device=AP3012KTR-G1
T 53800 42900 5 10 1 1 0 0 1
footprint=SOT23-5.lht
T 53300 42300 5 10 0 0 0 0 1
value=IC
}
T 50100 43700 9 10 1 0 0 0 1
18V/1.25 +1 = R1/2.4M  R2= 2.4M
T 49400 56800 9 10 1 0 0 0 1
1.0 MHz switcher TPS61041
T 49800 49900 9 10 1 0 0 0 2
1.0 MHz switcher G5111T11UF

C 49900 58700 1 270 0 capacitor-1.sym
{
T 50600 58500 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 50200 58400 5 10 1 1 0 0 1
refdes=C33
T 50200 58000 5 10 1 1 0 0 1
value=3.3pF
T 49900 58700 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 49900 58700 5 10 0 0 0 0 1
description=MLCC 3.3pF 50V COG
}
N 50100 58800 50100 58700 4
N 50100 57700 50100 57800 4
T 48800 52500 9 10 1 0 0 0 1
18.24V with 169k and 2.4M R41 R42
C 50500 52200 1 270 0 capacitor-1.sym
{
T 51200 52000 5 10 0 0 270 0 1
device=MLCC_8.2pF_50V
T 50500 52200 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
T 50500 52200 5 10 0 0 0 0 1
description=MLCC 8.2pF 50V COG
T 50800 51900 5 10 1 1 0 0 1
refdes=C43
T 50800 51500 5 10 1 1 0 0 1
value=8.2pF
}
N 50700 52300 50700 52200 4
N 50700 51200 50700 51300 4
C 55400 51400 1 270 1 schottky.sym
{
T 56700 51600 5 10 1 1 0 6 1
device=B5819WS
T 56250 51850 5 10 1 1 0 6 1
refdes=D41
T 55700 51500 5 10 0 0 0 0 1
footprint=SOD323_N.lht
T 55400 51400 5 10 0 0 0 0 1
value=semicon
}
C 53200 46500 1 270 0 pnp-bec-123.sym
{
T 54200 46000 5 10 1 1 0 0 1
device=BC857B
T 53200 46500 5 10 0 0 0 0 1
footprint=SOT23-3N.lht
T 53200 46500 5 10 0 0 0 0 1
description=Hfe=220 Vcemax=45 Vbemax=5V
T 54000 46200 5 10 1 1 0 0 1
refdes=Q51
T 53200 46500 5 10 0 0 0 0 1
value=semicon
}
T 49800 49900 9 10 1 0 0 0 1
1.0 MHz switcher TPS61041
T 49400 56400 9 10 1 0 0 0 2
1.0 MHz switcher G5111T11UF

T 49400 56200 9 10 1 0 0 0 2
10 Uh 2424 footprint

T 49700 49000 9 10 1 0 0 0 2
4.7 Uh 2020 footprint

C 56800 47800 1 90 0 capacitor-1.sym
{
T 56100 48000 5 10 0 0 90 0 1
device=CL10A475MO8NNNC
T 56800 47800 5 10 0 0 180 0 1
footprint=CAPC3216N.lht
T 56700 47900 5 10 1 1 0 0 1
refdes=C44
T 56700 48400 5 10 1 1 0 0 1
value=10 uF
}
N 56600 47500 56600 47800 4
T 50000 44300 9 10 1 0 0 0 1
FB pin voltage = 1.25 (1.xx to 1.xxV)
T 49400 50700 9 10 1 0 0 0 1
FB pin voltage = 1.20 (1.xx to 1.xxV)
T 50600 43500 9 10 1 0 0 0 1
1.5 MHz switcher AP3012
T 49300 52800 9 10 1 0 0 0 1
(18V/1.25 -1)R2 = R1  R2= 169k
C 56400 43900 1 270 0 coil-1.sym
{
T 56800 43700 5 10 0 0 270 0 1
device=NRS2012T4R7MGJ
T 56400 43900 5 10 0 0 0 0 1
footprint=INDC2020N.lht
T 56600 43200 5 10 1 1 0 0 1
refdes=L51
T 56600 43500 5 10 1 1 0 0 1
value=4.7 uH
}
N 52800 45900 52800 46800 4
N 52800 46800 61400 46800 4
N 61400 46800 61400 59700 4
N 51500 58800 51500 59700 4
N 51500 59700 61400 59700 4
N 52200 52300 52200 53200 4
N 52200 53200 61400 53200 4
C 54400 53700 1 0 0 vss.sym
C 56200 47200 1 0 0 vss.sym
C 56100 40800 1 0 0 vss.sym
C 47300 49400 1 0 0 vss.sym
