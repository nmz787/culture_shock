v 20150930 2
N 58800 52100 59800 52100 4
{
T 59300 52200 5 10 1 1 0 0 1
netname=doubler_out
}
C 57900 51900 1 0 0 diode-1.sym
{
T 58300 52500 5 10 0 0 0 0 1
device=M20F
T 58500 52500 5 10 1 1 180 0 1
refdes=D4
T 58600 51800 5 10 1 1 180 0 1
value=2KV
T 57900 51900 5 10 0 0 0 0 1
description=diode rect 2000V 1A SMAF
T 57900 51900 5 10 0 0 0 0 1
footprint=DIOM_SMAFN.lht
}
C 59600 50900 1 270 0 capacitor-1.sym
{
T 60300 50700 5 10 0 0 270 0 1
device=472-3KV
T 59900 50600 5 10 1 1 0 0 1
refdes=C3
T 60500 50700 5 10 0 0 270 0 1
symversion=0.1
T 59300 50200 5 10 1 1 0 0 1
value=4.7nF
T 59600 50900 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat.fp
}
C 56500 52300 1 180 0 capacitor-1.sym
{
T 56300 51600 5 10 0 0 180 0 1
device=472-3KV
T 55700 52300 5 10 1 1 0 0 1
refdes=C1
T 56300 51400 5 10 0 0 180 0 1
symversion=0.1
T 55800 51700 5 10 1 1 0 0 1
value=4.7nF
T 56500 52300 5 10 0 0 180 0 1
footprint=disk_cap_5mm_flat.fp
}
N 56500 52100 57000 52100 4
{
T 56700 52100 5 10 1 1 0 0 1
netname=v2
}
N 56500 52100 56500 53000 4
N 56600 48900 56600 49400 4
N 59800 50900 59800 52100 4
C 56500 53200 1 180 0 capacitor-1.sym
{
T 56300 52500 5 10 0 0 180 0 1
device=472-3KV
T 55700 53100 5 10 1 1 0 0 1
refdes=C2
T 56300 52300 5 10 0 0 180 0 1
symversion=0.1
T 55800 52600 5 10 1 1 0 0 1
value=4.7nF
T 56500 53200 5 10 0 0 180 0 1
footprint=disk_cap_5mm_flat_flip.fp
}
N 58100 44700 60800 44700 4
N 59800 48900 59800 50000 4
N 59800 51800 63100 51800 4
N 55600 53000 55500 53000 4
N 55500 53000 55500 52100 4
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
rev=2018-04-29 v0.6
T 64400 43100 5 18 1 1 270 0 1
title2=Culture Shock kvboard PCB
}
C 54000 50500 1 0 0 transformer-epc19.sym
{
T 54150 52200 5 10 1 1 0 0 1
refdes=T1
T 54000 52300 5 10 0 1 0 0 1
device=CS1-HVT-EPC19-04
T 54000 50500 5 10 0 0 0 6 1
description=1500V transformer
T 54000 50500 5 10 0 0 0 6 1
footprint=TXFMR_EPC19.lht
T 54000 50500 5 10 0 0 0 6 1
value=xfmr
}
C 61600 51300 1 0 0 resistor-1.sym
{
T 61900 51700 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 51600 5 10 1 1 0 0 1
refdes=R1
T 62000 51100 5 10 1 1 0 0 1
value=475K
T 61600 51300 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 51300 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 50500 1 0 0 resistor-1.sym
{
T 61900 50900 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 50800 5 10 1 1 0 0 1
refdes=R2
T 62000 50300 5 10 1 1 0 0 1
value=475K
T 61600 50500 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 50500 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
N 54000 51700 54000 51100 4
T 56300 47000 9 12 1 0 0 0 6
2500V divided by 800 
attenuation = 
3.125V, which is 
compatible with 
the 3.3V supply 
to the MCUs ADCs.
N 55400 45600 56400 45600 4
{
T 55500 45750 5 10 1 1 180 6 1
netname=HV_sense
}
N 56000 50700 56000 48900 4
N 56000 48900 59800 48900 4
N 53500 52100 54000 52100 4
T 63950 42450 9 12 1 0 90 0 2
spring contact T2 
is a lone contact.
T 63850 44950 9 12 1 0 90 0 1
spring                     contacts                     to hold                   cuvette
N 53200 53700 52700 53700 4
N 51500 53700 50300 53700 4
N 50400 49100 51800 49100 4
C 58900 51200 1 270 0 capacitor-1.sym
{
T 59600 51000 5 10 0 0 270 0 1
device=472-3KV
T 59200 50900 5 10 1 1 0 0 1
refdes=C4
T 59800 51000 5 10 0 0 270 0 1
symversion=0.1
T 58600 50500 5 10 1 1 0 0 1
value=4.7nF
T 58900 51200 5 10 0 0 0 0 1
footprint=disk_cap_5mm_flat_flip.fp
}
N 59100 51200 59100 52100 4
N 59100 48900 59100 50300 4
N 58900 45200 58900 44700 4
L 54800 53200 54800 52600 3 10 1 0 -1 -1
L 54800 54000 55400 54000 3 10 1 0 -1 -1
L 55700 54000 56500 54000 3 10 1 0 -1 -1
L 64300 54000 64300 53300 3 10 1 0 -1 -1
L 63500 46700 63000 46700 3 10 1 0 -1 -1
T 60200 53600 9 22 1 0 0 0 1
HV Zone
L 54800 54000 54800 53500 3 10 1 0 -1 -1
L 57900 54000 58700 54000 3 10 1 0 -1 -1
L 56800 54000 57600 54000 3 10 1 0 -1 -1
L 59000 54000 59800 54000 3 10 1 0 -1 -1
L 60100 54000 60900 54000 3 10 1 0 -1 -1
L 61200 54000 61800 54000 3 10 1 0 -1 -1
L 62100 54000 62600 54000 3 10 1 0 -1 -1
L 62900 54000 63400 54000 3 10 1 0 -1 -1
L 64300 52200 64300 53000 3 10 1 0 -1 -1
L 64300 51100 64300 51900 3 10 1 0 -1 -1
L 64300 50000 64300 50800 3 10 1 0 -1 -1
L 64300 48900 64300 49700 3 10 1 0 -1 -1
L 57700 49500 58500 49500 3 10 1 0 -1 -1
L 58500 46700 59400 46700 3 10 1 0 -1 -1
L 59700 46700 60400 46700 3 10 1 0 -1 -1
L 60700 46700 61500 46700 3 10 1 0 -1 -1
L 61800 46700 62600 46700 3 10 1 0 -1 -1
T 58700 49500 9 22 1 0 0 0 1
HV Zone
L 55800 49500 55800 49700 3 10 1 0 -1 -1
C 61600 49700 1 0 0 resistor-1.sym
{
T 61900 50100 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 50000 5 10 1 1 0 0 1
refdes=R3
T 62000 49500 5 10 1 1 0 0 1
value=475K
T 61600 49700 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 49700 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 48900 1 0 0 resistor-1.sym
{
T 61900 49300 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 49200 5 10 1 1 0 0 1
refdes=R4
T 62000 48700 5 10 1 1 0 0 1
value=475K
T 61600 48900 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 48900 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 48100 1 0 0 resistor-1.sym
{
T 61900 48500 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 48400 5 10 1 1 0 0 1
refdes=R5
T 62000 47900 5 10 1 1 0 0 1
value=475K
T 61600 48100 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 48100 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 61600 47300 1 0 0 resistor-1.sym
{
T 61900 47700 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 61700 47600 5 10 1 1 0 0 1
refdes=R6
T 62000 47100 5 10 1 1 0 0 1
value=475K
T 61600 47300 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 61600 47300 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 60400 47300 1 0 0 resistor-1.sym
{
T 60700 47700 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 60500 47600 5 10 1 1 0 0 1
refdes=R7
T 60800 47100 5 10 1 1 0 0 1
value=475K
T 60400 47300 5 10 0 0 0 0 1
footprint=RESC3216N.lht
T 60400 47300 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
}
C 59200 47300 1 0 0 resistor-1.sym
{
T 59500 47700 5 10 0 0 0 0 1
device=RC1206FR-07475KL
T 59300 47600 5 10 1 1 0 0 1
refdes=R8
T 59600 47100 5 10 1 1 0 0 1
value=475K
T 59200 47300 5 10 0 0 0 0 1
description=RES SMD 475K OHM 1% 1/4W 1206
T 59200 47300 5 10 0 0 0 0 1
footprint=RESC3216N.lht
}
N 61600 51400 61600 51800 4
N 52300 46600 52300 48600 4
N 52000 56100 52000 54200 4
N 50400 46600 51000 46600 4
N 51900 46600 52700 46600 4
L 56700 49500 57300 49500 3 10 1 0 -1 -1
L 55800 49500 56400 49500 3 10 1 0 -1 -1
N 50400 46600 50400 49900 4
T 47000 54800 9 12 1 0 0 0 11
ZIF connector pin 
numbering is not 
connected 1 to 1,
but by FFC cable 
with same side up 
on both boards.
Connector left pin 
connects to other 
end connector right 
pin as a signal goes 
through FFC cable.
N 55500 52100 55600 52100 4
N 44200 50300 44200 50950 4
N 53200 49100 53200 49700 4
N 52600 47800 52600 46600 4
N 52400 48300 50400 48300 4
N 52100 54500 50300 54500 4
N 44200 51850 44200 52000 4
N 47100 50000 47350 50000 4
N 47350 50000 47350 52000 4
N 47100 50300 47800 50300 4
N 47800 51850 47800 52000 4
C 51000 46500 1 0 0 resistor-1.sym
{
T 51300 46900 5 10 0 0 0 0 1
device=R_3.30k_1%_1608
T 51650 46450 5 10 1 1 180 0 1
refdes=R10
T 51650 46850 5 10 1 1 180 0 1
value=3.30k
T 51000 46500 5 10 0 0 270 0 1
description=Res thick 3.30k 1%   
T 51000 46500 5 10 0 0 270 0 1
footprint=RESC1608N.lht
}
C 44300 50950 1 90 0 resistor-1.sym
{
T 43900 51250 5 10 0 0 90 0 1
device=R_4.7R_1%_1608
T 44000 51150 5 10 1 1 90 0 1
refdes=R15
T 44500 51350 5 10 1 1 90 0 1
value=4.70K
T 44300 50950 5 10 0 0 0 0 1
description=Res thick 4.70k 1% 1608
T 44300 50950 5 10 0 0 0 0 1
footprint=RESC1608N.lht
}
N 47100 50600 48400 50600 4
{
T 47200 50650 5 10 1 1 0 0 1
netname=PUSH_DRIVE
}
N 47800 48900 47800 50950 4
T 51900 48500 9 12 1 0 0 6 2
alt. transistors
use only one
N 49500 49900 51600 49900 4
N 52500 49900 53500 49900 4
N 52900 54500 53500 54500 4
N 51200 52900 49500 52900 4
N 52100 52900 53500 52900 4
N 52300 55000 52300 56100 4
N 53200 53700 53200 52900 4
T 56900 55600 9 12 1 0 0 0 7
layout two vias to gnd 
near TXFMR pads
(heatsink), and
put Schottkys in close,
and caps in close,

and transistors!
L 55100 53000 56600 55600 3 10 1 0 -1 -1
B 56600 54600 2600 2800 3 10 1 0 -1 -1 0 -1 -1 -1 -1 -1
C 51600 50100 1 180 1 schottky.sym
{
T 52000 49500 5 10 0 0 0 2 1
device=B5819WS
T 52600 50000 5 10 1 1 0 6 1
refdes=D6
T 52400 50200 5 10 1 1 0 6 1
value=.9V .5W
T 51800 50150 5 10 0 1 270 6 1
footprint=SOD-323N.lht
}
N 51300 56100 52400 56100 4
N 53300 56100 54000 56100 4
{
T 53300 56150 5 10 1 1 0 0 1
netname=PULL_DRIVE
}
N 50400 56100 50300 56100 4
C 53300 56000 1 0 1 resistor-1.sym
{
T 53000 56400 5 10 0 0 180 2 1
device=Res_3.30k_1%_1608
T 52750 55950 5 10 1 1 180 6 1
refdes=R13
T 53300 56000 5 10 0 1 0 2 1
footprint=RESC1608N.lht
T 52550 56350 5 10 1 1 180 6 1
value=3.30k
T 53300 56000 5 10 0 0 0 2 1
description=Res thick 3.30k Ohm 1%
}
N 49500 51400 54000 51400 4
{
T 52250 51600 5 11 1 1 180 6 1
netname=18V_ONBOARD
}
C 57000 51900 1 0 0 diode-1.sym
{
T 57400 52500 5 10 0 0 0 0 1
device=M20F
T 57600 52500 5 10 1 1 180 0 1
refdes=D3
T 57700 51800 5 10 1 1 180 0 1
value=2KV
T 57000 51900 5 10 0 0 0 0 1
description=diode rect 2000V 1A SMAF
T 57000 51900 5 10 0 0 0 0 1
footprint=DIOM_SMAFN.lht
}
C 56800 49400 1 90 0 diode-1.sym
{
T 56200 49800 5 10 0 0 90 0 1
device=M20F
T 56200 50100 5 10 1 1 0 0 1
refdes=D7
T 56900 49700 5 10 1 1 0 0 1
value=2KV
T 56800 49400 5 10 0 0 90 0 1
description=diode rect 2000V 1A SMAF
T 56800 49400 5 10 0 0 90 0 1
footprint=DIOM_SMAFN.lht
}
N 56600 51200 56600 52100 4
N 59200 47400 59200 46400 4
C 60100 45200 1 90 0 capacitor-variable-1.sym
{
T 59400 45400 5 10 0 0 90 0 1
device=TZC3Z300A110
T 59800 45900 5 10 1 1 180 0 1
refdes=C15
T 59200 45400 5 10 0 0 90 0 1
symversion=0.1
T 60000 45300 5 10 1 1 0 0 1
value=30pF
T 60100 45200 5 10 0 0 0 0 1
footprint=CAP_VAR3040N.lht
}
C 61100 41600 1 90 0 resistor-1.sym
{
T 60700 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 60900 41500 5 10 1 1 90 0 1
refdes=R25
T 61200 42400 5 10 1 1 90 0 1
value=22
T 61100 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 61100 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
N 61000 41600 61000 40700 4
N 62500 40700 61000 40700 4
N 62500 41600 62500 40700 4
N 62200 41600 62200 40700 4
N 62500 42500 62500 43200 4
N 61600 42500 61600 43200 4
N 62200 42500 62200 43200 4
N 61900 42500 61900 43200 4
N 61900 41600 61900 40700 4
N 61600 41600 61600 40700 4
N 61300 42500 61300 43200 4
N 61300 41600 61300 40700 4
N 61000 42500 61000 43200 4
N 59900 46400 59900 46100 4
N 59900 45200 59900 44700 4
T 55100 40200 9 12 1 0 0 0 4
Current sense R's
paralleled so they 
will not likely go 
open circuit.
N 55200 41400 58200 41400 4
{
T 55400 41400 5 10 1 1 0 0 1
netname=PA5_ADC1.5_current_sense
}
N 59100 41400 60100 41400 4
T 61200 45900 9 12 1 0 0 0 2
probe comp.
capacitance
N 62500 51400 62500 50600 4
L 58500 47800 58500 48600 3 10 1 0 -1 -1
L 58500 46700 58500 47500 3 10 1 0 -1 -1
L 64300 46700 64300 47500 3 10 1 0 -1 -1
L 64300 47800 64300 48600 3 10 1 0 -1 -1
N 60100 47400 60400 47400 4
N 61300 47400 61600 47400 4
N 62500 47400 62500 48200 4
N 61600 48200 61600 49000 4
N 61600 50600 61600 49800 4
N 62500 49800 62500 49000 4
N 60800 45200 60800 44700 4
C 61000 45200 1 90 0 capacitor-variable-1.sym
{
T 60700 45900 5 10 1 1 180 0 1
refdes=C16
T 60900 45300 5 10 1 1 0 0 1
value=6pF
T 60300 45400 5 10 0 0 90 0 1
device=TZC3Z060A110
T 60100 45400 5 10 0 0 90 0 1
symversion=0.1
T 61000 45200 5 10 0 0 0 0 1
footprint=CAP_VAR3040N.lht
}
N 60800 46400 60800 46100 4
N 51700 56100 51700 57600 4
C 52100 45800 1 0 1 resistor-1.sym
{
T 51800 46200 5 10 0 0 180 2 1
device=R_22.0_1%_1608
T 51700 45600 5 10 1 1 0 6 1
refdes=R17
T 51800 46100 5 10 1 1 0 6 1
value=22
T 52100 45800 5 10 0 0 180 2 1
footprint=RESC1608N.lht
T 52100 45800 5 10 0 0 180 2 1
description=Res thick 22.0 1% 1608
}
N 52100 46600 52100 45900 4
T 44100 44400 9 14 1 0 0 0 1
Lockout when cover open
N 45700 46000 45700 47600 4
T 43600 44100 9 14 1 0 0 0 1
wires to dual normally open limit switch
C 52100 55000 1 270 0 NFET_enh.sym
{
T 52800 54900 5 10 1 1 0 0 1
device=NTR5198NL
T 52500 55200 5 10 1 1 0 0 1
refdes=Q1
T 52100 55000 5 10 0 0 270 0 1
footprint=SOT23-3N.lht
T 52100 55000 5 10 0 0 270 0 1
description=MOSFET N-CH 60V 1.7A SOT23
T 52100 55000 5 10 0 0 0 0 1
value=semicon
}
C 52400 47800 1 270 1 NFET_enh.sym
{
T 53900 47600 5 10 1 1 0 6 1
device=NTR5198NL
T 53200 47800 5 10 1 1 0 6 1
refdes=Q2
T 52400 47800 5 10 0 0 270 6 1
footprint=SOT23-3N.lht
T 52400 47800 5 10 0 0 270 6 1
description=MOSFET N-CH 60V 1.7A SOT23
T 52400 47800 5 10 0 0 0 0 1
value=semicon
}
C 61400 41600 1 90 0 resistor-1.sym
{
T 61000 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 61200 41500 5 10 1 1 90 0 1
refdes=R24
T 61500 42400 5 10 1 1 90 0 1
value=22
T 61400 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 61400 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
C 61700 41600 1 90 0 resistor-1.sym
{
T 61300 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 61500 41500 5 10 1 1 90 0 1
refdes=R23
T 61800 42400 5 10 1 1 90 0 1
value=22
T 61700 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 61700 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
C 62000 41600 1 90 0 resistor-1.sym
{
T 61600 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 61800 41500 5 10 1 1 90 0 1
refdes=R22
T 62100 42400 5 10 1 1 90 0 1
value=22
T 62000 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 62000 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
C 62300 41600 1 90 0 resistor-1.sym
{
T 61900 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 62100 41500 5 10 1 1 90 0 1
refdes=R21
T 62400 42400 5 10 1 1 90 0 1
value=22
T 62300 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 62300 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
C 62600 41600 1 90 0 resistor-1.sym
{
T 62200 41900 5 10 0 0 90 0 1
device=R_22.0_1%_1608
T 62400 41500 5 10 1 1 90 0 1
refdes=R20
T 62700 42400 5 10 1 1 90 0 1
value=22
T 62600 41600 5 10 0 0 90 0 1
footprint=RESC1608N.lht
T 62600 41600 5 10 0 0 90 0 1
description=Res thick 22.0 1% 1608
}
C 50800 57700 1 180 1 resistor-1.sym
{
T 51100 57300 5 10 0 0 0 2 1
device=R_22.0_1%_1608
T 51300 57500 5 10 1 1 180 6 1
refdes=R18
T 51200 57900 5 10 1 1 180 6 1
value=22
T 50800 57700 5 10 0 0 0 2 1
footprint=RESC1608N.lht
T 50800 57700 5 10 0 0 0 2 1
description=Res thick 22.0 1% 1608
}
C 46300 44700 1 90 0 ffc-zif-6.sym
{
T 46500 45200 5 10 1 1 180 0 1
refdes=J4
T 46300 44700 5 10 0 0 0 0 1
footprint=ffc_zif_20_1.25.lht
T 46300 44700 5 10 0 0 0 0 1
device=FC5030-06
T 46300 44700 5 10 0 0 0 0 1
value=conn
}
N 45400 46800 45400 46000 4
{
T 45350 46800 5 10 1 1 90 6 1
netname=Q1_OFF
}
C 47900 50950 1 90 0 resistor-1.sym
{
T 47500 51250 5 10 0 0 90 0 1
device=R_4.7R_1%_1608
T 47600 51150 5 10 1 1 90 0 1
refdes=R16
T 48100 51350 5 10 1 1 90 0 1
value=4.70K
T 47900 50950 5 10 0 0 0 0 1
description=Res thick 4.70k 1% 1608
T 47900 50950 5 10 0 0 0 0 1
footprint=RESC1608N.lht
}
C 59200 44400 1 0 0 vss.sym
C 50000 53500 1 270 0 vss.sym
C 58200 45200 1 90 0 resistor-1.sym
{
T 57800 45500 5 10 0 0 90 0 1
device=R_9.10k_1%_1608
T 57900 45500 5 10 1 1 90 0 1
refdes=R30
T 58400 45600 5 10 1 1 90 0 1
value=9.10K
T 58200 45200 5 10 0 0 0 0 1
description=9.10k  ±1% 1608
T 58200 45200 5 10 0 0 0 0 1
footprint=RESC1608N.lht
}
C 59000 45200 1 90 0 resistor-1.sym
{
T 58600 45500 5 10 0 0 90 0 1
device=R_9.10k_1%_1608
T 58700 45400 5 10 1 1 90 0 1
refdes=R31
T 59200 45600 5 10 1 1 90 0 1
value=9.10K
T 59000 45200 5 10 0 0 0 0 1
description=9.10k  ±1% 1608
T 59000 45200 5 10 0 0 0 0 1
footprint=RESC1608N.lht
}
N 58100 45200 58100 44700 4
N 58100 46100 58100 46400 4
N 58900 46100 58900 46400 4
C 51200 52700 1 0 0 schottky.sym
{
T 51600 53300 5 10 0 0 0 0 1
device=B5819WS
T 51900 52600 5 10 1 1 0 0 1
refdes=D5
T 51900 52400 5 10 1 1 0 0 1
value=.9V .5W
T 51400 52650 5 10 0 1 270 0 1
footprint=SOD-323N.lht
}
L 64300 46700 63800 46700 3 10 1 0 -1 -1
L 63700 54000 64300 54000 3 10 1 0 -1 -1
T 63950 52150 9 12 1 0 90 0 4
spring contact T1 
has cutout and 
R locations in 
footprint.
C 64250 44300 1 0 1 cuvette_cont.sym
{
T 63940 45050 5 10 0 0 0 6 1
device=36-110
T 63940 44900 5 10 0 0 0 6 1
footprint=cuvette_cont_notch.lht
T 64000 44350 5 10 1 1 0 0 1
refdes=J2
T 64250 44300 5 10 0 0 0 0 1
value=conn
}
C 64050 51900 1 180 0 cuvette_cont.sym
{
T 63740 51150 5 10 0 0 180 0 1
device=36-110
T 63740 51300 5 10 0 0 180 0 1
footprint=cuvette_cont_resistors.lht
T 63800 51850 5 10 1 1 180 6 1
refdes=J3
T 64050 51900 5 10 0 0 0 0 1
value=conn
}
C 51500 54200 1 270 0 NFET_enh_TSOT-26.sym
{
T 51200 53300 5 10 1 1 0 0 1
device=DMN4060SVT
T 52350 53900 5 10 1 1 0 0 1
refdes=Q11
T 51500 54200 5 10 0 0 270 0 1
description=MOSFET N-CH 45V 4.8A TSOT-26
T 51500 54200 5 10 0 0 270 0 1
footprint=TSOT-26N.lht
T 51500 54200 5 10 0 0 0 0 1
value=semicon
}
N 53200 53500 52700 53500 4
N 53200 53300 52700 53300 4
C 51800 48600 1 270 1 NFET_enh_TSOT-26.sym
{
T 52600 49300 5 10 1 1 0 6 1
device=DMN4060SVT
T 53050 48800 5 10 1 1 0 6 1
refdes=Q12
T 51800 48600 5 10 0 0 270 6 1
description=MOSFET N-CH 45V 4.8A TSOT-26
T 51800 48600 5 10 0 0 270 6 1
footprint=TSOT-26N.lht
T 51800 48600 5 10 0 0 0 0 1
value=semicon
}
N 53000 49100 53200 49100 4
N 53000 49500 53200 49500 4
N 53000 49700 53500 49700 4
N 53000 49300 53200 49300 4
N 53200 48300 53500 48300 4
N 53500 50700 54000 50700 4
N 52700 53100 53200 53100 4
C 58300 52900 1 0 0 diode-1.sym
{
T 58700 53500 5 10 0 0 0 0 1
device=S1Y
T 58600 53500 5 10 1 1 0 0 1
refdes=D2
T 58600 52700 5 10 1 1 0 0 1
value=2KV
T 58300 52900 5 10 0 0 0 0 1
description=diode rect 2000V 1A DO-15
T 58300 52900 5 10 0 0 0 0 1
footprint=diode_10mm.fp
}
C 57400 52900 1 0 0 diode-1.sym
{
T 57800 53500 5 10 0 0 0 0 1
device=S1Y
T 57700 53500 5 10 1 1 0 0 1
refdes=D1
T 57700 52700 5 10 1 1 0 0 1
value=2KV
T 57400 52900 5 10 0 0 0 0 1
description=diode rect 2000V 1A DO-15
T 57400 52900 5 10 0 0 0 0 1
footprint=diode_10mm.fp
}
N 57400 53100 57000 53100 4
N 57000 53100 57000 52100 4
N 59200 53100 59200 52100 4
N 57500 51300 57500 51200 4
N 57500 51300 56600 51300 4
N 57500 49400 57500 48900 4
C 57700 49400 1 90 0 diode-1.sym
{
T 57100 49800 5 10 0 0 90 0 1
device=S1Y
T 57100 50100 5 10 1 1 0 0 1
refdes=D8
T 57800 49700 5 10 1 1 0 0 1
value=2KV
T 57700 49400 5 10 0 0 90 0 1
description=diode rect 2000V 1A DO-15
T 57700 49400 5 10 0 0 90 0 1
footprint=diode_10mm.fp
}
N 44200 52000 47800 52000 4
{
T 45250 52200 5 11 1 1 180 6 1
netname=18V_ONBOARD
}
C 50100 51100 1 270 0 capacitor-1.sym
{
T 50800 50900 5 10 0 0 270 0 1
device=CL31A106MBHNNNE
T 51000 50900 5 10 0 0 270 0 1
symversion=0.1
T 50100 51100 5 10 0 0 0 0 1
description=CAP CER 10UF 50V X5R 1206 
T 50100 51100 5 10 0 0 0 0 1
footprint=CAPC3216N.lht
T 50100 50500 5 10 1 1 270 0 1
refdes=C12
T 50500 50800 5 10 1 1 90 0 1
value=10 uF
T 49900 51000 5 10 0 1 0 0 1
comment=X5R
}
C 49300 50800 1 270 0 capacitor-2.sym
{
T 50000 50600 5 10 0 0 270 0 1
device=UUX1E221MNL1GS
T 50200 50600 5 10 0 0 270 0 1
symversion=0.1
T 49300 50800 5 10 0 0 0 0 1
description=CAP ALUM 220UF 20% 25V SMD
T 49300 50800 5 10 0 0 0 0 1
footprint=CAPAE10X10.lht
T 49200 50000 5 10 1 1 0 0 1
refdes=C7
T 49400 50500 5 10 1 1 90 0 1
value=220 uF
}
C 49700 52000 1 90 0 capacitor-2.sym
{
T 49000 52200 5 10 0 0 90 0 1
device=UUX1E221MNL1GS
T 48800 52200 5 10 0 0 90 0 1
symversion=0.1
T 49700 52000 5 10 0 0 180 0 1
description=CAP ALUM 220UF 20% 25V SMD
T 49700 52000 5 10 0 0 180 0 1
footprint=CAPAE10X10.lht
T 49800 52800 5 10 1 1 180 0 1
refdes=C8
T 49600 52300 5 10 1 1 270 0 1
value=220 uF
}
C 50700 52700 1 270 0 capacitor-1.sym
{
T 51400 52500 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 51600 52500 5 10 0 0 270 0 1
symversion=0.1
T 50700 52700 5 10 0 0 0 0 1
description=CAP CER 1UF 50V X7R
T 50700 52700 5 10 0 0 0 0 1
footprint=CAPC2013N.lht
T 50700 52100 5 10 1 1 270 0 1
refdes=C10
T 51100 52400 5 10 1 1 90 0 1
value=1 uF
T 50500 52600 5 10 0 1 0 0 1
comment=X5R
}
C 50000 52700 1 270 0 capacitor-1.sym
{
T 50700 52500 5 10 0 0 270 0 1
device=CL31A106MBHNNNE
T 50900 52500 5 10 0 0 270 0 1
symversion=0.1
T 50000 52700 5 10 0 0 0 0 1
description=CAP CER 10UF 50V X5R 1206 
T 50000 52700 5 10 0 0 0 0 1
footprint=CAPC3216N.lht
T 50000 52100 5 10 1 1 270 0 1
refdes=C11
T 50400 52400 5 10 1 1 90 0 1
value=10 uF
T 49800 52600 5 10 0 1 0 0 1
comment=X5R
}
N 44800 50600 44800 52000 4
L 55200 50400 55800 50400 3 10 1 0 -1 -1
L 55800 50000 55800 50400 3 10 1 0 -1 -1
N 56000 50700 55500 50700 4
L 55000 52800 55200 52900 3 10 1 0 -1 -1
L 55000 53100 55200 52900 3 10 1 0 -1 -1
L 55000 53100 55000 52800 3 10 1 0 -1 -1
N 50800 57600 50000 57600 4
{
T 50700 57650 5 10 1 1 0 6 1
netname=Q1_OFF
}
C 45800 47300 1 0 0 vss.sym
N 45700 47600 45900 47600 4
N 50400 45900 51200 45900 4
{
T 50500 45950 5 10 1 1 180 8 1
netname=Q2_OFF
}
C 50100 49300 1 270 1 vss.sym
N 50300 56100 50300 52900 4
N 46000 46800 46000 46000 4
{
T 45950 46800 5 10 1 1 90 6 1
netname=Q2_OFF
}
N 53500 48300 53500 50700 4
{
T 53550 49200 5 10 1 1 270 0 1
netname=PUSH
}
N 43600 50000 44800 50000 4
{
T 43700 50050 5 10 1 1 0 0 1
netname=PULL_DRIVE
}
N 54900 46600 53600 46600 4
{
T 54800 46650 5 10 1 1 0 6 1
netname=PUSH_DRIVE
}
N 53500 54500 53500 52100 4
{
T 53550 54000 5 10 1 1 270 0 1
netname=PULL
}
N 50900 51400 50900 51800 4
C 52700 46500 1 0 0 resistor-1.sym
{
T 53000 46900 5 10 0 0 0 0 1
device=R_3.30k_1%_1608
T 53350 46450 5 10 1 1 180 0 1
refdes=R12
T 53350 46850 5 10 1 1 180 0 1
value=3.30k
T 52700 46500 5 10 0 0 270 0 1
description=Res thick 3.30k 1%   
T 52700 46500 5 10 0 0 270 0 1
footprint=RESC1608N.lht
}
T 53000 47000 9 12 1 0 0 0 1
3.40k subs
T 51200 47000 9 12 1 0 0 0 1
3.40k subs
T 52200 56500 9 12 1 0 0 0 1
3.40k subs
C 51300 56000 1 0 1 resistor-1.sym
{
T 51000 56400 5 10 0 0 180 2 1
device=Res_3.30k_1%_1608
T 50750 55950 5 10 1 1 180 6 1
refdes=R11
T 51300 56000 5 10 0 1 0 2 1
footprint=RESC1608N.lht
T 50550 56350 5 10 1 1 180 6 1
value=3.30k
T 51300 56000 5 10 0 0 0 2 1
description=Res thick 3.30k Ohm 1%
}
T 50100 56400 9 12 1 0 0 0 1
3.40k subs
C 50900 45900 1 270 0 capacitor-1.sym
{
T 51600 45700 5 10 0 0 270 0 1
device=06033C104KAT2A
T 51800 45700 5 10 0 0 270 0 1
symversion=0.1
T 50600 45500 5 10 1 1 0 0 1
refdes=C18
T 50500 45200 5 10 1 1 0 0 1
value=0.1 uF 25V
T 50900 45900 5 10 0 0 0 0 1
footprint=CAPC1608N.lht
}
C 51200 44700 1 0 1 vss.sym
C 50800 56900 1 0 0 capacitor-1.sym
{
T 51000 57600 5 10 0 0 0 0 1
device=06033C104KAT2A
T 51000 57800 5 10 0 0 0 0 1
symversion=0.1
T 51100 57000 5 10 1 1 180 0 1
refdes=C19
T 51100 57300 5 10 1 1 180 0 1
value=0.1 uF 25V
T 50800 56900 5 10 0 0 90 0 1
footprint=CAPC1608N.lht
}
C 50500 56800 1 0 1 vss.sym
N 50800 57100 50400 57100 4
C 44800 49400 1 0 0 PFET_enh_dual.sym
{
T 45300 51250 5 14 1 1 0 3 1
refdes=U1
T 45300 49500 5 10 1 1 0 0 1
footprint=SOT563-6-M.lht
T 45600 51300 5 10 1 1 0 0 1
device=NTZD3152PT1G
T 44800 49400 5 10 0 0 0 0 1
value=semicon
}
C 57000 48600 1 0 0 vss.sym
C 58200 41300 1 0 0 EMBEDDEDresistor-1.sym
[
L 58800 41500 58700 41300 3 0 0 0 -1 -1
L 58700 41300 58600 41500 3 0 0 0 -1 -1
L 58600 41500 58500 41300 3 0 0 0 -1 -1
L 58500 41300 58400 41500 3 0 0 0 -1 -1
L 58800 41500 58900 41300 3 0 0 0 -1 -1
L 58900 41300 58950 41400 3 0 0 0 -1 -1
P 59100 41400 58950 41400 1 0 0
{
T 59000 41450 5 8 0 1 0 0 1
pinnumber=2
T 59000 41450 5 8 0 0 0 0 1
pinseq=2
T 59000 41450 5 8 0 1 0 0 1
pinlabel=2
T 59000 41450 5 8 0 1 0 0 1
pintype=pas
}
P 58200 41400 58352 41400 1 0 0
{
T 58300 41450 5 8 0 1 0 0 1
pinnumber=1
T 58300 41450 5 8 0 0 0 0 1
pinseq=1
T 58300 41450 5 8 0 1 0 0 1
pinlabel=1
T 58300 41450 5 8 0 1 0 0 1
pintype=pas
}
L 58401 41500 58350 41400 3 0 0 0 -1 -1
T 58500 41700 5 10 0 0 0 0 1
device=RESISTOR
T 58400 41600 8 10 0 1 0 0 1
refdes=R?
T 58200 41300 8 10 0 1 0 0 1
pins=2
T 58200 41300 8 10 0 1 0 0 1
class=DISCRETE
]
{
T 58500 41700 5 10 0 0 0 0 1
device=R_133_1%_2013
T 58200 41300 5 10 0 0 180 0 1
footprint=RESC2013N.lht
T 58900 41200 5 10 1 1 180 0 1
refdes=R26
T 58600 41700 5 10 1 1 180 0 1
value=133
}
N 50200 52700 50200 52900 4
N 50200 51400 50200 51800 4
N 50300 50200 50300 49900 4
N 49500 52000 49500 50800 4
N 50300 51100 50300 51400 4
T 51700 54000 9 12 1 0 0 6 2
alt. transistors
use only one
N 50900 52700 50900 52900 4
C 50800 51100 1 270 0 capacitor-1.sym
{
T 51500 50900 5 10 0 0 270 0 1
device=CL21B105KBFNFNE
T 51700 50900 5 10 0 0 270 0 1
symversion=0.1
T 50800 51100 5 10 0 0 0 0 1
description=CAP CER 1UF 50V X7R
T 50800 51100 5 10 0 0 0 0 1
footprint=CAPC2013N.lht
T 50800 50500 5 10 1 1 270 0 1
refdes=C6
T 51200 50800 5 10 1 1 90 0 1
value=1 uF
T 50600 51000 5 10 0 1 0 0 1
comment=X5R
}
N 51000 51100 51000 51400 4
N 51000 50200 51000 49900 4
N 60100 43200 63100 43200 4
{
T 61800 43250 5 10 1 1 0 0 1
netname=LOW_SIDE
}
C 61700 40400 1 0 0 vss.sym
N 60100 43200 60100 41400 4
N 57500 46400 60800 46400 4
N 63100 43200 63100 44400 4
N 63100 44400 63300 44400 4
L 58500 49100 58500 49500 3 10 1 0 -1 -1
C 56600 46300 1 0 0 EMBEDDEDresistor-1.sym
[
L 57200 46500 57100 46300 3 0 0 0 -1 -1
L 57100 46300 57000 46500 3 0 0 0 -1 -1
L 57000 46500 56900 46300 3 0 0 0 -1 -1
L 56900 46300 56800 46500 3 0 0 0 -1 -1
L 57200 46500 57300 46300 3 0 0 0 -1 -1
L 57300 46300 57350 46400 3 0 0 0 -1 -1
P 57500 46400 57350 46400 1 0 0
{
T 57400 46450 5 8 0 1 0 0 1
pinnumber=2
T 57400 46450 5 8 0 0 0 0 1
pinseq=2
T 57400 46450 5 8 0 1 0 0 1
pinlabel=2
T 57400 46450 5 8 0 1 0 0 1
pintype=pas
}
P 56600 46400 56752 46400 1 0 0
{
T 56700 46450 5 8 0 1 0 0 1
pinnumber=1
T 56700 46450 5 8 0 0 0 0 1
pinseq=1
T 56700 46450 5 8 0 1 0 0 1
pinlabel=1
T 56700 46450 5 8 0 1 0 0 1
pintype=pas
}
L 56801 46500 56750 46400 3 0 0 0 -1 -1
T 56900 46700 5 10 0 0 0 0 1
device=RESISTOR
T 56800 46600 8 10 0 1 0 0 1
refdes=R?
T 56600 46300 8 10 0 1 0 0 1
pins=2
T 56600 46300 8 10 0 1 0 0 1
class=DISCRETE
]
{
T 56900 46700 5 10 0 0 0 0 1
device=R_133_1%_2013
T 56600 46300 5 10 0 0 180 0 1
footprint=RESC2013N.lht
T 57300 46200 5 10 1 1 180 0 1
refdes=R27
T 57000 46700 5 10 1 1 180 0 1
value=133
}
N 56600 46400 56400 46400 4
N 56400 46400 56400 45600 4
N 43300 50300 44800 50300 4
{
T 43400 50450 5 10 1 1 180 6 1
netname=PA1_TIM2_CH2
}
N 44700 48900 47800 48900 4
{
T 45000 49050 5 10 1 1 180 6 1
netname=PA0_TIM2_CH1
}
C 57700 50300 1 90 0 diode-1.sym
{
T 57100 50700 5 10 0 0 90 0 1
device=S1Y
T 57100 51000 5 10 1 1 0 0 1
refdes=D10
T 57800 50600 5 10 1 1 0 0 1
value=2KV
T 57700 50300 5 10 0 0 90 0 1
description=diode rect 2000V 1A DO-15
T 57700 50300 5 10 0 0 90 0 1
footprint=diode_10mm.fp
}
C 56800 50300 1 90 0 diode-1.sym
{
T 56200 50700 5 10 0 0 90 0 1
device=M20F
T 56200 51000 5 10 1 1 0 0 1
refdes=D9
T 56900 50600 5 10 1 1 0 0 1
value=2KV
T 56800 50300 5 10 0 0 90 0 1
description=diode rect 2000V 1A SMAF
T 56800 50300 5 10 0 0 90 0 1
footprint=DIOM_SMAFN.lht
}
C 43700 45700 1 0 1 vss.sym
N 45100 46000 43600 46000 4
C 39100 57200 1 180 1 ffc-zif-20.sym
{
T 40607 52754 5 10 0 0 180 6 1
symversion=0.2
T 40542 52608 5 10 0 1 180 6 1
device=FC5030-20
T 39100 57200 5 10 0 0 0 0 1
value=conn
T 39100 57200 5 10 0 0 0 0 1
footprint=ffc_zif_20_1.25.lht
T 39200 50200 5 10 1 1 180 6 1
refdes=J1
}
N 40400 53100 41400 53100 4
{
T 41100 53245 5 10 1 1 180 3 1
netname=VSS
}
N 40400 54300 41400 54300 4
{
T 41100 54445 5 10 1 1 180 3 1
netname=VSS
}
N 40400 55800 42600 55800 4
{
T 40950 56000 5 11 1 1 180 6 1
netname=18V_ONBOARD
}
N 40400 54900 41400 54900 4
{
T 41100 55045 5 10 1 1 180 3 1
netname=VSS
}
N 40400 51300 43000 51300 4
{
T 42800 51500 5 10 1 1 180 0 1
netname=PA5_ADC1.5_current_sense
}
N 40400 56100 41700 56100 4
{
T 40800 56250 5 10 1 1 180 6 1
netname=HV_sense
}
N 40400 52200 41400 52200 4
{
T 41100 52345 5 10 1 1 180 3 1
netname=VSS
}
N 40400 51000 40800 51000 4
N 40400 54600 40800 54600 4
N 40400 51600 41400 51600 4
{
T 41100 51745 5 10 1 1 180 3 1
netname=VSS
}
N 40400 51900 42100 51900 4
{
T 40700 52050 5 10 1 1 180 6 1
netname=PA0_TIM2_CH1
}
N 40400 52800 42100 52800 4
{
T 40700 52950 5 10 1 1 180 6 1
netname=PA1_TIM2_CH2
}
C 40800 50900 1 0 0 nc-right-1.sym
{
T 40900 51400 5 10 0 0 0 0 1
value=NoConnection
T 40900 51600 5 10 0 0 0 0 1
device=DRC_Directive
}
C 40800 54500 1 0 0 nc-right-1.sym
{
T 40900 55000 5 10 0 0 0 0 1
value=NoConnection
T 40900 55200 5 10 0 0 0 0 1
device=DRC_Directive
}
N 40400 50700 41400 50700 4
{
T 41100 50845 5 10 1 1 180 3 1
netname=VSS
}
N 40400 56400 41400 56400 4
{
T 41100 56545 5 10 1 1 180 3 1
netname=VSS
}
N 40400 55200 41400 55200 4
{
T 41100 55345 5 10 1 1 180 3 1
netname=VSS
}
N 40400 53400 41400 53400 4
{
T 41100 53545 5 10 1 1 180 3 1
netname=VSS
}
N 40400 52500 41400 52500 4
{
T 41100 52645 5 10 1 1 180 3 1
netname=VSS
}
N 40400 55500 42600 55500 4
{
T 40950 55700 5 11 1 1 180 6 1
netname=18V_ONBOARD
}
N 40100 57000 40500 57000 4
N 40500 57000 40500 56400 4
N 40500 50700 40500 50100 4
N 40500 50100 40100 50100 4
C 40800 53900 1 0 0 nc-right-1.sym
{
T 40900 54400 5 10 0 0 0 0 1
value=NoConnection
T 40900 54600 5 10 0 0 0 0 1
device=DRC_Directive
}
N 40400 54000 40800 54000 4
C 40800 53600 1 0 0 nc-right-1.sym
{
T 40900 54100 5 10 0 0 0 0 1
value=NoConnection
T 40900 54300 5 10 0 0 0 0 1
device=DRC_Directive
}
N 40400 53700 40800 53700 4
