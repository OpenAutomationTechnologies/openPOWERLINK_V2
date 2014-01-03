setMode -bs
setCable -port auto
Identify -inferir
identifyMPM
assignFile -p 1 -file implementation/download.bit
Program -p 1
exit
