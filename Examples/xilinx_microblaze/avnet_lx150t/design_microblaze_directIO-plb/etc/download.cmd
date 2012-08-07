setMode -bs
setCable -port auto
Identify -inferir
identifyMPM 
assignFile -p 3 -file implementation/download.bit
Program -p 3
exit
