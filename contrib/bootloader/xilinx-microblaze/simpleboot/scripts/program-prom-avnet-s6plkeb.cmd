setMode -bscan
setCable -p auto
Identify -inferir
identifyMPM
attachflash -position 1 -spi "S25FL064P"
assignfiletoattachedflash -position 1 -file flash_image.mcs
Program -p 1 -dataWidth 1 -spionly -e -v -loadfpga
quit
