clearlibrary -simdata
set path "C:/WORK/repo/VHDL_IP-Cores"
acom $path/common/lib/src/global.vhd
acom $path/common/lib/src/sync.vhd
acom $path/common/lib/src/edgedet.vhd
acom $path/common/util/src/clkGenBhv.vhd
acom $path/common/util/src/resetGenBhv.vhd
acom $path/common/hostinterface/src/hostInterfacePkg.vhd
acom $path/common/hostinterface/src/parallelInterfaceRtl.vhd
acom $path/common/hostinterface/tb/tbParallelInterfaceBhv.vhd
asim tbParallelInterface