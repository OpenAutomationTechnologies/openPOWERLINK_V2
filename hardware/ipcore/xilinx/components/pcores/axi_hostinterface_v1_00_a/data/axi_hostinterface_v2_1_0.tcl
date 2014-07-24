#------------------------------------------------------------------------------
#-- HOST INTERFACE XPS AXI Component (TCL)
#------------------------------------------------------------------------------
#
#   (c) B&R, 2014
#   (c) Kalycito Infotech Private Limited, 2014
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#   3. Neither the name of B&R nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without prior written permission. For written
#      permission, please contact office@br-automation.com
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
# -----------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-- Version History
#-------------------------------------------------------------------------------
# -- 2014-01-13    V0.01    Vinod PA            initial draft
#-------------------------------------------------------------------------------
#uses "xillib.tcl"

proc calc_baseadr_dbuf1 { param_handle} {
     set mhsinst     [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseDynBuf0"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf0"]
     set Incr_Addr_Size [format 0x%x [expr $User_Size * 1024]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_Errcntr { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseDynBuf1"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf1"]
     set Incr_Addr_Size [format 0x%x [expr $User_Size * 1024]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_TxNmtQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseErrCntr"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_B_ErrorCounter"]
     set Incr_Addr_Size [format 0x%x $User_Size]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_TxGenQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseTxNmtQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxNmtQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_TxSynQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseTxGenQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxGenQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_TxVetQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseTxSynQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxSynQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_RxVetQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseTxVetQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxVetQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_K2UQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseRxVetQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_RxVetQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_U2KQ { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseK2UQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_K2UQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_Pdo { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBaseU2KQ"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_KB_U2KQ"]
     set Incr_Addr_Size [format 0x%x [expr {$User_Size * 1024} + 16]]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_baseadr_Res { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Base_Addr   [xget_hw_parameter_value $mhsinst "gBasePdo"]
     set User_Size   [xget_hw_parameter_value $mhsinst "Size_B_Pdo"]
     set Incr_Addr_Size [format 0x%x $User_Size ]
     set Updated_Addr [format 0x%x [expr $Base_Addr + $Incr_Addr_Size]]
     return $Updated_Addr
}

proc calc_size_dbuf0 {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf0"]
     set Updated_Size [expr $Size * 1024]
     return $Updated_Size
}

proc calc_size_dbuf1 {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf1"]
     set Updated_Size [expr $Size * 1024]
     return $Updated_Size
}

proc calc_size_Errcntr {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_B_ErrorCounter"]
     set Updated_Size [expr $Size * 1]
     return $Updated_Size
}

proc calc_size_TxNmtQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxNmtQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}


proc calc_size_TxGenQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxGenQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_TxSynQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxSynQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_TxVetQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_TxVetQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_RxVetQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_RxVetQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_K2UQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_K2UQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_U2KQ {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_KB_U2KQ"]
     set Updated_Size [expr {$Size * 1024} + 16]
     return $Updated_Size
}

proc calc_size_Pdo {param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set Size   [xget_hw_parameter_value $mhsinst "Size_B_Pdo"]
     set Updated_Size [expr $Size * 1]
     return $Updated_Size
}




proc calc_total_memory { param_handle} {
     set mhsinst      [xget_hw_parent_handle $param_handle]
     set listGuiParam [list "Size_KB_DynBuf0" "Size_KB_DynBuf1" "Size_B_ErrorCounter" "Size_KB_TxNmtQ" "Size_KB_TxGenQ" "Size_KB_TxSynQ" "Size_KB_TxVetQ" "Size_KB_RxVetQ" "Size_KB_K2UQ" "Size_KB_U2KQ" "Size_B_Pdo"]
     set DynBuf0_Size  [expr [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf0"] * 1024]
     set DynBuf1_Size  [expr [xget_hw_parameter_value $mhsinst "Size_KB_DynBuf1"] *1024]
     set Errcntr_Size  [xget_hw_parameter_value $mhsinst "Size_B_ErrorCounter"]
     set TxNmtQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_TxNmtQ"] * 1024]
     set TxGenQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_TxGenQ"] * 1024]
     set TxSynQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_TxSynQ"] * 1024]
     set TxVetQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_TxVetQ"] * 1024]
     set RxVetQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_RxVetQ"] * 1024]
     set K2UQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_K2UQ"] * 1024]
     set U2KQ_Size   [expr [xget_hw_parameter_value $mhsinst "Size_KB_U2KQ"] * 1024]
     set Pdo_Size   [xget_hw_parameter_value $mhsinst "Size_B_Pdo"]
     set Qheader_Size 16
     set statusControlSize 2048
     set accumulator [expr $DynBuf0_Size + $DynBuf1_Size + $Errcntr_Size + $TxNmtQ_Size + $TxGenQ_Size + $TxSynQ_Size + $TxVetQ_Size + $RxVetQ_Size + $K2UQ_Size + $U2KQ_Size + $Pdo_Size]
     set total [expr $accumulator + 7 * $Qheader_Size + $statusControlSize ]
     return $total
}


proc generate {drv_handle} {
     set mhsinst [xget_hw_parent_handle $drv_handle]
     xdefine_include_file $drv_handle "xparameters.h" "axi_hostinterface" "C_BASEADDR" "C_HIGHADDR" "axi_hostinterface" "C_HOST_BASEADDR" "C_HOST_HIGHADDR" "gBaseDynBuf0" "gBaseDynBuf1" "gBaseErrCntr" "gBaseTxNmtQ" "gBaseTxGenQ" "gBaseTxSynQ" "gBaseTxVetQ" "gBaseRxVetQ" "gBaseK2UQ" "gBaseU2KQ" \
      "gBasePdo" "gBaseRes" "Conv_Size_KB_DynBuf0" "Conv_Size_KB_DynBuf1" "Conv_Size_B_ErrorCounter" "Conv_Size_KB_TxNmtQ" "Conv_Size_KB_TxGenQ" "Conv_Size_KB_TxSynQ" "Conv_Size_KB_TxVetQ" \
      "Conv_Size_KB_RxVetQ" "Conv_Size_KB_K2UQ" "Conv_Size_KB_U2KQ" "Conv_Size_B_Pdo" "gVersionMajor" "gVersionMinor" "gVersionRevision" "gVersionCount"

     my_xdefine_include_file $drv_handle "hostiflib-mem.h" "axi_hostinterface" "gBaseDynBuf0" "gBaseDynBuf1" "gBaseErrCntr" "gBaseTxNmtQ" "gBaseTxGenQ" "gBaseTxSynQ" "gBaseTxVetQ" "gBaseRxVetQ" "gBaseK2UQ" "gBaseU2KQ" \
      "gBasePdo" "gBaseRes" "Conv_Size_KB_DynBuf0" "Conv_Size_KB_DynBuf1" "Conv_Size_B_ErrorCounter" "Conv_Size_KB_TxNmtQ" "Conv_Size_KB_TxGenQ" "Conv_Size_KB_TxSynQ" "Conv_Size_KB_TxVetQ" \
      "Conv_Size_KB_RxVetQ" "Conv_Size_KB_K2UQ" "Conv_Size_KB_U2KQ" "Conv_Size_B_Pdo" "gVersionMajor" "gVersionMinor" "gVersionRevision" "gVersionCount"

}

proc my_xdefine_include_file {drv_handle file_name drv_string args} {
     # Open include file
     set file_handle [xopen_include_file $file_name]

     # Get all peripherals connected to this driver
     set periphs [xget_periphs $drv_handle]

     # Handle special cases
     set arg "NUM_INSTANCES"
     set posn [lsearch -exact $args $arg]
     if {$posn > -1} {
     # Define NUM_INSTANCES
     puts $file_handle "#define [xget_dname $drv_string $arg] [llength $periphs]"
     set args [lreplace $args $posn $posn]
     }
     # Check if it is a driver parameter

     lappend newargs
     foreach arg $args {
     set value [xget_value $drv_handle "PARAMETER" $arg]
     if {[llength $value] == 0} {
          lappend newargs $arg
     } else {
     puts $file_handle "#define [xget_dname $drv_string $arg] [xget_value $drv_handle "PARAMETER" $arg]"
     }
     }
     set args $newargs

     # Print all parameters for all peripherals
     set device_id 0
     foreach periph $periphs {
     foreach arg $args {
          if {[string compare -nocase "DEVICE_ID" $arg] == 0} {
          set value $device_id
          incr device_id
          } else {
          set value [xget_param_value $periph $arg]
          }
          set value [xformat_addr_string $value $arg]
          puts $file_handle "#define [my_xget_name $periph $arg] $value"
     }
     }
     puts $file_handle "\n /* INIT VECTOR */
                     \n #define HOSTIF_INIT_VEC {\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_DYNBUF0, HOSTIF_SIZE_DYNBUF0 },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_DYNBUF1, HOSTIF_SIZE_DYNBUF1 },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_ERRORCOUNTER, HOSTIF_SIZE_ERRORCOUNTER },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_TXNMTQ, HOSTIF_SIZE_TXNMTQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_TXGENQ, HOSTIF_SIZE_TXGENQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_TXSYNCQ, HOSTIF_SIZE_TXSYNCQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_TXVETHQ, HOSTIF_SIZE_TXVETHQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_RXVETHQ, HOSTIF_SIZE_RXVETHQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_K2UQ, HOSTIF_SIZE_K2UQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_U2KQ, HOSTIF_SIZE_U2KQ },\\\
                     \n\t\t\t\t\t\t  { HOSTIF_BASE_PDO, HOSTIF_SIZE_PDO },\\\
                     \n\t\t\t\t\t\t }"
     puts $file_handle "\n/******************************************************************/\n"
     close $file_handle
}


proc my_xget_name {periph_handle param} {
     set name [xget_value $periph_handle "NAME"]
     set name [string toupper $name]
     if {[string match C_* $param]} {
     set name [format "%s%s" $name [string range $param 2 end]]
     } elseif {[string compare $param "gBaseDynBuf0"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "DYNBUF0"]
     } elseif {[string compare $param "gBaseDynBuf1"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "DYNBUF1"]
     } elseif {[string compare $param "gBaseErrCntr"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "ERRORCOUNTER"]
     } elseif {[string compare $param "gBaseTxNmtQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "TXNMTQ"]
     } elseif {[string compare $param "gBaseTxGenQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "TXGENQ"]
     } elseif {[string compare $param "gBaseTxSynQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "TXSYNCQ"]
     } elseif {[string compare $param "gBaseTxVetQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "TXVETHQ"]
     } elseif {[string compare $param "gBaseRxVetQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "RXVETHQ"]
     } elseif {[string compare $param "gBaseK2UQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "K2UQ"]
     } elseif {[string compare $param "gBaseU2KQ"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "U2KQ"]
     } elseif {[string compare $param "gBasePdo"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "PDO"]
     } elseif {[string compare $param "gBaseRes"] == 0} {
     set name [format "%s%s" "HOSTIF_BASE_" "RES"]
     } elseif {[string compare $param "Conv_Size_KB_DynBuf0"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "DYNBUF0"]
     } elseif {[string compare $param "Conv_Size_KB_DynBuf1"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "DYNBUF1"]
     }  elseif {[string compare $param "Conv_Size_B_ErrorCounter"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "ERRORCOUNTER"]
     } elseif {[string compare $param "Conv_Size_KB_TxNmtQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "TXNMTQ"]
     } elseif {[string compare $param "Conv_Size_KB_TxGenQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "TXGENQ"]
     } elseif {[string compare $param "Conv_Size_KB_TxSynQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "TXSYNCQ"]
     } elseif {[string compare $param "Conv_Size_KB_TxVetQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "TXVETHQ"]
     } elseif {[string compare $param "Conv_Size_KB_RxVetQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "RXVETHQ"]
     } elseif {[string compare $param "Conv_Size_KB_K2UQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "K2UQ"]
     } elseif {[string compare $param "Conv_Size_KB_U2KQ"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "U2KQ"]
     } elseif {[string compare $param "Conv_Size_B_Pdo"] == 0} {
     set name [format "%s%s" "HOSTIF_SIZE_" "PDO"]
     }  elseif {[string compare $param "gVersionMajor"] == 0} {
     set name [format "%s%s" "HOSTIF_" "VERSION_MAJOR"]
     }  elseif {[string compare $param "gVersionMinor"] == 0} {
     set name [format "%s%s" "HOSTIF_" "VERSION_MINOR"]
     }  elseif {[string compare $param "gVersionRevision"] == 0} {
     set name [format "%s%s" "HOSTIF_" "VERSION_REVISION"]
     }  elseif {[string compare $param "gVersionCount"] == 0} {
     set name [format "%s%s" "HOSTIF_" "VERSION_COUNT"]
     }  else {
     set name [format "%s%s" "HOSTIF_SIZE_" $param]
     }
     return $name
}
