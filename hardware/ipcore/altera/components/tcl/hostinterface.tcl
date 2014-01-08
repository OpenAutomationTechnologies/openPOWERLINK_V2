# openPOWERLINK root path
set OPLK_path "../../../../.."

#borrowed from hw.tcl
set listVersionCmacro [list "VERSION_MAJOR" "VERSION_MINOR" "VERSION_REVISION" "VERSION_COUNT"]
set listSizeCmacro [list "SIZE_DYNBUF0" "SIZE_DYNBUF1" "SIZE_ERRORCOUNTER" "SIZE_TXNMTQ" "SIZE_TXGENQ" "SIZE_TXSYNCQ" "SIZE_TXVETHQ" "SIZE_RXVETHQ" "SIZE_K2UQ" "SIZE_U2KQ" "SIZE_TPDO" "SIZE_RPDO"]
set listBaseCmacro [list "BASE_DYNBUF0" "BASE_DYNBUF1" "BASE_ERRORCOUNTER" "BASE_TXNMTQ" "BASE_TXGENQ" "BASE_TXSYNCQ" "BASE_TXVETHQ" "BASE_RXVETHQ" "BASE_K2UQ" "BASE_U2KQ" "BASE_TPDO" "BASE_RPDO"]

proc generationCallback { instName tgtDir bspDir } {
    set LIB_HOSTIF_path $tgtDir

    puts ""
    puts "***********************************************************"
    puts " HOSTINTERFACE script for $instName instance "
    puts " (Version field = [getIpcoreVersion $::listVersionCmacro])"
    puts ""

    set headerFile "hostiflib-mem.h"
    puts "  -> generate $LIB_HOSTIF_path/$headerFile file"
    createHostifMemFile "$LIB_HOSTIF_path/$headerFile"

    puts "***********************************************************"
    puts ""

}

proc getIpcoreVersion { listVersionCmacro } {
    set versionString ""
    foreach cmacro $listVersionCmacro {
        set versionString "$versionString [get_module_assignment embeddedsw.CMacro.$cmacro]"
    }
    return $versionString
}

proc createHostifMemFile { fileName } {
    set headerFile [writeFile_open "$fileName"]
    writeFile_header $headerFile
    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "#ifndef __HOSTIF_MEM_H__\n"
    writeFile_string $headerFile "#define __HOSTIF_MEM_H__\n"
    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "/* VERSION */\n"
    writeFile_list_cmacro $headerFile $::listVersionCmacro
    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "/* BASE */\n"
    writeFile_list_cmacro $headerFile $::listBaseCmacro
    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "/* SIZE */\n"
    writeFile_list_cmacro $headerFile $::listSizeCmacro

    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "/* INIT VECTOR */\n"

    # Write initialization vector
    writeFile_string $headerFile "#define HOSTIF_INIT_VEC { \\"
    writeFile_string $headerFile "\n"

    # Get number of buffer, to know the vector length
    set numOfBuf [llength $::listBaseCmacro]

    set cnt 0
    foreach off_name $::listBaseCmacro siz_name $::listSizeCmacro {
        set tmpString "                        "
        set tmpString "${tmpString}{ HOSTIF_${off_name}, HOSTIF_${siz_name} }"

        incr cnt

        if { $cnt < $numOfBuf } {
            set tmpString "${tmpString}, "
        }

        set tmpString "${tmpString} \\"

        writeFile_string $headerFile $tmpString
        writeFile_string $headerFile "\n"
    }

    writeFile_string $headerFile "                      }"

    writeFile_string $headerFile "\n"
    writeFile_string $headerFile "#endif\n"
    writeFile_close $headerFile
}

proc writeFile_list_cmacro { fileId listCmacro } {
    foreach cmacro $listCmacro {
        set val [get_module_assignment embeddedsw.CMacro.$cmacro]
        set writeData "#define HOSTIF_$cmacro $val\n"
        puts -nonewline $fileId $writeData
    }
}

proc writeFile_string { fileId string } {
    puts -nonewline $fileId "$string"
}

proc writeFile_header { fileId } {
    set timeStamp [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]
    puts -nonewline $fileId "/***************************************************************************/\n"
    puts -nonewline $fileId "/* DO NOT MODIFY THIS FILE!                                                */\n"
    puts -nonewline $fileId "/* This file is generated automatically depending on the ipcore settings!  */\n"
    puts -nonewline $fileId "/* Hence, it is highly recommended to avoid manual modifications!          */\n"
    puts -nonewline $fileId "/*                                                                         */\n"
    puts -nonewline $fileId "/* timestamp = $timeStamp */\n"
    puts -nonewline $fileId "/***************************************************************************/\n"
}

proc writeFile_open { fileName } {
    set headerFile [open $fileName "w"]
    return $headerFile
}

proc writeFile_close { fileId } {
    close $fileId
}
