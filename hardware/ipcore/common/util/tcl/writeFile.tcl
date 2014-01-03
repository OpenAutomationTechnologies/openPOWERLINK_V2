#!/usr/bin/tclsh
# This file provides functions to write a generated header file.

# Open a writeable file. The return value is the fileId
proc writeFile_open { fileName } {
    set headerFile [open $fileName "w"]
    return $headerFile
}

# Close the provided file
proc writeFile_close { fileId } {
    close $fileId
}

# Write a standard file header with current time
proc writeFile_header { fileId } {
    set timeStamp [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]

    puts -nonewline $fileId "//-------------------------------------------------------------------------\n"
    puts -nonewline $fileId "// DO NOT MODIFY THIS FILE!\n"
    puts -nonewline $fileId "// This file is generated automatically depending on the ipcore settings!\n"
    puts -nonewline $fileId "// Hence, it is highly recommended to avoid manual modifications!\n"
    puts -nonewline $fileId "//\n"
    puts -nonewline $fileId "// timestamp = $timeStamp\n"
    puts -nonewline $fileId "//-------------------------------------------------------------------------\n"
}

# Write a cmacro define into the file
proc writeFile_cmacro { fileId cmacro val } {
    set writeData "#define $cmacro $val\n"
    puts -nonewline $fileId $writeData
}

# Write a sting line into the file
proc writeFile_string { fileId string } {
    puts -nonewline $fileId "$string\n"
}

# Write empty line into the file
proc writeFile_emptyLine { fileId } {
    writeFile_string $fileId ""
}
