#!/usr/bin/tclsh
################################################################################
#
# \file  T011-1.tcl
#
# \brief Curly brackets from the same pair should be either
#        in the same line or in the same column
#
# Copyright (c) 2017, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

proc acceptPairs {} {
    global file parens index reclevel end

    while {$index != $end} {
        set nextToken [lindex $parens $index]
        set tokenValue [lindex $nextToken 0]
        set tokenName [lindex $nextToken 3]
        set tokenLine [lindex $nextToken 1]

        while { $tokenName == "newline" } {
            #puts "nl($index): $tokenName at $tokenLine"
            incr index
            set nextToken [lindex $parens $index]
            set tokenName [lindex $nextToken 3]
            set tokenLine [lindex $nextToken 1]

            if { $tokenName == "pp_define" } {
                #puts "found($index) $tokenName at $tokenLine"
                incr index
                set nextToken [lindex $parens $index]
                set tokenName [lindex $nextToken 3]
                set tokenLine [lindex $nextToken 1]
                # ignore every brace until the next newline
                while { $tokenName != "newline" } {
                    #puts "ignore($index) $tokenName at $tokenLine"
                    incr index
                    set nextToken [lindex $parens $index]
                    set tokenName [lindex $nextToken 3]
                    set tokenLine [lindex $nextToken 1]
                }
            }
        }

        if { $index == $end } {
            return
        }

        #puts "brace($index): $tokenName at $tokenLine"
        if {$tokenName == "leftbrace"} {
            #puts "handling leftbrace($index) level:$reclevel"
            incr index
            set leftParenLine [lindex $nextToken 1]
            set leftParenColumn [lindex $nextToken 2]

            acceptPairs

            if {$index == $end} {
                report $file $leftParenLine "opening curly bracket is not closed"
                return
            }

            set nextToken [lindex $parens $index]
            incr index
            set tokenValue [lindex $nextToken 0]
            set rightParenLine [lindex $nextToken 1]
            set rightParenColumn [lindex $nextToken 2]

            #puts "found rightparen at $rightParenLine"

            if {($leftParenLine != $rightParenLine) && ($leftParenColumn != $rightParenColumn)} {
                # make an exception for line continuation
                set leftLine [getLine $file $leftParenLine]
                set rightLine [getLine $file $rightParenLine]
                if {[string index $leftLine end] != "\\" && [string index $rightLine end] != "\\"} {
                    report $file $rightParenLine "closing curly bracket not in the same line or column"
                }
            }
        } else {
            return
        }
    }
}

foreach file [getSourceFileNames] {
    set parens [getTokens $file 1 0 -1 -1 {leftbrace rightbrace pp_define newline}]
    set index 0
    set reclevel 0
    set end [llength $parens]
    #puts "number of tokens $end"
    acceptPairs
    if {$index != $end} {
        report $file [lindex [lindex $parens $index] 1] "excessive closing bracket?"
    }
}
