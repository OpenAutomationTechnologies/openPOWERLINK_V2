Coding Guidelines {#page_coding}
=================

[TOC]

This page lists the coding guidelines which apply to the openPOWERLINK source
code. The coding guidelines should ensure that the code looks the same, even if
it is provided by different developers. It shall ensure that the code documenting
tools are working correctly, common programming errors are avoided and the
code remains maintainable.

# General {#sect_coding_general}

## White-space and Indentation {#sect_coding_whitespace}

- Use spaces to indent blocks.
- Use an indentation width of 4 spaces.

## Formatting {#sect_coding_format}

- Do not use hanging braces. Put braces associated with a control statement on
  the next line, indented to the same level as the control statement. Statements
  within the braces are indented to the next level (Allman style).
  ~~~{.c}
  if (condition)
  {
        // do something
  }
  else
  {
        // do something else
  }
  ~~~

- Put * next to the type rather than the variable name.
- Only declare one single variable in a line.
- All binary operators (operators that come between two values) shall have a
  space around the operator. Unary operators, such as ++, must not have a
  space between the operator and the variable.
- Put the opening brace immediately after the function name and add a space between a
  control statement (if, switch, for, ...) and the opening brace.
- Always put a linebreak after a conditional statement, even if the body is only
  a return or other simple action.
- Keep line length below 80 characters. This enables to view and compare two
  files side by side on the screen. However, this is not a strict rule! If the
  line is only some characters longer and a linebreak would decrease
  readability than you should keep it on a single line. If a line must be split
  the following rules should be followed:
    + Break after a comma.
    + Break after an operator.
    + Align the new line with the beginning of the expression on the previous line.

- Include a newline at every file end.
- Use UTF-8 file encodings and LF line endings.

## Common Programming Rules {#sect_coding_commonrules}

### switch/case fall through {#sect_coding_case}
Every case statement in a switch/case block shall be ended with a _break_
statement. If you ever need to do a fall-through you must enter the comment
<b>"// no break"</b> which shows that this fall-through was intentionally used.
Use exactly this statement as it can be used to avoid warnings of code analyzer
tools (e.g. Eclipse). Enumerations of case statements without code statements or
empty lines in between are valid cases of fall-through even without a special
comment.

~~~{.c}
switch (var)
{
        case val1:
        case val2:
                doSomething();
                break;

        case val3:
                doSomethingElse();
                // no break
                // This fall-through is intended because .....

        case val4:
        default:
                doSomethingOther();
                break;
}
~~~

### Using goto {#sect_coding_goto}

The use of the keyword goto is often tainted with a negative image. However,
there is one case where it makes sense to use it. As C provides no exceptions
it's sometimes difficult to clearly perform error handling and leave a function.
In this case (and __only__ in this case) the usage of goto could be used to
implement a single function exit.

__Example:__

~~~{.c}
        doSomething();
        if (error1)
        {
                retCode = error1;
                goto Exit;
        }
        doSomethingElse();
        if (error2)
        {
                retCode = error2;
                goto Exit;
        }
        return OK;

Exit:
        doCleanup();
        return ERROR;
~~~

# Naming Conventions {#sect_coding_naming_conventions}

##  Constants {#sect_coding_naming_constants}
Constants will be defined with the Preprocessor command '\#define'. As usual for
C programs, constants shall contain only upper-case letters. Multiple words must be
concatenated by an underline (_).

__Examples:__

        #define TRANSMIT_TIMEOUT    5000
        #define MDC_FILE_EXTENSION  ".MDC"

## Type Definitions {#sect_coding_naming_types}
Type definitions in C/C++ will be used to define enumerations, structures and
unions. They will be defined using the keyword 'typedef'. A type definition
must be prefixed with '_t_', except an enumeration type must be prefixed with
'_e_'. Enumeration constants must be prefixed with '_k_'.
The name is formatted in lower camel case format.
Every enumeration type must have an associated fixed-size type that can hold
the maximum possible enumerator. This avoids size mismatches across different
architectures.

__Examples:__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
typedef enum
{
        kModBit        = 1,    // bit access
        kModByte       = 8,    // byte access
        kModShort      = 16,   // short access
        kModLong       = 32,   // long access
} eModAccess;

typedef UINT8 tModAccess; // Associated with eModAccess

typedef struct
{
        UINT8          modAddr;        // Module address
        UINT8          modId;          // ID
        UINT16         piSize;         // size of process image
        tModAccess     modAccess;      // module access size
} tModInf;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

## Variables {#sect_coding_naming_variables}

Variables are written in lower camel case format and can be extended by a prefix
and postfix.

- Use descriptive names.
- Avoid reusing a single temporary variable for different purposes.
- Variables with a large scope should have long descriptive names, variables with
  a small scope can have short names.

The prefix of a variable is used to describe its contents in more detail. The
following prefixes are used:

Prefix          | Description
------          | -----------
a               | Array (may be used together with further prefixes)
p               | Pointer (could be used together with further prefixes)
f               | Flag/boolean
pfn             | Pointer to function

__Table 1:__ Variable prefixes

The suffix of a variable describes its declaration place and therefore also its
scope.

Suffix          | Description
--------------- | -----------
_g              | External global variable
_l              | Static global variable
_p              | Parameter

__Table 2:__ Variable suffixes

- An external global variable is labeled with the suffix '_g'. This shows that
  the variable could also be referenced in other source files. Global variables
  should be avoided were possible and used with care.

- A static global variable which is labeled with the suffix '_l' to show that it
  can be referenced in all functions of the file.

- Parameters are labeled with the suffix '_p'.

- Local variables get no suffix.

__Examples:__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
        static BOOL             fIsValid_l;
        UINT32                  statusFlags_g;

        UINT32 sysRegisterStatus (UINT32 statusFlags_p)
        {
                UINT32      oldStatusFlags;

                oldStatusFlags = statusFlags_g;
                statusFlags_g = statusFlags_p;

                return oldStatusFlags;
        }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

## Functions {#sect_coding_naming_functions}

Functions will also be written in lower camel case format. The function name
should be descriptive, start with a verb and should have a reasonable length.
To avoid naming conflicts, external functions must be prefixed with the module
ID which is separated from the function name by an underscore.

__Examples:__

~~~{.c}
processCnStateChange()
identu_getIdentResponse()
~~~

## File names {#sect_coding_naming_files}
Source and header file names are written in lower case format. If there are
several implementations of a module for different architectures or systems, the
architecture name is appended. It is separated by a dash.

__Examples:__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dll.h
circbuffer.c
eventucal-linux.c
eventucal-win32.c
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Data Types {#sect_coding_datatypes}

- Use the predefined macros located in oplk/basictypes.h for standard data types
  (e.g. BOOL, INT, UINT, INT8, UINT8,  INT16, UINT16, DOUBLE, ...)

# Comments {#sect_coding_comments}

## Doxygen tags {#sect_coding_comments_doxygen}
The documentation of the openPOWERLINK stack is generated by [Doxygen](http://www.doxygen.org).
Therefore you need to add Doxygen tags to your source code comments.

Doxygen allows different styles of tags. For openPOWERLINK the javadoc style
is used. Doxygen commands start with a backslash (\\).

\verbatim
/** \brief Brief description

This is a detailed description.
*/
\endverbatim

## File headers {#sect_coding_comments_fileheader}

A file header must be placed at the beginning of every source and header
file. The file header ensures that Doxygen recognizes and parses the file:

\verbatim
/**
********************************************************************************
\file

\brief  SHORT DESCRIPTION OF FILE

DETAILED DESCRIPTION OF FILE

\ingroup MODULE_ID
*******************************************************************************/
\endverbatim

## License and Copyright  {#sect_coding_comments_copyright}
openPOWERLINK is licensed under a BSD license. A common license header must be
appended after the file header of each source file. The following license header
is used:

~~~
/*------------------------------------------------------------------------------
Copyright (c) YEAR, AUTHOR
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
~~~
If you modify an existing file add your copyright after the existing copyrights.


## Function headers {#sect_coding_comments_function}

Each C function must be prefixed by a Doxygen function header so that Doxygen
can parse and document the function interface.

\verbatim
//------------------------------------------------------------------------------
/**
\brief  BRIEF FUNCTION DESCRIPTION

DETAILED FUNCTION DESCRIPTION

\param  PARAM1              DESCRIPTION OF PARAM1
\param  PARAM2              DESCRIPTION OF PARAM2
...

\return DESCRIPTION OF THE RETURN VALUE
\retval RETURN_VALUE1   DESCRIPTION OF RETURN_VALUE1
...

\ingroup MODULE_ID
*/
//------------------------------------------------------------------------------
\endverbatim

Every function (global and static) in a C file must be documented! However, the
module identification (\\ingroup MODULE_ID) must be removed from local functions
headers so that only global functions appear in the interface documentation of
the module.

