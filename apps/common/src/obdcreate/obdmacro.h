/**
********************************************************************************
\file   obdcreate/obdmacro.h

\brief  Macros for OD creation

This file contains the macros for the creation of the OD data structure tables and
initialization code. The macros will be used in the OD creation module
obdcreate.c. The object dictionaries in the directory objdicts are defined by
these macros.

For further information on a POWERLINK object dictionary (OD) refer to
\ref sect_powerlink_od

### Access Rights

Combinations of \ref sect_obdAccessRights "access rights" are possible. Some of
these access rights are automatically set by the macros. Which objects contain
which access rights depends on the applied device profile or on the application.

Macro                           | Automatically assigned rights
--------------------------------|------------------------------
OBD_SUBINDEX_RAM_VAR            | None
OBD_SUBINDEX_RAM_VAR_RG         | kObdAccRange
OBD_SUBINDEX_RAM_VSTRING        | None
OBD_SUBINDEX_RAM_OSTRING        | None
OBD_SUBINDEX_RAM_DOMAIN         | kObdAccVar
OBD_SUBINDEX_RAM_USERDEF        | kObdAccVar
OBD_SUBINDEX_RAM_USERDEF_RG     | kObdAccVar \| kObdAccRange

For readable and writable objects (kObdAccRead and kObdAccWrite) there is always
a value available in ROM, which contains the default value, as well as a current
value in RAM. The default value is copied to the current value in the NMT states
kNmtGsResetApplication or kNmtGsResetCommunication and on the command to restore
the default parameters (object 0x1011, NMT_RestoreDefParam_REC). The current value
can be written and read for both SDO accesses or from the application.

Read-only objects created with macro OBD_SUBINDEX_RAM_... but without kEplObdAccWrite
cannot be written per SDO. However, the application can modify its object data by
calling the function oplk_writeLocalObject(). Therefore, a value is created in
ROM as well as in RAM.

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

// This header file must be included multiple times, therefore no single
// inclusion macro is used!

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#if defined(OBD_DEFINE_MACRO)

#if defined(OBD_CREATE_ROM_DATA)
//------------------------------------------------------------------------------
// Macros for generating the ROM tables of the OD are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()                                                             static CONST DWORD  dwObd_OBK_g = 0x0000;
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()
#define OBD_BEGIN_PART_MANUFACTURER()
#define OBD_BEGIN_PART_DEVICE()
#define OBD_END_PART()

// index macros
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)          static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt); \
                                                                                    static CONST dtyp ROM xDef##ind##_0x01_g = (def);
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)      static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt); \
                                                                                    static CONST dtyp ROM xDef##ind##_0x01_g = (def);
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)       static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt); \
                                                                                    static CONST dtyp ROM xDef##ind##_0x01_g = (def);
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name)     static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt);
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)               static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = 0; \
                                                                                    static CONST tObdUnsigned64 ROM xDef##ind##_0x01_g = (def);
// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)                   static CONST dtyp ROM xDef##ind##_##sub##_g = val;
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)     static CONST dtyp ROM xDef##ind##_##sub##_g[3] = {val, low, high};
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)                    static char  MEM szCur##ind##_##sub##_g[size + 1]; \
                                                                                    static CONST tObdVStringDef ROM xDef##ind##_##sub##_g = {size, val, szCur##ind##_##sub##_g};

#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)                         static BYTE  MEM bCur##ind##_##sub##_g[size]; \
                                                                                    static CONST tObdOStringDef ROM xDef##ind##_##sub##_g = {size, ((BYTE*)""), bCur##ind##_##sub##_g};
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)               static CONST dtyp ROM xDef##ind##_##sub##_g = val;
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high) static CONST dtyp ROM xDef##ind##_##sub##_g[3] = {val, low, high};
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)

#elif defined(OBD_CREATE_RAM_DATA)
//------------------------------------------------------------------------------
// Macros for generating the RAM tables of the OD are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()
#define OBD_BEGIN_PART_MANUFACTURER()
#define OBD_BEGIN_PART_DEVICE()
#define OBD_END_PART()

// index macros
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)           static dtyp           MEM axCur##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)       static tObdUnsigned8  MEM xCur##ind##_0x00_g; \
                                                                                     static dtyp           MEM axCur##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)        static tObdVarEntry   MEM aVarEntry##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name)      static tObdVarEntry   MEM aVarEntry##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)                static tObdUnsigned8  MEM xCur##ind##_0x00_g; \
                                                                                     static tObdUnsigned64 MEM axCur##ind##_g[cnt];
// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)                    static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)      static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)                     static tObdVString  MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)                          static tObdOString  MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)                  static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)                                 static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)                static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high)  static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)              static tObdVarEntry MEM VarEntry##ind##_##sub##_g;

#elif defined(OBD_CREATE_SUBINDEX_TAB)
//------------------------------------------------------------------------------
// Macros for generating the sub-index tables of the OD are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()
#define OBD_BEGIN_PART_MANUFACTURER()
#define OBD_BEGIN_PART_DEVICE()
#define OBD_END_PART()

// index macros
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)                                     static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[cnt]= {
#define OBD_END_INDEX(ind)                                                      OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)      static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,                        &xDef##ind##_0x00_g, NULL}, \
                                                                                {1, typ,          (acc) | kObdAccArray,              &xDef##ind##_0x01_g, &axCur##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)  static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, (acc),                             &xDef##ind##_0x00_g, &xCur##ind##_0x00_g}, \
                                                                                {1, typ,           (acc) | kObdAccArray,              &xDef##ind##_0x01_g, &axCur##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)   static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,                        &xDef##ind##_0x00_g, NULL}, \
                                                                                {1, typ,          (acc) | kObdAccArray | kObdAccVar, &xDef##ind##_0x01_g, &aVarEntry##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name) static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,                        &xDef##ind##_0x00_g, NULL}, \
                                                                                {1, typ,          (acc) | kObdAccArray | kObdAccVar, NULL,                &aVarEntry##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)           static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8,  (acc),                           &xDef##ind##_0x00_g, &xCur##ind##_0x00_g}, \
                                                                                {1, kObdTypeUInt64, (acc) | kObdAccArray,            &xDef##ind##_0x01_g, &axCur##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)                    {sub, typ,             (acc),                             &xDef##ind##_##sub##_g,    &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)      {sub, typ,             (acc) | kObdAccRange,              &xDef##ind##_##sub##_g[0], &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)                  {sub, typ,             (acc),                             NULL,                      &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)                     {sub, kObdTypeVString, (acc)/* | kObdAccVar*/,            &xDef##ind##_##sub##_g,    &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)                          {sub, kObdTypeOString, (acc)/* | kObdAccVar*/,            &xDef##ind##_##sub##_g,    &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)                                 {sub, kObdTypeDomain,  (acc) | kObdAccVar,                NULL,                      &VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)                {sub, typ,             (acc) | kObdAccVar,                &xDef##ind##_##sub##_g,    &VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high)  {sub, typ,             (acc) | kObdAccVar | kObdAccRange, &xDef##ind##_##sub##_g[0], &VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)              {sub, typ,             (acc) | kObdAccVar,                NULL,                      &VarEntry##ind##_##sub##_g},


#elif defined(OBD_CREATE_INDEX_TAB)
//------------------------------------------------------------------------------
// Macros for generating the index tables of the OD are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()                                                static  tObdEntry  aObdTabGeneric_g[]      = {
#define OBD_BEGIN_PART_MANUFACTURER()                                           static  tObdEntry  aObdTabManufacturer_g[] = {
#define OBD_BEGIN_PART_DEVICE()                                                 static  tObdEntry  aObdTabDevice_g[]       = {
#define OBD_END_PART()                                                          {OBD_TABLE_INDEX_END, (tObdSubEntryPtr)(void*)&dwObd_OBK_g, 0, FALSE}};

// index macros
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)                                     {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], cnt, call},
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)      {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], (cnt)+1, call},
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)  {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], (cnt)+1, call},
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)   {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], (cnt)+1, call},
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name) {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], (cnt)+1, call},
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)           {ind, (tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0], (cnt)+1, call},

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)
#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high)
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)

#elif defined(OBD_CREATE_INIT_FUNCTION)
//------------------------------------------------------------------------------
// Macros for generating the initialization functions are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()                                                pInitParam->pGenericPart      = (tObdEntryPtr)&aObdTabGeneric_g[0];
#define OBD_BEGIN_PART_MANUFACTURER()                                           pInitParam->pManufacturerPart = (tObdEntryPtr)&aObdTabManufacturer_g[0];
#define OBD_BEGIN_PART_DEVICE()                                                 pInitParam->pDevicePart       = (tObdEntryPtr)&aObdTabDevice_g[0];
#define OBD_END_PART()

// index macros
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name)
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)
#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high)
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)

#else
//------------------------------------------------------------------------------
// Nothing specified macros are empty
//------------------------------------------------------------------------------

///\{
/**
********************************************************************************
\name OD Section Limiters

The Object Dictionary is always introduced with the macro OBD_BEGIN.
OBD_END ends the definition of the Object Dictionary. No other macros can be
used for the Object Dictionary outside of the boundary set by OBD_BEGIN and
OBD_END.
*/
#define OBD_BEGIN()                                                         ///< Begin of an OD definition section
#define OBD_END()                                                           ///< End of an OD definition section
///\}

///\{
/**
********************************************************************************
\name OD Partition Limiters

These  macros are always positioned between the macros OBD_BEGIN and OBD_END and
introduce a partition of the Object Dictionary. The "GENERIC" range is always
utilized for the index range 0x1000 to 0x1FFF, the "MANUFACTURER" section
is used for 0x2000 to 0x5FFF and the "DEVICE" section for index range 0x6000 to
0x9FFF. Each of these macros must be used a single time (and only a single time!)
within an Object Dictionary. The applicable range or partial section is always
closed with the macro OBD_END_PART.
*/
#define OBD_BEGIN_PART_GENERIC()                                            ///< Begin of generic partition
#define OBD_BEGIN_PART_MANUFACTURER()                                       ///< Begin of manufacturer partition
#define OBD_BEGIN_PART_DEVICE()                                             ///< Begin of device partition
#define OBD_END_PART()                                                      ///< End of partition
///\}

///\{
/**
********************************************************************************
\name OD Index Definitions

These macros are found within a partition of the Object Dictionary. They are
located within the range between the macros OBD_BEGIN_PART_... and OBD_END_PART.
They must be ordered with ascending object index. These macros define an index
entry in the Object Dictionary. An index entry is therefore always introduced
with the macro OBD_BEGIN_INDEX_... and ended with OBD_END_INDEX. The suffix
..._RAM indicates that the sub index table is located in RAM.
*/
/**
\brief Begin of index entry

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the calling of the api function
                            \ref oplk_cbGenericObdAccess for this index entry.
                            The callback function is always called if an object
                            has been read or written. It doesn’t matter if the
                            access comes from the application or per SDO. The
                            POWERLINK ctrl module has one callback function that
                            must be specified for some objects in the index range
                            0x1000 through 0x1FFF and may be specified for any
                            other object indexes, including application-specific
                            objects.
*/
#define OBD_BEGIN_INDEX_RAM(ind, cnt, call)

/**
\brief Begin of array index entry

This macro simplifies the definition of arrays. It can replace the OBD_BEGIN_INDEX_...,
OBD_END_INDEX and OBD_SUBINDEX_... macros. The macro reduces the allocation of
const memory, because of less sub-index table entries. The drawback is that it
needs a little more RAM.

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the generic callback for this index entry
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param def                  Default value of object
*/
#define OBD_RAM_INDEX_RAM_ARRAY(ind, cnt, call, typ, acc, dtyp, name, def)

/**
\brief Begin of array index entry with writable subindex 0

This macro simplifies the definition of arrays. It can replace the OBD_BEGIN_INDEX_...,
OBD_END_INDEX and OBD_SUBINDEX_... macros. The macro reduces the allocation of
const memory, because of less sub-index table entries. The drawback is that it
needs a little more RAM. For objects created with this macro the array size could
be modified (sub-index 0 is writable).

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the generic callback for this index entry
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param def                  Default value of object
*/
#define OBD_RAM_INDEX_RAM_ARRAY_ALT(ind, cnt, call, typ, acc, dtyp, name, def)
/**
\brief Begin of var-array index entry

Same as \ref OBD_RAM_INDEX_RAM_ARRAY but it contains a tVarEntry information structure
like it was defined by the \ref OBD_SUBINDEX_RAM_DOMAIN macro so it must be linked to
a variable by oplk_linkObject().

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the generic callback for this index entry.
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param def                  Default value of object
*/
#define OBD_RAM_INDEX_RAM_VARARRAY(ind, cnt, call, typ, acc, dtyp, name, def)

/**
\brief Begin of var-array index entry without initialization

Same as \ref OBD_RAM_INDEX_RAM_VARARRAY but it isn't initialized with a default
value.

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the generic callback for this index entry
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
*/
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind, cnt, call, typ, acc, dtyp, name)

/**
\brief Begin of PDO mapped variable

This macro generates an entry for a PDO mapping object.

\param ind                  Object index of the entry to be defined
\param cnt                  Number of sub-indices whithin this index entry
\param call                 Flag for enabling the generic callback for this index entry
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param name                 Name of object
\param def                  Default value of object
*/
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind, cnt, call, acc, name, def)

#define OBD_END_INDEX(ind)                                                  ///< End of index entry
///\}

///\{
/**
********************************************************************************
\name OD Sub-index Entry Definitions

The sub-indexes are now defined within an index entry. They are always located
within the range between the macros OBD_BEGIN_INDEX_... and OBD_END_INDEX and
must be ordered with ascending sub-index. Since there are various object types
and therefore various data types that have to be created, there are different
macros as well.

The macros ..._DOMAIN, ..._USERDEF and ..._USERDEF_RG define a variable
information structure of type tVarEntry in the RAM along with the sub-index entry.
This structure contains the data length and a pointer to the data. Upon initialization
of the openPOWERLINK stack with the function oplk_create() all variable information is
deleted. The application has to link these objects to its own variables by calling
the function oplk_linkObject().
*/

/**
\brief  Definition of a variable

This macro defines an object for variables that have a defined data length, which
is determined by the object type (e.g. UNSIGNED8, UNSIGNED16, INTEGER8, etc.).
These objects cannot be mapped to a PDO!

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param val                  Default value of this sub-index entry
*/
#define OBD_SUBINDEX_RAM_VAR(ind, sub, typ, acc, dtyp, name, val)

/**
\brief  Definition of a variable with range check

Same as \ref OBD_SUBINDEX_RAM_VAR but with a range check for minimum and
maximum values. If CONFIG_OBD_CHECK_OBJECT_RANGE is set to TRUE, the openPOWERLINK
stack automatically checks the value range before an object is written to
(from the application or per SDO).

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param val                  Default value of this sub-index entry
\param low                  Lower limit of valid range
\param high                 Higher limit of valid range
*/
#define OBD_SUBINDEX_RAM_VAR_RG(ind, sub, typ, acc, dtyp, name, val, low, high)

/**
\brief  Definition of a VSTRING variable

This macro defines an object for variables of type VSTRING.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param name                 Name of object
\param size                 Maximum length of the string in RAM (incl. \0 termination)
\param val                  Default value of this sub-index entry
*/
#define OBD_SUBINDEX_RAM_VSTRING(ind, sub, acc, name, size, val)

/**
\brief  Definition of an OSTRING variable

This macro defines an object for variables of type OSTRING.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param name                 Name of object
\param size                 Maximum length of the string in RAM (incl. \0 termination)

*/
#define OBD_SUBINDEX_RAM_OSTRING(ind, sub, acc, name, size)

/**
Definition of a variable which isn't initialized

The suffix ..._NOINIT defines objects which have no default value. That means the
openPOWERLINK stack does not initialize those variables with a default value on
NMT reset events. It is the responsibility of the application to initialize those
objects.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
*/
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind, sub, typ, acc, dtyp, name)

/**
\brief  Definition of a DOMAIN variable

This macro defines an object for variables of type DOMAIN.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param name                 Name of object
*/
#define OBD_SUBINDEX_RAM_DOMAIN(ind, sub, acc, name)

/**
\brief  Definition of a variable with user-specific type

Objects, which the user wants to manage in his application, can be created with
this _USERDEF macro. Only these objects can be mapped to a PDO as process variables.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param val                  Default value of this sub-index entry
*/
#define OBD_SUBINDEX_RAM_USERDEF(ind, sub, typ, acc, dtyp, name, val)

/**
\brief  Definition of a variable with user-specific type and range check

Same as \ref OBD_SUBINDEX_RAM_USERDEF but with a range check for minimum and
maximum values.  If CONFIG_OBD_CHECK_OBJECT_RANGE is set to TRUE, the openPOWERLINK
stack automatically checks the value range before an object is written to
(from the application or per SDO).

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
\param val                  Default value of this sub-index entry
\param low                  Lower limit of valid range
\param high                 Higher limit of valid range
*/
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind, sub, typ, acc, dtyp, name, val, low, high)

/**
\brief  Definition of a variable with user-specific type that isn't initialized

The suffix ..._NOINIT defines objects which have no default value. That means the
openPOWERLINK stack does not initialize those variables with a default value on
NMT reset events. It is the responsibility of the application to initialize those
objects.

\param ind                  Object index of the entry to be defined
\param sub                  Sub-index of the entry to be defined
\param typ                  Coded Object type (see \ref tObdType)
\param acc                  Access rights for object (see \ref sect_obdAccessRights "access rights")
\param dtyp                 C Data type definition used for this object
\param name                 Name of object
*/
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind, sub, typ, acc, dtyp, name)
///\}

#endif

#elif defined(OBD_UNDEFINE_MACRO)
//------------------------------------------------------------------------------
// Undefine the macros now
//------------------------------------------------------------------------------

// generic macros
#undef OBD_BEGIN
#undef OBD_END

// partition macros
#undef OBD_BEGIN_PART_GENERIC
#undef OBD_BEGIN_PART_MANUFACTURER
#undef OBD_BEGIN_PART_DEVICE
#undef OBD_END_PART

// index macros
#undef OBD_BEGIN_INDEX_RAM
#undef OBD_END_INDEX
#undef OBD_RAM_INDEX_RAM_ARRAY
#undef OBD_RAM_INDEX_RAM_ARRAY_ALT
#undef OBD_RAM_INDEX_RAM_VARARRAY
#undef OBD_RAM_INDEX_RAM_VARARRAY_NOINIT
#undef OBD_RAM_INDEX_RAM_PDO_MAPPING

// subindex macros
#undef OBD_SUBINDEX_RAM_VAR
#undef OBD_SUBINDEX_RAM_VAR_RG
#undef OBD_SUBINDEX_RAM_VSTRING
#undef OBD_SUBINDEX_RAM_OSTRING
#undef OBD_SUBINDEX_RAM_VAR_NOINIT
#undef OBD_SUBINDEX_RAM_DOMAIN
#undef OBD_SUBINDEX_RAM_USERDEF
#undef OBD_SUBINDEX_RAM_USERDEF_RG
#undef OBD_SUBINDEX_RAM_USERDEF_NOINIT

#else

#error "Please define the type of initialization before including this file!"

#endif
