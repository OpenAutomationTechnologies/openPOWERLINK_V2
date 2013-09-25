/**
********************************************************************************
\file   obdmacro.h

\brief  Macros for OD creation

This file contains macros for creation the OD data structure tables and
initialization code.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#if defined (OBD_CREATE_ROM_DATA)
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
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)             static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt); \
                                                                                static CONST dtyp ROM xDef##ind##_0x01_g = (def);
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)          static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt); \
                                                                                static CONST dtyp ROM xDef##ind##_0x01_g = (def);
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)       static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = (cnt);
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)                static CONST tObdUnsigned8 ROM xDef##ind##_0x00_g = 0; \
                                                                                static CONST tObdUnsigned64 ROM xDef##ind##_0x01_g = (def);
// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)                     static CONST dtyp ROM xDef##ind##_##sub##_g        = val;
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)         static CONST dtyp ROM xDef##ind##_##sub##_g[3]     = {val,low,high};
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,size,val)                     static char  MEM szCur##ind##_##sub##_g[size+1]; \
                                                                                static CONST tObdVStringDef ROM xDef##ind##_##sub##_g = {size, val, szCur##ind##_##sub##_g};

#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)                         static BYTE  MEM bCur##ind##_##sub##_g[size]; \
                                                                                static CONST tObdOStringDef ROM xDef##ind##_##sub##_g = {size, ((BYTE*)""), bCur##ind##_##sub##_g};
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)                 static CONST dtyp ROM xDef##ind##_##sub##_g        = val;
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)     static CONST dtyp ROM xDef##ind##_##sub##_g[3]     = {val,low,high};
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)

#elif defined (OBD_CREATE_RAM_DATA)
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
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)             static dtyp         MEM axCur##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)          static tObdVarEntry MEM aVarEntry##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)       static tObdVarEntry MEM aVarEntry##ind##_g[cnt];
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)                static tObdUnsigned8 MEM xCur##ind##_0x00_g; \
                                                                                static tObdUnsigned64 MEM axCur##ind##_g[cnt];
// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)                     static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)         static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,size,val)                     static tObdVString  MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)                         static tObdOString  MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)                  static dtyp         MEM xCur##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)                               static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)                 static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)     static tObdVarEntry MEM VarEntry##ind##_##sub##_g;
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)              static tObdVarEntry MEM VarEntry##ind##_##sub##_g;

#elif defined (OBD_CREATE_SUBINDEX_TAB)
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
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)                                       static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[cnt]= {
#define OBD_END_INDEX(ind)                                                      OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)             static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,          &xDef##ind##_0x00_g,   NULL}, \
                                                                                {1, typ,          (acc)|kObdAccArray, &xDef##ind##_0x01_g,   &axCur##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)          static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,                     &xDef##ind##_0x00_g,   NULL}, \
                                                                                {1, typ,          (acc)|kObdAccArray|kObdAccVar, &xDef##ind##_0x01_g,   &aVarEntry##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)       static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccCR,                     &xDef##ind##_0x00_g,   NULL}, \
                                                                                {1, typ,          (acc)|kObdAccArray|kObdAccVar, NULL,                  &aVarEntry##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)                static tObdSubEntry MEM aObdSubEntry##ind##Ram_g[]= { \
                                                                                {0, kObdTypeUInt8, kObdAccRW,          &xDef##ind##_0x00_g,   &xCur##ind##_0x00_g}, \
                                                                                {1, kObdTypeUInt64, (acc)|kObdAccArray, &xDef##ind##_0x01_g,   &axCur##ind##_g[0]}, \
                                                                                OBD_END_SUBINDEX()};

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)                     {sub,typ,            (acc),                        &xDef##ind##_##sub##_g,   &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)         {sub,typ,            (acc)|kObdAccRange,           &xDef##ind##_##sub##_g[0],&xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)                  {sub,typ,            (acc),                        NULL,   &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,size,val)                     {sub,kObdTypeVString,(acc)/*|kObdAccVar*/,         &xDef##ind##_##sub##_g,   &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)                         {sub,kObdTypeOString,(acc)/*|kObdAccVar*/,         &xDef##ind##_##sub##_g,   &xCur##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)                               {sub,kObdTypeDomain, (acc)|kObdAccVar,             NULL,                     &VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)                 {sub,typ,           (acc)|kObdAccVar,             &xDef##ind##_##sub##_g,   &VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)     {sub,typ,           (acc)|kObdAccVar|kObdAccRange,&xDef##ind##_##sub##_g[0],&VarEntry##ind##_##sub##_g},
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)              {sub,typ,           (acc)|kObdAccVar,             NULL,    &VarEntry##ind##_##sub##_g},


#elif defined (OBD_CREATE_INDEX_TAB)
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
#define OBD_END_PART()                                                          {OBD_TABLE_INDEX_END,(tObdSubEntryPtr)(void*)&dwObd_OBK_g,0,NULL}};

// index macros
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)                                       {ind,(tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0],cnt,(tObdCallback)call},
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)             {ind,(tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0],(cnt)+1,(tObdCallback)call},
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)          {ind,(tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0],(cnt)+1,(tObdCallback)call},
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)       {ind,(tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0],(cnt)+1,(tObdCallback)call},
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)                {ind,(tObdSubEntryPtr)&aObdSubEntry##ind##Ram_g[0],(cnt)+1,(tObdCallback)call},

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,size,val)
#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)

#elif defined (OBD_CREATE_INIT_FUNCTION)
//------------------------------------------------------------------------------
// Macros for generating the initialization functions are used now
//------------------------------------------------------------------------------

// generic macros
#define OBD_BEGIN()
#define OBD_END()

// partition macros
#define OBD_BEGIN_PART_GENERIC()                                                pInitParam->pGenericPart      = (tObdEntryPtr) &aObdTabGeneric_g[0];
#define OBD_BEGIN_PART_MANUFACTURER()                                           pInitParam->pManufacturerPart = (tObdEntryPtr) &aObdTabManufacturer_g[0];
#define OBD_BEGIN_PART_DEVICE()                                                 pInitParam->pDevicePart       = (tObdEntryPtr) &aObdTabDevice_g[0];
#define OBD_END_PART()

// index macros
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,size,val)
#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)

#else
//------------------------------------------------------------------------------
// Nothing specified macros are empty
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
#define OBD_BEGIN_INDEX_RAM(ind,cnt,call)
#define OBD_END_INDEX(ind)
#define OBD_RAM_INDEX_RAM_ARRAY(ind,cnt,call,typ,acc,dtyp,name,def)
#define OBD_RAM_INDEX_RAM_VARARRAY(ind,cnt,call,typ,acc,dtyp,name,def)
#define OBD_RAM_INDEX_RAM_VARARRAY_NOINIT(ind,cnt,call,typ,acc,dtyp,name)
#define OBD_RAM_INDEX_RAM_PDO_MAPPING(ind,cnt,call,acc,name,def)

// subindex macros
#define OBD_SUBINDEX_RAM_VAR(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_VAR_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_VSTRING(ind,sub,acc,name,sizes,val)
#define OBD_SUBINDEX_RAM_OSTRING(ind,sub,acc,name,size)
#define OBD_SUBINDEX_RAM_VAR_NOINIT(ind,sub,typ,acc,dtyp,name)
#define OBD_SUBINDEX_RAM_DOMAIN(ind,sub,acc,name)
#define OBD_SUBINDEX_RAM_USERDEF(ind,sub,typ,acc,dtyp,name,val)
#define OBD_SUBINDEX_RAM_USERDEF_RG(ind,sub,typ,acc,dtyp,name,val,low,high)
#define OBD_SUBINDEX_RAM_USERDEF_NOINIT(ind,sub,typ,acc,dtyp,name)

#endif

#elif defined (OBD_UNDEFINE_MACRO)
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

