
#ifndef ___FS_SDCARD_H___
#define ___FS_SDCARD_H___

#include "ff.h"
#include "xil_types.h"
#include "xparameters.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************** Function Prototypes ******************************/

#ifdef XPAR_PS7_SD_0_S_AXI_BASEADDR
INT     sd_fs_init(void);
INT     sd_open(FIL*pFile_p, char*strFilename_p, BYTE bMode_p);
INT     sd_read(FIL*pFile_p, void*pBuffer_p, UINT uiCount_p, UINT*pReadNum_p);
INT     sd_write(FIL*pFile_p, void*pBuffer_p, UINT uiCount_p, UINT*pWriteNum_p);
UINT    sd_get_fsize(FIL*pFile_p);
void    sd_close(FIL*pFile_p);

#endif
/************************** Variable Definitions *****************************/
#ifdef __cplusplus
}
#endif

#endif /* ___FS_SDCARD_H___ */
