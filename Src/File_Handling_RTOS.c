/*
 * File_Handling_RTOS.c
 *
 *  Created on: 30-April-2020
 *      Author: Controllerstech
 */

#include "File_Handling_RTOS.h"
#include "stm32f3xx_hal.h"



/* =============================>>>>>>>> NO CHANGES AFTER THIS LINE =====================================>>>>>>> */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count
BYTE open = 0;

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint8_t Mount_SD (const TCHAR* path)
{
	fresult = f_mount(&fs, path, 1);
	if (fresult == FR_OK)
		return 1;
	return 0;
}

uint8_t Unmount_SD (const TCHAR* path)
{
	fresult = f_mount(NULL, path, 1);
}

FRESULT Write_File (char *name, char *data)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, strlen(data), &bw);
	    	if (fresult != FR_OK)
	    	{
	    		return fresult;
	    	}

	    	/* Close file */
	    	fresult = f_close(&fil);
	    	if (fresult != FR_OK)
	    		return fresult;
	    	}
	    }
	    return fresult;
}

FRESULT open_file(char *name){
	if((fresult = f_stat (name, &fno)) == FR_OK){
		if((fresult = f_open(&fil, name, FA_READ)) == FR_OK){
			open = 1;
		}
	}
	return fresult;
}

FRESULT close_file(){
	if(open){
		if((fresult = f_close(&fil)) == FR_OK){
			open = 0;
		}
	}
	return fresult;
}

FRESULT Read_config_File (char *name, char* buffer, uint8_t buffer_size, uint32_t offset)
{
	if(open){
		f_lseek(&fil, offset);
		fresult = f_read (&fil, buffer, buffer_size, &br);
	}else{
		open_file(name);
		fresult = f_read (&fil, buffer, buffer_size, &br);
	}
	return fresult;
}

FRESULT Create_File (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
	    return fresult;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
		if (fresult != FR_OK)
		{
		    return fresult;
		}

		fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Remove_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		return fresult;
	} else {
		fresult = f_unlink (name);
	}
	return fresult;
}

uint32_t Check_SD_Space (void)
{
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);
    return (uint32_t)(fre_clust * pfs->csize * 0.5);
}


