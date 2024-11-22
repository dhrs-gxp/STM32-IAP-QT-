#include "flash.h"	

static uint16_t getSector(uint32_t addr) {
	int sectorVal = (addr - 0x08000000)/0x4000;
	
	if(sectorVal == 0) return 0x0000;
	else if(sectorVal == 1) return 0x0008;
	else if(sectorVal == 2) return 0x0010;
	else if(sectorVal == 3) return 0x0018;
	else if(sectorVal >= 4 && sectorVal <= 7) return 0x0020;
	else if(sectorVal >= 8 && sectorVal <= 15) return 0x0028;
	else if(sectorVal >= 16 && sectorVal <= 23) return 0x0030;
	else if(sectorVal >= 24 && sectorVal <= 31) return 0x0038;
	else if(sectorVal >= 32 && sectorVal <= 39) return 0x0040;
	else if(sectorVal >= 40 && sectorVal <= 47) return 0x0048;
	else if(sectorVal >= 48 && sectorVal <= 55) return 0x0050;
	else if(sectorVal >= 56 && sectorVal <= 63) return 0x0058;
	else return 0x0000;
}

void flashErase(uint32_t startAddr, uint32_t endAddr)
{
	int startSector = getSector(startAddr);
	int endSector = getSector(endAddr);
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	for(int SectorCounter = startSector; SectorCounter <= endSector; SectorCounter += 8)
		FLASH_EraseSector(SectorCounter, VoltageRange_3);

	FLASH_Lock();
}