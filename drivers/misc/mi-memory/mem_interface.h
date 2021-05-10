#ifndef __MEM_INTERFACE_H__
#define __MEM_INTERFACE_H__

#include "../../scsi/ufs/ufshcd.h"

#define SD_ASCII_STD true
#define SD_RAW false

/* Health descriptor parameters offsets in bytes*/
enum health_desc_param {
	HEALTH_DESC_PARAM_LEN			= 0x0,
	HEALTH_DESC_PARAM_TYPE			= 0x1,
	HEALTH_DESC_PARAM_EOL_INFO		= 0x2,
	HEALTH_DESC_PARAM_LIFE_TIME_EST_A	= 0x3,
	HEALTH_DESC_PARAM_LIFE_TIME_EST_B	= 0x4,
};

u8 memblock_mem_size_in_gb(void);

void get_ufs_hba_data(struct ufs_hba *mi_hba);

void send_ufs_hba_data(struct ufs_hba **mi_hba);

int ufs_get_string_desc(void *buf, int size, enum device_desc_param pname,
	bool ascii_std);

int ufs_read_desc_param(enum desc_idn desc_id, u8 desc_index, u8 param_offset,
		void *buf, u8 param_size);
int ufshcd_read_desc_mi(struct ufs_hba *hba, enum desc_idn desc_id,
		int desc_index, void *buf, u32 size);
extern int ufshcd_read_desc_param(struct ufs_hba *hba, enum desc_idn desc_id,
	int desc_index, u8 param_offset, u8 *param_read_buf, u8 param_size);
extern int ufshcd_read_string_desc(struct ufs_hba *hba, int desc_index,
		u8 *buf, u32 size, bool ascii);
#endif
