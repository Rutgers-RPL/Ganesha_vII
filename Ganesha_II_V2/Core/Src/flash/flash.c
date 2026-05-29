#include "flash.h"

#include "w25n01kv.h"
#include "gd5f1gq5xe.h"
#include "lfs.h"

#include "STRUCTS.h"

#include <stdint.h>

void flash_init(struct flash_dev *dev, enum flash_name name)
{
	dev->name = name;
}

lfs_ssize_t flash_mount(struct flash_dev *flash, const struct lfs_config *config)
{
	gd5f1gq5xe_unlock();
	int err = lfs_mount(&(flash->lfs), config);
	if (err) {
		lfs_format(&(flash->lfs), config);
		int res = lfs_mount(&(flash->lfs), config);
		if (res < 0) return -1;
	}
	flash_boot_count(flash, true);
	return lfs_fs_size(&(flash->lfs));
}

int flash_unmount(struct flash_dev *flash)
{
	return lfs_unmount(&(flash->lfs));
}

uint32_t flash_boot_count(struct flash_dev *flash, bool update)
{
	lfs_file_t file;
	uint32_t boot_count = 0;

	lfs_file_open(&(flash->lfs), &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_read(&(flash->lfs), &file, &boot_count, sizeof(boot_count));

	if (update) {
		++boot_count;
		lfs_file_rewind(&(flash->lfs), &file);
		lfs_file_write(&(flash->lfs), &file, &boot_count, sizeof(boot_count));
	}

	lfs_file_close(&(flash->lfs), &file);
	return boot_count;
}

uint32_t flash_open(struct flash_dev *flash, lfs_file_t *file, const char *filename)
{
	lfs_file_open(&(flash->lfs), file, filename, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
	return lfs_file_size(&(flash->lfs), file);
}

bool flash_append(struct flash_dev *flash, lfs_file_t *file, const uint8_t *bytes, const size_t size)
{
	lfs_ssize_t s = lfs_file_write(&(flash->lfs), file, bytes, size);
	lfs_file_sync(&(flash->lfs), file);
	return s == size;
}

int flash_close(struct flash_dev *flash, lfs_file_t *file)
{
	return lfs_file_close(&(flash->lfs), file);
}
