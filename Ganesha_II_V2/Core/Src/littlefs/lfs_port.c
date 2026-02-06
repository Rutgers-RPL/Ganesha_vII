#include "main.h"
#include "lfs.h"
#include "lfs_port.h"

extern SPI_HandleTypeDef hspi4;

// Sets FLASH_CS low
static void csLow() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

// Sets FLASH_CS high
static void csHigh() {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

// Sends array of commands or data
static void spiSend(uint8_t *send, uint16_t size) {
    HAL_SPI_Transmit(&hspi4, send, size, HAL_MAX_DELAY);
}

// Receives data and stores in location
static void spiReceive(uint8_t *location, uint16_t size) {
    HAL_SPI_Receive(&hspi4, location, size, HAL_MAX_DELAY);
}

struct lfs_config lfsconfig = {
    .read  = lfs_read,
    .prog  = lfs_prog,
    .erase = lfs_erase,
    .sync  = lfs_sync,

    .read_size      = W25N_PAGE_SIZE,
    .prog_size      = W25N_PAGE_SIZE,
    .block_size     = W25N_BLOCK_SIZE,
    .block_count    = W25N_BLOCK_COUNT,
    .cache_size     = W25N_PAGE_SIZE,		// Keep a multiple of page size
    .lookahead_size = 128,					// In BYTES, not bits (1 bit represents 1 block)
    .block_cycles   = 500,					// Write to a block x times before moving on (lower better for wear leveling, higher better for performance)
};

int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, uint8_t *location, lfs_size_t size){
	// Calculate page to read from and column within page to start at
	uint16_t page = block * W25N_PAGES_PER_BLOCK + (offset / W25N_PAGE_SIZE);
	uint16_t col = offset % W25N_PAGE_SIZE;

	// Read full page and put into cache
	csLow();
	uint8_t cmd1[] = {						// 4 byte command as per datasheet (1 is command, 2 is dummy, 3 and 4 tell which page to read)
			W25N_CMD_PAGE_DATA_READ,
			(page >> 16) & 0xFF,
			(page >> 8) & 0xFF,
			page & 0xFF
	};
	spiSend(cmd1, 4);
	csHigh();
	HAL_Delay(1);							// Page Data Read time is around 25-100 nanoseconds, so 1 millisecond is plenty wait

	// Read specific data from cache
	csLow();
	uint8_t cmd2[] = {						// 4 byte command as per datasheet (1 is command, 3 and 4 is where in page to start reading, 4 is dummy)
			W25N_CMD_READ_FROM_BUFFER,
			(col >> 8) & 0xFF,
			col & 0xFF,
			0x00
	};
	spiSend(cmd2, 4);
	spiReceive(location, size);
	csHigh();

	return 0;
}

int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, uint8_t *data, lfs_size_t size){
	// Calculate page to read from and column within page to start at
	uint16_t page = block * W25N_PAGES_PER_BLOCK + (offset / W25N_PAGE_SIZE);
	uint16_t col = offset % W25N_PAGE_SIZE;

	// Send Write Enable
	uint8_t writeEnable = W25N_CMD_WRITE_ENABLE;
	csLow();
	spiSend(&writeEnable, 1);
	csHigh();

	// Send Load Program Data
	csLow();
	uint8_t cmd1[] = {
			W25N_CMD_LOAD_PROGRAM_DATA,
			(col >> 8) & 0xFF,
			col & 0xFF
	};
	spiSend(cmd1, 3);
	spiSend(data, size);
	csHigh();

	// Send Program Execute command
	csLow();
	uint8_t cmd2[] = {
			W25N_CMD_PROGRAM_EXECUTE,
			(page >> 16) & 0xFF,
			(page >> 8) & 0xFF,
			page & 0xFF
	};
	spiSend(cmd2, 4);
	csHigh();

	// Wait for everything to be done (I want to use READ_STATUS_REGISTER to detect when the operation is done instead of always waiting 3 milliseconds. But if the operation always takes less than 3 milliseconds, it doesn't really matter)
	HAL_Delay(3);

	return 0;
}

int lfs_erase(const struct lfs_config *c, lfs_block_t block){
	// Calculate which page to erase
	uint32_t page = block * W25N_PAGES_PER_BLOCK;

	// Send Write Enable
	uint8_t writeEnable = W25N_CMD_WRITE_ENABLE;
	csLow();
	spiSend(&writeEnable, 1);
	csHigh();

	// Send Block Erase
	csLow();
	uint8_t cmd1[] = {
			W25N_CMD_BLOCK_ERASE,
			(page >> 16) && 0xFF,
			(page >> 8) && 0xFf,
			page & 0xFF
	};
	spiSend(cmd1, 4);
	csLow();

	HAL_Delay(10);

	return 0;
}

int lfs_sync(const struct lfs_config *c){
	// Writes and erases are sent immediately within the methods, so nothing to actually sync. Method is just required by lfs.
	return 0;
}
