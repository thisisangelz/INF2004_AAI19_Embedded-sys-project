/* Physical flash is 2 MiB. Reserve 8 KiB for BTstack and 512 KiB for logs.
 * Match REPORT_BT_FLASH_OFFSET in report_store.h and CMake's bank offset. */
FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 1528k
