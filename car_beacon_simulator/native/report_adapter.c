#include "sim.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define report_printf stored_report_printf
#define report_defer_printf stored_report_defer_printf
#include "run_report.c"
#undef report_printf
#undef report_defer_printf
static uint8_t journal[REPORT_FLASH_BYTES];
void report_flash_read(uint32_t offset,void *data,size_t length) {
    if(offset<=sizeof(journal) && length<=sizeof(journal)-offset) memcpy(data,journal+offset,length);
}
bool report_flash_erase(void) { memset(journal,0xff,sizeof(journal)); return true; }
bool report_flash_program(uint32_t offset,const void *page) {
    if(offset>sizeof(journal) || REPORT_PAGE_BYTES>sizeof(journal)-offset) return false;
    memcpy(journal+offset,page,REPORT_PAGE_BYTES); return true;
}
static void observed_report(bool deferred,const char *fmt,va_list args) {
    char text[512]; va_list copy; va_copy(copy,args);
    vsnprintf(text,sizeof(text),fmt,copy); va_end(copy);
    // Actual formatter handles line truncation and queue/pending overflow.
    report_format(deferred,fmt,args); sim_report_observe(text);
}
void report_printf(const char *fmt,...) { va_list a; va_start(a,fmt); observed_report(false,fmt,a); va_end(a); }
void report_defer_printf(const char *fmt,...) { va_list a; va_start(a,fmt); observed_report(true,fmt,a); va_end(a); }
