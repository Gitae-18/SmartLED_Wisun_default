#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

#ifndef FOCUS_TIMING_LOG
#define FOCUS_TIMING_LOG 0
#endif

#if FOCUS_TIMING_LOG
#undef DEBUG_AT_TO_PC
#endif

void uart6_log(const char *fmt, ...);
uint8_t focus_timing_log_enabled(void);
void timing_log(const char *fmt, ...);
void dbg_dump_uart6_with_tag(const char *tag, const uint8_t *p, uint16_t n);
void debug6(const char *s);
void PrintReceivedPacket(const char* prefix, const uint8_t* data, uint16_t length);

#endif /* DEBUG_LOG_H */
