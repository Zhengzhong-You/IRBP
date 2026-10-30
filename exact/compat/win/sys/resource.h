/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#pragma once
#include <windows.h>
#include <psapi.h>
struct rusage { SIZE_T ru_maxrss; };
#define RUSAGE_SELF 0
inline int getrusage(int,rusage* usage) {PROCESS_MEMORY_COUNTERS p{};if(!GetProcessMemoryInfo(GetCurrentProcess(),&p,sizeof(p)))return -1;usage->ru_maxrss=p.PeakWorkingSetSize;return 0;}
