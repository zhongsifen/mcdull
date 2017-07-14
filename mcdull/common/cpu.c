/*****************************************************************************
 * cpu.c: cpu detection
 *****************************************************************************
 * Copyright (C) 2003-2010 x264 project
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
 *          Laurent Aimar <fenrir@via.ecp.fr>
 *          Jason Garrett-Glaser <darkshikari@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#ifdef _MSC_VER
#pragma warning( disable : 4113 )
#endif

#define _GNU_SOURCE // for sched_getaffinity
#include "common.h"
#include "cpu.h"

#if HAVE_PTHREAD && SYS_LINUX
#include <sched.h>
#endif
#if SYS_BEOS
#include <kernel/OS.h>
#endif
#if SYS_MACOSX || SYS_FREEBSD
#include <sys/types.h>
#include <sys/sysctl.h>
#endif
#if SYS_OPENBSD
#include <sys/param.h>
#include <sys/sysctl.h>
#include <machine/cpu.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

const x264_cpu_name_t x264_cpu_names[] = {
    {"Altivec", X264_CPU_ALTIVEC},
//  {"MMX",     X264_CPU_MMX}, // we don't support asm on mmx1 cpus anymore
    {"MMX2",    X264_CPU_MMX|X264_CPU_MMXEXT},
    {"MMXEXT",  X264_CPU_MMX|X264_CPU_MMXEXT},
//  {"SSE",     X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE}, // there are no sse1 functions in x264
    {"SSE2Slow",X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE2_IS_SLOW},
    {"SSE2",    X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2},
    {"SSE2Fast",X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE2_IS_FAST},
    {"SSE3",    X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE3},
    {"SSSE3",   X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE3|X264_CPU_SSSE3},
    {"FastShuffle",   X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SHUFFLE_IS_FAST},
    {"SSE4.1",  X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE3|X264_CPU_SSSE3|X264_CPU_SSE4},
    {"SSE4.2",  X264_CPU_MMX|X264_CPU_MMXEXT|X264_CPU_SSE|X264_CPU_SSE2|X264_CPU_SSE3|X264_CPU_SSSE3|X264_CPU_SSE4|X264_CPU_SSE42},
    {"Cache32", X264_CPU_CACHELINE_32},
    {"Cache64", X264_CPU_CACHELINE_64},
    {"SSEMisalign", X264_CPU_SSE_MISALIGN},
    {"LZCNT", X264_CPU_LZCNT},
    {"Slow_mod4_stack", X264_CPU_STACK_MOD4},
    {"ARMv6", X264_CPU_ARMV6},
    {"NEON",  X264_CPU_NEON},
    {"Fast_NEON_MRC",  X264_CPU_FAST_NEON_MRC},
    {"SlowCTZ", X264_CPU_SLOW_CTZ},
    {"SlowAtom", X264_CPU_SLOW_ATOM},
    {"", 0},
};

#if (ARCH_PPC && SYS_LINUX) || (ARCH_ARM && !HAVE_NEON)
#include <signal.h>
#include <setjmp.h>
static sigjmp_buf jmpbuf;
static volatile sig_atomic_t canjump = 0;

static void sigill_handler( int sig )
{
    if( !canjump )
    {
        signal( sig, SIG_DFL );
        raise( sig );
    }

    canjump = 0;
    siglongjmp( jmpbuf, 1 );
}
#endif

uint32_t x264_cpu_detect( void )
{
    return 0;
}

int x264_cpu_num_processors( void )
{
#if !HAVE_THREAD
    return 1;

#elif defined(_WIN32)
    static int np = 0;
    if (!np)
        np = x264_pthread_num_processors_np();
    return np;

#elif SYS_LINUX
    unsigned int bit;
    int np;
    cpu_set_t p_aff;
    memset( &p_aff, 0, sizeof(p_aff) );
    sched_getaffinity( 0, sizeof(p_aff), &p_aff );
    for( np = 0, bit = 0; bit < sizeof(p_aff); bit++ )
        np += (((uint8_t *)&p_aff)[bit / 8] >> (bit % 8)) & 1;
    return np;

#elif SYS_BEOS
    system_info info;
    get_system_info( &info );
    return info.cpu_count;

#elif SYS_MACOSX || SYS_FREEBSD || SYS_OPENBSD
    int ncpu;
    size_t length = sizeof( ncpu );
#if SYS_OPENBSD
    int mib[2] = { CTL_HW, HW_NCPU };
    if( sysctl(mib, 2, &ncpu, &length, NULL, 0) )
#else
    if( sysctlbyname("hw.ncpu", &ncpu, &length, NULL, 0) )
#endif
    {
        ncpu = 1;
    }
    return ncpu;

#else
    return 1;
#endif
}

int x264_cpu_num_physical_cores( void )
{
#ifdef _WIN32
#ifndef CACHE_FULLY_ASSOCIATIVE
#define CACHE_FULLY_ASSOCIATIVE 0xFF
    typedef enum _LOGICAL_PROCESSOR_RELATIONSHIP {
        RelationProcessorCore,
        RelationNumaNode,
        RelationCache
    } LOGICAL_PROCESSOR_RELATIONSHIP;
    typedef enum _PROCESSOR_CACHE_TYPE {
        CacheUnified,
        CacheInstruction,
        CacheData,
        CacheTrace
    } PROCESSOR_CACHE_TYPE;
    typedef struct _CACHE_DESCRIPTOR {
        BYTE   Level;
        BYTE   Associativity;
        WORD   LineSize;
        DWORD  Size;
        PROCESSOR_CACHE_TYPE Type;
    } CACHE_DESCRIPTOR, *PCACHE_DESCRIPTOR;
    typedef struct _SYSTEM_LOGICAL_PROCESSOR_INFORMATION {
        ULONG_PTR   ProcessorMask;
        LOGICAL_PROCESSOR_RELATIONSHIP Relationship;
        union {
            struct {
                BYTE  Flags;
            } ProcessorCore;
            struct {
                DWORD NodeNumber;
            } NumaNode;
            CACHE_DESCRIPTOR Cache;
            ULONGLONG  Reserved[2];
        };
    } SYSTEM_LOGICAL_PROCESSOR_INFORMATION, *PSYSTEM_LOGICAL_PROCESSOR_INFORMATION;
#endif
    SYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer[128];
    PSYSTEM_LOGICAL_PROCESSOR_INFORMATION p = buffer;
    BOOL (WINAPI *Glpi)(PSYSTEM_LOGICAL_PROCESSOR_INFORMATION, PDWORD);
    DWORD returnLength = sizeof(buffer);
    DWORD byteOffset = 0;
    static DWORD procCoreCount = 0;
    BOOL rc;

    if (procCoreCount)
        return procCoreCount;

    Glpi = GetProcAddress(GetModuleHandleA("kernel32"),"GetLogicalProcessorInformation");
    if (!Glpi)
        return procCoreCount = x264_cpu_num_processors();

    rc = Glpi(buffer, &returnLength);
    if (FALSE == rc)
        return procCoreCount = x264_cpu_num_processors();

    while (byteOffset < returnLength)
    {
        if (p->Relationship == RelationProcessorCore)
            procCoreCount++;
        byteOffset += sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
        p++;
    }

    if (!procCoreCount)
        procCoreCount = x264_cpu_num_processors();
    return procCoreCount;
#else
    return x264_cpu_num_processors();
#endif
}
