/*
 * Amazon FreeRTOS POSIX V1.1.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file time.h
 * @brief Time types.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/time.h.html
 */

#ifndef _FREERTOS_POSIX_TIME_H_
#define _FREERTOS_POSIX_TIME_H_

/* FreeRTOS+POSIX includes. */
#include "FreeRTOS_POSIX/sys/types.h"
#include "FreeRTOS_POSIX/signal.h"

/**
 * @name Unit conversion constants.
 */
/**@{ */
#define MICROSECONDS_PER_SECOND    ( 1000000LL )                                   /**< Microseconds per second. */
#define NANOSECONDS_PER_SECOND     ( 1000000000LL )                                /**< Nanoseconds per second. */
#define NANOSECONDS_PER_TICK       ( NANOSECONDS_PER_SECOND / configTICK_RATE_HZ ) /**< Nanoseconds per FreeRTOS tick. */
/**@} */

/**
 * @name Clock identifiers.
 */
/**@{ */
#define CLOCK_REALTIME     0     /**< The identifier of the system-wide clock measuring real time. */
#define CLOCK_MONOTONIC    1     /**< The identifier for the system-wide monotonic clock.*/
/**@} */

/**
 * @name A number used to convert the value returned by the clock() function into seconds.
 */
/**@{ */
#define CLOCKS_PER_SEC    ( ( clock_t ) configTICK_RATE_HZ )
/**@} */

/**
 * @name Flag indicating time is absolute.
 *
 * For functions taking timer objects, this refers to the clock associated with the timer.
 */
/**@{ */
#define TIMER_ABSTIME    0x01
/**@} */

#if !defined( posixconfigENABLE_TIMESPEC ) || ( posixconfigENABLE_TIMESPEC == 1 )

/**
 * @brief represents an elapsed time
 */
    struct timespec
    {
        time_t tv_sec; /**< Seconds. */
        long tv_nsec;  /**< Nanoseconds. */
    };
#endif

#if !defined( posixconfigENABLE_ITIMERSPEC ) || ( posixconfigENABLE_ITIMERSPEC == 1 )

/**
 * @brief timer
 */
    struct itimerspec
    {
        struct timespec it_interval; /**< Timer period. */
        struct timespec it_value;    /**< Timer expiration. */
    };
#endif

/**
 * @brief Report CPU time used.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock.html
 *
 * @return  The number of FreeRTOS ticks since the scheduler
 * was started minus the ticks spent in the idle task.
 *
 * @note This function does NOT report the number of ticks spent by the calling thread.
 */
clock_t clock( void );

/**
 * @brief Access a process CPU-time clock.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock_getcpuclockid.html
 *
 * @retval EPERM
 *
 * @note This function is currently unsupported.
 *
 */
int clock_getcpuclockid( pid_t pid,
                         clockid_t * clock_id );

/**
 * @brief Returns the resolution of a clock.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock_getres.html
 *
 * @note clock_id is ignored
 * @note This function stores the resolution of the FreeRTOS tick count in the object res points to.
 *
 * @retval 0 - Upon successful execution
 */
int clock_getres( clockid_t clock_id,
                  struct timespec * res );

/**
 * @brief Returns the current value for the specified clock, clock_id.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock_gettime.html
 *
 * @note clock_id is ignored
 * @note  this function does not check for overflows of time_t.
 *
 * @retval 0 - Upon successful completion.
 */
int clock_gettime( clockid_t clock_id,
                   struct timespec * tp );

/**
 * @brief High resolution sleep with specifiable clock.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock_nanosleep.html
 *
 * @note clock_id is ignored, as this function uses the FreeRTOS tick count as its clock.
 * @note flags is ignored, if INCLUDE_vTaskDelayUntil is 0. i.e. the FreeRTOS function vTaskDelayUntil isn't available.
 * @note rmtp is also ignored, as signals are not implemented.
 *
 * @retval 0 - Upon successful completion.
 * @retval EINVAL - The rqtp argument specified a nanosecond value less than zero or greater than or equal to 1000 million.
 */
int clock_nanosleep( clockid_t clock_id,
                     int flags,
                     const struct timespec * rqtp,
                     struct timespec * rmtp );

/**
 * @brief Sets the time for the specified clock.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/clock_settime.html
 *
 * @retval -1 with errno set to EPERM.
 *
 * @note This function is currently unsupported, as FreeRTOS does not provide a function to modify the tick count.
 */
int clock_settime( clockid_t clock_id,
                   const struct timespec * tp );

/**
 * @brief High resolution sleep.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/nanosleep.html
 *
 * @note rmtp is ignored, as signals are not implemented.
 *
 * @retval 0 - Upon successful completion.
 * @retval -1 - The rqtp argument is invalid OR the rqtp argument specified a nanosecond value less than zero or greater than or equal to 1000 million.
 *
 */
int nanosleep( const struct timespec * rqtp,
               struct timespec * rmtp );

/**
 * @brief Create a per-process timer.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/timer_create.html
 *
 * @note clock_id is ignored, as this function used the FreeRTOS tick count as its clock.
 * @note evp.sigev_notify must be set to SIGEV_THREAD, since signals are currently not supported.
 *
 * @retval 0 - Upon successful completion, with location referenced by timerid updated.
 * @retval -1 - If an error occurs. errno is also set.
 *
 * @sideeffect Possible errno values
 * <br>
 * ENOTSUP - If evp is NULL OR evp->sigen_notify == SIGEV_SIGNAL.
 * <br>
 * EAGAIN - The system lacks sufficient signal queuing resources to honor the request.
 */
int timer_create( clockid_t clockid,
                  struct sigevent * evp,
                  timer_t * timerid );

/**
 * @brief Delete a per-process timer.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/timer_delete.html
 *
 * @retval 0 - Upon successful completion.
 */
int timer_delete( timer_t timerid );

/**
 * @brief Get the timer overrun count.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/timer_getoverrun.html
 *
 * @retval 0 - Always return 0, since signals are not supported.
 */
int timer_getoverrun( timer_t timerid );

/**
 * @brief Get the amount of time until the timer expires.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/timer_gettime.html
 *
 * @retval 0 - Upon successful completion.
 */
int timer_gettime( timer_t timerid,
                   struct itimerspec * value );

/**
 * @brief Set the time until the next expiration of the timer.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/functions/timer_settime.html
 *
 * @retval 0 - Upon successful completion.
 * @retval -1 - An error occurred, errno is also set.
 *
 * @sideeffect Possible errno values
 * <br>
 * EINVAL - A value structure specified a nanosecond value less than zero or greater than or equal to 1000 million,
 * AND the it_value member of that structure did not specify zero seconds and nanoseconds.
 */
int timer_settime( timer_t timerid,
                   int flags,
                   const struct itimerspec * value,
                   struct itimerspec * ovalue );

#ifdef GRPC_COMPAT
/***************************************/

/* time.h -- An implementation of the standard Unix <sys/time.h> file.
   Written by Geoffrey Noer <noer@cygnus.com>
   Public domain; no rights reserved. */

/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1982, 1986, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)time.h	8.5 (Berkeley) 5/4/95
 * $FreeBSD: head/sys/sys/time.h 346176 2019-04-13 04:46:35Z imp $
 */

#ifndef _SYS_TIME_H_
#define	_SYS_TIME_H_

//#include <_ansi.h>
//#include <sys/cdefs.h>
//#include <sys/_timeval.h>
//#include <sys/types.h>
//#include <sys/timespec.h>

//#if __BSD_VISIBLE || __POSIX_VISIBLE >= 200112 || __XSI_VISIBLE
//#include <sys/select.h>
//#endif

struct timezone {
	int	tz_minuteswest;	/* minutes west of Greenwich */
	int	tz_dsttime;	/* type of dst correction */
};
#define	DST_NONE	0	/* not on dst */
#define	DST_USA		1	/* USA style dst */
#define	DST_AUST	2	/* Australian style dst */
#define	DST_WET		3	/* Western European dst */
#define	DST_MET		4	/* Middle European dst */
#define	DST_EET		5	/* Eastern European dst */
#define	DST_CAN		6	/* Canada */

#ifndef _SUSECONDS_T_DECLARED
typedef long   suseconds_t;    /* long */
#define _SUSECONDS_T_DECLARED
#endif

#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED
/*
 * Structure returned by gettimeofday(2) system call, and used in other calls.
 */
struct timeval {
    time_t      tv_sec;     /* seconds */
    suseconds_t tv_usec;    /* and microseconds */
};
#else
#error "Already defined?"
#endif /* _TIMEVAL_DEFINED */


#if __BSD_VISIBLE
struct bintime {
	time_t	sec;
	uint64_t frac;
};

static __inline void
bintime_addx(struct bintime *_bt, uint64_t _x)
{
	uint64_t _u;

	_u = _bt->frac;
	_bt->frac += _x;
	if (_u > _bt->frac)
		_bt->sec++;
}

static __inline void
bintime_add(struct bintime *_bt, const struct bintime *_bt2)
{
	uint64_t _u;

	_u = _bt->frac;
	_bt->frac += _bt2->frac;
	if (_u > _bt->frac)
		_bt->sec++;
	_bt->sec += _bt2->sec;
}

static __inline void
bintime_sub(struct bintime *_bt, const struct bintime *_bt2)
{
	uint64_t _u;

	_u = _bt->frac;
	_bt->frac -= _bt2->frac;
	if (_u < _bt->frac)
		_bt->sec--;
	_bt->sec -= _bt2->sec;
}

typedef unsigned int u_int;

static __inline void
bintime_mul(struct bintime *_bt, u_int _x)
{
	uint64_t _p1, _p2;

	_p1 = (_bt->frac & 0xffffffffull) * _x;
	_p2 = (_bt->frac >> 32) * _x + (_p1 >> 32);
	_bt->sec *= _x;
	_bt->sec += (_p2 >> 32);
	_bt->frac = (_p2 << 32) | (_p1 & 0xffffffffull);
}

static __inline void
bintime_shift(struct bintime *_bt, int _exp)
{

	if (_exp > 0) {
		_bt->sec <<= _exp;
		_bt->sec |= _bt->frac >> (64 - _exp);
		_bt->frac <<= _exp;
	} else if (_exp < 0) {
		_bt->frac >>= -_exp;
		_bt->frac |= (uint64_t)_bt->sec << (64 + _exp);
		_bt->sec >>= -_exp;
	}
}

typedef __int64_t sbintime_t;    /* sys/types.h */

#define	bintime_clear(a)	((a)->sec = (a)->frac = 0)
#define	bintime_isset(a)	((a)->sec || (a)->frac)
#define	bintime_cmp(a, b, cmp)						\
	(((a)->sec == (b)->sec) ?					\
	    ((a)->frac cmp (b)->frac) :					\
	    ((a)->sec cmp (b)->sec))

#define	SBT_1S	((sbintime_t)1 << 32)
#define	SBT_1M	(SBT_1S * 60)
#define	SBT_1MS	(SBT_1S / 1000)
#define	SBT_1US	(SBT_1S / 1000000)
#define	SBT_1NS	(SBT_1S / 1000000000) /* beware rounding, see nstosbt() */
#define	SBT_MAX	0x7fffffffffffffffLL

static __inline int
sbintime_getsec(sbintime_t _sbt)
{

	return (_sbt >> 32);
}

static __inline sbintime_t
bttosbt(const struct bintime _bt)
{

	return (((sbintime_t)_bt.sec << 32) + (_bt.frac >> 32));
}

static __inline struct bintime
sbttobt(sbintime_t _sbt)
{
	struct bintime _bt;

	_bt.sec = _sbt >> 32;
	_bt.frac = _sbt << 32;
	return (_bt);
}

/*
 * Decimal<->sbt conversions.  Multiplying or dividing by SBT_1NS results in
 * large roundoff errors which sbttons() and nstosbt() avoid.  Millisecond and
 * microsecond functions are also provided for completeness.
 *
 * These functions return the smallest sbt larger or equal to the
 * number of seconds requested so that sbttoX(Xtosbt(y)) == y.  Unlike
 * top of second computations below, which require that we tick at the
 * top of second, these need to be rounded up so we do whatever for at
 * least as long as requested.
 *
 * The naive computation we'd do is this
 *	((unit * 2^64 / SIFACTOR) + 2^32-1) >> 32
 * However, that overflows. Instead, we compute
 *	((unit * 2^63 / SIFACTOR) + 2^31-1) >> 32
 * and use pre-computed constants that are the ceil of the 2^63 / SIFACTOR
 * term to ensure we are using exactly the right constant. We use the lesser
 * evil of ull rather than a uint64_t cast to ensure we have well defined
 * right shift semantics. With these changes, we get all the ns, us and ms
 * conversions back and forth right.
 */
static __inline int64_t
sbttons(sbintime_t _sbt)
{
	uint64_t ns;

	ns = _sbt;
	if (ns >= SBT_1S)
		ns = (ns >> 32) * 1000000000;
	else
		ns = 0;

	return (ns + (1000000000 * (_sbt & 0xffffffffu) >> 32));
}

static __inline sbintime_t
nstosbt(int64_t _ns)
{
	sbintime_t sb = 0;

	if (_ns >= SBT_1S) {
		sb = (_ns / 1000000000) * SBT_1S;
		_ns = _ns % 1000000000;
	}
	/* 9223372037 = ceil(2^63 / 1000000000) */
	sb += ((_ns * 9223372037ull) + 0x7fffffff) >> 31;
	return (sb);
}

static __inline int64_t
sbttous(sbintime_t _sbt)
{

	return ((1000000 * _sbt) >> 32);
}

static __inline sbintime_t
ustosbt(int64_t _us)
{
	sbintime_t sb = 0;

	if (_us >= SBT_1S) {
		sb = (_us / 1000000) * SBT_1S;
		_us = _us % 1000000;
	}
	/* 9223372036855 = ceil(2^63 / 1000000) */
	sb += ((_us * 9223372036855ull) + 0x7fffffff) >> 31;
	return (sb);
}

static __inline int64_t
sbttoms(sbintime_t _sbt)
{

	return ((1000 * _sbt) >> 32);
}

static __inline sbintime_t
mstosbt(int64_t _ms)
{
	sbintime_t sb = 0;

	if (_ms >= SBT_1S) {
		sb = (_ms / 1000) * SBT_1S;
		_ms = _ms % 1000;
	}
	/* 9223372036854776 = ceil(2^63 / 1000) */
	sb += ((_ms * 9223372036854776ull) + 0x7fffffff) >> 31;
	return (sb);
}



/*-
 * Background information:
 *
 * When converting between timestamps on parallel timescales of differing
 * resolutions it is historical and scientific practice to round down rather
 * than doing 4/5 rounding.
 *
 *   The date changes at midnight, not at noon.
 *
 *   Even at 15:59:59.999999999 it's not four'o'clock.
 *
 *   time_second ticks after N.999999999 not after N.4999999999
 */

static __inline void
bintime2timespec(const struct bintime *_bt, struct timespec *_ts)
{

	_ts->tv_sec = _bt->sec;
	_ts->tv_nsec = ((uint64_t)1000000000 *
	    (uint32_t)(_bt->frac >> 32)) >> 32;
}

static __inline void
timespec2bintime(const struct timespec *_ts, struct bintime *_bt)
{

	_bt->sec = _ts->tv_sec;
	/* 18446744073 = int(2^64 / 1000000000) */
	_bt->frac = _ts->tv_nsec * (uint64_t)18446744073LL;
}

static __inline void
bintime2timeval(const struct bintime *_bt, struct timeval *_tv)
{

	_tv->tv_sec = _bt->sec;
	_tv->tv_usec = ((uint64_t)1000000 * (uint32_t)(_bt->frac >> 32)) >> 32;
}

static __inline void
timeval2bintime(const struct timeval *_tv, struct bintime *_bt)
{

	_bt->sec = _tv->tv_sec;
	/* 18446744073709 = int(2^64 / 1000000) */
	_bt->frac = _tv->tv_usec * (uint64_t)18446744073709LL;
}

static __inline struct timespec
sbttots(sbintime_t _sbt)
{
	struct timespec _ts;

	_ts.tv_sec = _sbt >> 32;
	_ts.tv_nsec = sbttons((uint32_t)_sbt);
	return (_ts);
}

static __inline sbintime_t
tstosbt(struct timespec _ts)
{

	return (((sbintime_t)_ts.tv_sec << 32) + nstosbt(_ts.tv_nsec));
}

static __inline struct timeval
sbttotv(sbintime_t _sbt)
{
	struct timeval _tv;

	_tv.tv_sec = _sbt >> 32;
	_tv.tv_usec = sbttous((uint32_t)_sbt);
	return (_tv);
}

static __inline sbintime_t
tvtosbt(struct timeval _tv)
{

	return (((sbintime_t)_tv.tv_sec << 32) + ustosbt(_tv.tv_usec));
}

/* Operations on timespecs */
#define	timespecclear(tvp)	((tvp)->tv_sec = (tvp)->tv_nsec = 0)
#define	timespecisset(tvp)	((tvp)->tv_sec || (tvp)->tv_nsec)
#define	timespeccmp(tvp, uvp, cmp)					\
	(((tvp)->tv_sec == (uvp)->tv_sec) ?				\
	    ((tvp)->tv_nsec cmp (uvp)->tv_nsec) :			\
	    ((tvp)->tv_sec cmp (uvp)->tv_sec))

#define	timespecadd(tsp, usp, vsp)					\
	do {								\
		(vsp)->tv_sec = (tsp)->tv_sec + (usp)->tv_sec;		\
		(vsp)->tv_nsec = (tsp)->tv_nsec + (usp)->tv_nsec;	\
		if ((vsp)->tv_nsec >= 1000000000L) {			\
			(vsp)->tv_sec++;				\
			(vsp)->tv_nsec -= 1000000000L;			\
		}							\
	} while (0)
#define	timespecsub(tsp, usp, vsp)					\
	do {								\
		(vsp)->tv_sec = (tsp)->tv_sec - (usp)->tv_sec;		\
		(vsp)->tv_nsec = (tsp)->tv_nsec - (usp)->tv_nsec;	\
		if ((vsp)->tv_nsec < 0) {				\
			(vsp)->tv_sec--;				\
			(vsp)->tv_nsec += 1000000000L;			\
		}							\
	} while (0)

#ifndef _KERNEL			/* NetBSD/OpenBSD compatible interfaces */

#define	timerclear(tvp)		((tvp)->tv_sec = (tvp)->tv_usec = 0)
#define	timerisset(tvp)		((tvp)->tv_sec || (tvp)->tv_usec)
#define	timercmp(tvp, uvp, cmp)					\
	(((tvp)->tv_sec == (uvp)->tv_sec) ?				\
	    ((tvp)->tv_usec cmp (uvp)->tv_usec) :			\
	    ((tvp)->tv_sec cmp (uvp)->tv_sec))
#define	timeradd(tvp, uvp, vvp)						\
	do {								\
		(vvp)->tv_sec = (tvp)->tv_sec + (uvp)->tv_sec;		\
		(vvp)->tv_usec = (tvp)->tv_usec + (uvp)->tv_usec;	\
		if ((vvp)->tv_usec >= 1000000) {			\
			(vvp)->tv_sec++;				\
			(vvp)->tv_usec -= 1000000;			\
		}							\
	} while (0)
#define	timersub(tvp, uvp, vvp)						\
	do {								\
		(vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec;		\
		(vvp)->tv_usec = (tvp)->tv_usec - (uvp)->tv_usec;	\
		if ((vvp)->tv_usec < 0) {				\
			(vvp)->tv_sec--;				\
			(vvp)->tv_usec += 1000000;			\
		}							\
	} while (0)
#endif
#endif /* __BSD_VISIBLE */

/*
 * Names of the interval timers, and structure
 * defining a timer setting.
 */
#define	ITIMER_REAL	0
#define	ITIMER_VIRTUAL	1
#define	ITIMER_PROF	2

struct itimerval {
	struct	timeval it_interval;	/* timer interval */
	struct	timeval it_value;	/* current value */
};

/* C++ needs to know that types and declarations are C, not C++.  */
#ifdef  __cplusplus
# define __BEGIN_DECLS  extern "C" {
# define __END_DECLS    }
#else
# define __BEGIN_DECLS
# define __END_DECLS
#endif

__BEGIN_DECLS
// moved from hal_time.h
struct tm
{
    int tm_sec;
    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
    int tm_yday;
    int tm_isdst;
#ifdef __TM_GMTOFF
    long __TM_GMTOFF;
#endif
#ifdef __TM_ZONE
    const char *__TM_ZONE;
#endif
};

//clock() already defined earlier in this file.
//clock_t clock(void);
double difftime(time_t _time2, time_t _time1);
time_t mktime(struct tm *_timeptr);
time_t time(time_t *_timer);
#ifndef _REENT_ONLY
char *asctime(const struct tm *_tblock);
char *ctime(const time_t *_time);
struct tm *gmtime(const time_t *_timer);
struct tm *localtime(const time_t *_timer);
#endif
size_t strftime(char *__restrict _s, size_t _maxsize,
                const char *__restrict _fmt, const struct tm *__restrict _t);

char *asctime_r(const struct tm *__restrict, char *__restrict);
char *ctime_r(const time_t *, char *);
struct tm *gmtime_r(const time_t *__restrict, struct tm *__restrict);
struct tm *localtime_r(const time_t *__restrict, struct tm *__restrict);
__END_DECLS
// end of moved from hal_time.h



#ifndef _KERNEL
#include <time.h>


__BEGIN_DECLS
int utimes (const char *__path, const struct timeval *__tvp);

#if __BSD_VISIBLE
int adjtime (const struct timeval *, struct timeval *);
int futimes (int, const struct timeval *);
int lutimes (const char *, const struct timeval *);
int settimeofday (const struct timeval *, const struct timezone *);
#endif

#if __MISC_VISIBLE || __XSI_VISIBLE
int getitimer (int __which, struct itimerval *__value);
int setitimer (int __which, const struct itimerval *__restrict __value,
					struct itimerval *__restrict __ovalue);
#endif

int gettimeofday (struct timeval *__restrict __p,
			  void *__restrict __tz);

#if __GNU_VISIBLE
int futimesat (int, const char *, const struct timeval [2]);
#endif

#ifdef _COMPILING_NEWLIB
int _gettimeofday (struct timeval *__p, void *__tz);
#endif

__END_DECLS

#endif /* !_KERNEL */
#include <machine/_time.h>

#endif /* !_SYS_TIME_H_ */

#endif /* GRPC_COMPAT */

#endif /* ifndef _FREERTOS_POSIX_TIME_H_ */
