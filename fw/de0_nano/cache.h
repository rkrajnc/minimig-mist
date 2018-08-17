/* Copyright (c) 2015 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi> */

#ifndef _CACHE_H_
#define _CACHE_H_

#include "spr_defs.h"

static inline uint32_t dc_block_size(void)
{
	return (mfspr(SPR_DCCFGR) & SPR_DCCFGR_CBS) ? 32 : 16;
}

static inline uint32_t dc_sets(void)
{
	uint32_t dccfgr = mfspr(SPR_DCCFGR);

	return 1 << ((dccfgr & SPR_DCCFGR_NCS) >> 3);
}

static inline uint32_t ic_block_size(void)
{
	return (mfspr(SPR_ICCFGR) & SPR_ICCFGR_CBS) ? 32 : 16;
}

static inline uint32_t ic_sets(void)
{
	uint32_t iccfgr = mfspr(SPR_ICCFGR);

	return 1 << ((iccfgr & SPR_ICCFGR_NCS) >> 3);
}

static inline void icache_invalidate_all(void)
{
	uint32_t i;
	uint32_t cache_size;
	uint32_t block_size;

	block_size = ic_block_size();
	cache_size = block_size * ic_sets();
	for (i = 0; i < cache_size; i += block_size)
		mtspr(SPR_ICBIR, i);
}

static inline void dcache_invalidate_all(void)
{
	uint32_t i;
	uint32_t cache_size;
	uint32_t block_size;

	block_size = dc_block_size();
	cache_size = block_size * dc_sets();
	for (i = 0; i < cache_size; i += block_size)
		mtspr(SPR_DCBIR, i);
}

static inline void dcache_disable(void)
{
	mtspr(SPR_SR, mfspr(SPR_SR) & ~SPR_SR_DCE);
}

static inline void dcache_enable(void)
{
	mtspr(SPR_SR, mfspr(SPR_SR) | SPR_SR_DCE);
}


static inline void icache_disable(void)
{
	mtspr(SPR_SR, mfspr(SPR_SR) & ~SPR_SR_ICE);
}

static inline void icache_enable(void)
{
	mtspr(SPR_SR, mfspr(SPR_SR) | SPR_SR_ICE);
}

#endif
