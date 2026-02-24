#pragma once

#include <basalt.h>

#define BASALT_POOL_MAX_ELEMENTS	0x00FFFFFF
#define BASALT_POOL_MAX_GENERATIONS	0xFF
#define BASALT_POOL_HANDLE_NULL		0xFFFFFFFF

typedef uint32_t Basalt_PoolHandle;

typedef struct Basalt_Pool_t
{
	uint8_t *data;
	uint8_t *generations;
	uint32_t *nexts;
	uint32_t *prevs;
	uint32_t head;
	uint32_t tail;

	uint32_t element_size;
	uint32_t size;
	uint32_t capacity;

	uint32_t *masks;
	uint32_t *indices;
	uint32_t num_free_indices;
} Basalt_Pool;

Basalt_Result basalt_poolInitialize(Basalt_Pool *pool, uint32_t element_size, uint32_t capacity);
Basalt_Result basalt_poolShutdown(Basalt_Pool *pool);

Basalt_PoolHandle basalt_poolAddElement(Basalt_Pool *pool, const void *data);
Basalt_Result basalt_poolRemoveElement(Basalt_Pool *pool, Basalt_PoolHandle handle);
void *basalt_poolGetElement(const Basalt_Pool *pool, Basalt_PoolHandle handle);

void *basalt_poolGetElementByIndex(const Basalt_Pool *pool, uint32_t index);
uint32_t basalt_poolGetHeadIndex(const Basalt_Pool *pool);
uint32_t basalt_poolGetTailIndex(const Basalt_Pool *pool);
uint32_t basalt_poolGetNextIndex(const Basalt_Pool *pool, uint32_t index);
uint32_t basalt_poolGetPrevIndex(const Basalt_Pool *pool, uint32_t index);
