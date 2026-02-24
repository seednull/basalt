#include "pool.h"
#include "intrinsics.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>

/*
 */
static BASALT_INLINE Basalt_PoolHandle basalt_poolHandlePack(uint32_t index, uint8_t generation)
{
	return (Basalt_PoolHandle)((index << 8) | generation);
}

static BASALT_INLINE uint32_t basalt_poolHandleGetIndex(Basalt_PoolHandle handle)
{
	return (uint32_t)(handle >> 8);
}

static BASALT_INLINE uint8_t basalt_poolHandleGetGeneration(Basalt_PoolHandle handle)
{
	return (uint8_t)(handle & 0xFF);
}

/*
 */
static BASALT_INLINE uint32_t basalt_poolGrabIndex(Basalt_Pool *pool)
{
	assert(pool);
	assert(pool->num_free_indices > 0);

	uint32_t index = pool->indices[pool->num_free_indices - 1];
	pool->num_free_indices--;

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	pool->masks[mask_index] &= ~(1 << mask_bit);

	return index;
}

static BASALT_INLINE void basalt_poolReleaseIndex(Basalt_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->num_free_indices < pool->capacity);
	assert(index < pool->capacity);

	pool->num_free_indices++;
	pool->indices[pool->num_free_indices - 1] = index;

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	pool->masks[mask_index] |= 1 << mask_bit;
}

static BASALT_INLINE uint32_t basalt_poolIsIndexFree(const Basalt_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(index < pool->capacity);

	uint32_t mask_index = index / 32;
	uint32_t mask_bit = index % 32;

	uint32_t free_mask = pool->masks[mask_index];
	uint32_t element_mask = 1 << mask_bit;

	return free_mask & element_mask;
}

static BASALT_INLINE uint32_t basalt_poolGetNumMasks(const Basalt_Pool *pool)
{
	assert(pool);

	return alignUp(pool->capacity, 32) / 32;
}

/*
 */
Basalt_Result basalt_poolInitialize(Basalt_Pool *pool, uint32_t element_size, uint32_t capacity)
{
	assert(pool);
	assert(element_size > 0);

	memset(pool, 0, sizeof(Basalt_Pool));

	pool->element_size = element_size;
	pool->capacity = capacity;
	pool->num_free_indices = capacity;
	pool->head = BASALT_POOL_HANDLE_NULL;
	pool->tail = BASALT_POOL_HANDLE_NULL;

	if (capacity > 0)
	{
		uint32_t num_masks = basalt_poolGetNumMasks(pool);

		pool->data = (uint8_t *)malloc(element_size * capacity);
		pool->generations = (uint8_t *)malloc(sizeof(uint8_t) * capacity);
		pool->nexts = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->prevs = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->indices = (uint32_t *)malloc(sizeof(uint32_t) * capacity);
		pool->masks = (uint32_t *)malloc(sizeof(uint32_t) * num_masks);

		for (uint32_t i = 0; i < capacity; ++i)
			pool->indices[i] = capacity - i - 1;

		memset(pool->nexts, BASALT_POOL_HANDLE_NULL, sizeof(uint32_t) * capacity);
		memset(pool->prevs, BASALT_POOL_HANDLE_NULL, sizeof(uint32_t) * capacity);
		memset(pool->masks, 0xFFFFFFFF, sizeof(uint32_t) * num_masks);
		memset(pool->generations, 0, sizeof(uint8_t) * capacity);
	}

	return BASALT_SUCCESS;
}

Basalt_Result basalt_poolShutdown(Basalt_Pool *pool)
{
	assert(pool);

	free(pool->data);
	free(pool->generations);
	free(pool->nexts);
	free(pool->prevs);
	free(pool->indices);
	free(pool->masks);

	memset(pool, 0, sizeof(Basalt_Pool));

	return BASALT_SUCCESS;
}

/*
 */
Basalt_PoolHandle basalt_poolAddElement(Basalt_Pool *pool, const void *data)
{
	assert(pool);
	assert(data);

	if (pool->size == pool->capacity)
	{
		uint32_t old_capacity = pool->capacity;
		uint32_t old_num_masks = basalt_poolGetNumMasks(pool);

		pool->capacity = (pool->capacity == 0) ? 1 : pool->capacity * 2;
		uint32_t new_num_masks = basalt_poolGetNumMasks(pool);

		pool->data = (uint8_t *)realloc(pool->data, pool->element_size * pool->capacity);
		pool->generations = (uint8_t *)realloc(pool->generations, sizeof(uint8_t) * pool->capacity);
		pool->nexts = (uint32_t *)realloc(pool->nexts, sizeof(uint32_t) * pool->capacity);
		pool->prevs = (uint32_t *)realloc(pool->prevs, sizeof(uint32_t) * pool->capacity);
		pool->indices = (uint32_t *)realloc(pool->indices, sizeof(uint32_t) * pool->capacity);

		if (old_num_masks != new_num_masks)
			pool->masks = (uint32_t *)realloc(pool->masks, sizeof(uint32_t) * new_num_masks);

		for (uint32_t i = old_capacity; i < pool->capacity; ++i)
		{
			pool->num_free_indices++;
			pool->indices[pool->num_free_indices - 1] = old_capacity + pool->capacity - i - 1;
			pool->generations[i] = 0;
			pool->nexts[i] = BASALT_POOL_HANDLE_NULL;
			pool->prevs[i] = BASALT_POOL_HANDLE_NULL;
		}

		for (uint32_t i = old_num_masks; i < new_num_masks; ++i)
			pool->masks[i] = 0xFFFFFFFF;
	}

	assert(pool->num_free_indices > 0);

	uint32_t index = basalt_poolGrabIndex(pool);

	uint8_t *data_ptr = pool->data + index * pool->element_size;
	uint8_t *generation_ptr = pool->generations + index;

	if (pool->head == BASALT_POOL_HANDLE_NULL)
		pool->head = index;

	if (pool->tail == BASALT_POOL_HANDLE_NULL)
	{
		pool->tail = index;
	}
	else
	{
		pool->nexts[pool->tail] = index;
		pool->prevs[index] = pool->tail;

		pool->tail = index;
	}

	memcpy(data_ptr, data, pool->element_size);

	uint8_t generation = *generation_ptr + 1;
	*generation_ptr = max(1, generation);

	pool->size++;

	return basalt_poolHandlePack(index, *generation_ptr);
}

Basalt_Result basalt_poolRemoveElement(Basalt_Pool *pool, Basalt_PoolHandle handle)
{
	assert(pool);

	if (handle == BASALT_POOL_HANDLE_NULL)
		return BASALT_INTERNAL_ERROR;

	if (pool->size == 0)
		return BASALT_INTERNAL_ERROR;

	uint32_t index = basalt_poolHandleGetIndex(handle);
	uint8_t generation = basalt_poolHandleGetGeneration(handle);

	if (pool->generations[index] != generation)
		return BASALT_INTERNAL_ERROR;

	if (basalt_poolIsIndexFree(pool, index))
		return BASALT_INTERNAL_ERROR;

	uint32_t prev = pool->prevs[index];
	uint32_t next = pool->nexts[index];

	pool->prevs[index] = BASALT_POOL_HANDLE_NULL;
	pool->nexts[index] = BASALT_POOL_HANDLE_NULL;

	if (next != BASALT_POOL_HANDLE_NULL)
		pool->prevs[next] = prev;

	if (prev != BASALT_POOL_HANDLE_NULL)
		pool->nexts[prev] = next;

	if (pool->head == index)
		pool->head = next;

	if (pool->tail == index)
		pool->tail = prev;

	basalt_poolReleaseIndex(pool, index);
	pool->size--;

	return BASALT_SUCCESS;
}

void *basalt_poolGetElement(const Basalt_Pool *pool, Basalt_PoolHandle handle)
{
	assert(pool);
	assert(pool->size > 0);

	if (handle == BASALT_POOL_HANDLE_NULL)
		return NULL;

	uint32_t index = basalt_poolHandleGetIndex(handle);
	uint8_t generation = basalt_poolHandleGetGeneration(handle);

	if (pool->generations[index] != generation)
		return NULL;

	if (basalt_poolIsIndexFree(pool, index))
		return NULL;

	return pool->data + index * pool->element_size;
}

void *basalt_poolGetElementByIndex(const Basalt_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != BASALT_POOL_HANDLE_NULL);

	return pool->data + index * pool->element_size;
}

uint32_t basalt_poolGetHeadIndex(const Basalt_Pool *pool)
{
	assert(pool);
	return pool->head;
}

uint32_t basalt_poolGetTailIndex(const Basalt_Pool *pool)
{
	assert(pool);
	return pool->tail;
}

uint32_t basalt_poolGetNextIndex(const Basalt_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != BASALT_POOL_HANDLE_NULL);
	
	return pool->nexts[index];
}

uint32_t basalt_poolGetPrevIndex(const Basalt_Pool *pool, uint32_t index)
{
	assert(pool);
	assert(pool->size > 0);
	assert(pool->capacity > index);
	assert(index != BASALT_POOL_HANDLE_NULL);
	
	return pool->prevs[index];
}
