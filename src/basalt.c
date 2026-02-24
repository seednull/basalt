#include "basalt_internal.h"

#include <assert.h>
#include <string.h>

/*
 */
typedef struct Basalt_InstanceInternal_t
{
	Basalt_InstanceTable *vtbl;
} Basalt_InstanceInternal;

/*
 */
Basalt_Result basaltCreateInstance(const Basalt_InstanceDesc *desc, Basalt_Instance *instance)
{
	return impl_createInstance(desc, instance);
}

Basalt_Result basaltGetInstanceTable(Basalt_Instance instance, Basalt_InstanceTable *instance_table)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	if (instance_table == NULL)
		return BASALT_INVALID_OUTPUT_ARGUMENT;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);

	memcpy(instance_table, ptr->vtbl, sizeof(Basalt_InstanceTable));
	return BASALT_SUCCESS;
}

/*
 */
Basalt_Result basaltDestroyInstance(Basalt_Instance instance)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyInstance);

	return ptr->vtbl->destroyInstance(instance);
}
