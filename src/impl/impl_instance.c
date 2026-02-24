#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>

/*
 */
Basalt_Result impl_instanceDestroy(Basalt_Instance this)
{
	assert(this);

	Impl_Instance *ptr = (Impl_Instance *)this;

	free(ptr);
	return BASALT_SUCCESS;
}

/*
 */
static Basalt_InstanceTable instance_vtbl =
{
	impl_instanceDestroy,
};

/*
 */
Basalt_Result impl_createInstance(const Basalt_InstanceDesc *desc, Basalt_Instance *instance)
{
	assert(desc);
	assert(instance);

	BASALT_UNUSED(desc);

	Impl_Instance *ptr = (Impl_Instance *)malloc(sizeof(Impl_Instance));
	assert(ptr);

	// vtable
	ptr->vtbl = &instance_vtbl;

	// data

	// pools

	*instance = (Basalt_Instance)ptr;
	return BASALT_SUCCESS;
}
