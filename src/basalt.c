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
Basalt_Result basaltCreateShapeSphere(Basalt_Instance instance, Basalt_Vec3 center, float radius, Basalt_Shape *shape)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createShapeSphere);

	return ptr->vtbl->createShapeSphere(instance, center, radius, shape);
}

Basalt_Result basaltCreateShapeCapsule(Basalt_Instance instance, Basalt_Vec3 center, float radius, float height, Basalt_CapsuleAxis axis, Basalt_Shape *shape)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createShapeCapsule);

	return ptr->vtbl->createShapeCapsule(instance, center, radius, height, axis, shape);
}

Basalt_Result basaltCreateShapeBox(Basalt_Instance instance, Basalt_Vec3 center, Basalt_Vec3 sizes, Basalt_Shape *shape)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->createShapeBox);

	return ptr->vtbl->createShapeBox(instance, center, sizes, shape);
}

Basalt_Result basaltGetPenetration(Basalt_Instance instance, Basalt_Shape shape, Basalt_Vec3 point, Basalt_Vec4 *penetration)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->getPenetration);

	return ptr->vtbl->getPenetration(instance, shape, point, penetration);
}

Basalt_Result basaltDestroyShape(Basalt_Instance instance, Basalt_Shape shape)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyShape);

	return ptr->vtbl->destroyShape(instance, shape);
}

Basalt_Result basaltDestroyInstance(Basalt_Instance instance)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->destroyInstance);

	return ptr->vtbl->destroyInstance(instance);
}
