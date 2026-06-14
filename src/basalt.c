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
	return impl_basaltCreateInstance(desc, instance);
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

Basalt_Result basaltShapeGetInfo(Basalt_Instance instance, Basalt_Shape shape, Basalt_ShapeInfo *info)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->shapeGetInfo);

	return ptr->vtbl->shapeGetInfo(instance, shape, info);
}

Basalt_Result basaltShapeIntersectPoint(Basalt_Instance instance, Basalt_Shape shape, Basalt_Transform transform, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->shapeIntersectPoint);

	return ptr->vtbl->shapeIntersectPoint(instance, shape, transform, point, manifold);
}

Basalt_Result basaltShapeIntersectShape(Basalt_Instance instance, Basalt_Shape shape_a, Basalt_Transform transform_a, Basalt_Shape shape_b, Basalt_Transform transform_b, Basalt_ContactManifold *manifold)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->shapeIntersectShape);

	return ptr->vtbl->shapeIntersectShape(instance, shape_a, transform_a, shape_b, transform_b, manifold);
}

Basalt_Result basaltShapeRaycast(Basalt_Instance instance, Basalt_Shape shape, Basalt_Transform transform, Basalt_Ray ray, Basalt_RayHit *hit)
{
	if (instance == BASALT_NULL_HANDLE)
		return BASALT_INVALID_INSTANCE;

	Basalt_InstanceInternal *ptr = (Basalt_InstanceInternal *)instance;
	assert(ptr->vtbl);
	assert(ptr->vtbl->shapeRaycast);

	return ptr->vtbl->shapeRaycast(instance, shape, transform, ray, hit);
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
