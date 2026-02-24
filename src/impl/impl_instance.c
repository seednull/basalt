#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*
 */
static BASALT_INLINE float basalt_floatClamp(float value, float min_val, float max_val)
{
	return value < min_val ? min_val : (value > max_val ? max_val : value);
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Mad(Basalt_Vec3 a, float s, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.x * s + b.x,
		a.y * s + b.y,
		a.z * s + b.z
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Add(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.x + b.x,
		a.y + b.y,
		a.z + b.z
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Sub(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.x - b.x,
		a.y - b.y,
		a.z - b.z
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Mul(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.x * b.x,
		a.y * b.y,
		a.z * b.z
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Div(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.x / b.x,
		a.y / b.y,
		a.z / b.z
	};
}

static BASALT_INLINE float basalt_vec3Dot(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
 */
static void impl_destroyShape(Impl_Instance *instance_ptr, Impl_Shape *shape_ptr)
{
	assert(instance_ptr);
	assert(shape_ptr);

	BASALT_UNUSED(instance_ptr);
	BASALT_UNUSED(shape_ptr);
}

/*
 */
Basalt_Result impl_instanceCreateShapeSphere(Basalt_Instance this, Basalt_Vec3 center, float radius, Basalt_Shape *shape)
{
	assert(this);
	assert(shape);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Shape result = {0};
	result.type = BASALT_SHAPE_TYPE_SPHERE;
	result.data.sphere = (Impl_ShapeSphere){center, radius};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceCreateShapeCapsule(Basalt_Instance this, Basalt_Vec3 center, float radius, float height, Basalt_CapsuleAxis axis, Basalt_Shape *shape)
{
	assert(this);
	assert(shape);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Shape result = {0};
	result.type = BASALT_SHAPE_TYPE_CAPSULE;
	result.data.capsule = (Impl_ShapeCapsule){center, radius, height, axis};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceCreateShapeBox(Basalt_Instance this, Basalt_Vec3 center, Basalt_Vec3 sizes, Basalt_Shape *shape)
{
	assert(this);
	assert(shape);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Shape result = {0};
	result.type = BASALT_SHAPE_TYPE_BOX;
	result.data.box = (Impl_ShapeBox){center, sizes};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceGetPenetration(Basalt_Instance this, Basalt_Shape shape, Basalt_Vec3 point, Basalt_Vec4 *penetration)
{
	assert(this);
	assert(shape);
	assert(penetration);

	Basalt_PoolHandle handle = (Basalt_PoolHandle)shape;
	assert(handle != BASALT_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, handle);
	assert(shape_ptr);

	switch (shape_ptr->type)
	{
		case BASALT_SHAPE_TYPE_SPHERE:
		{
			Impl_ShapeSphere *sphere = &shape_ptr->data.sphere;
			assert(sphere);

			Basalt_Vec3 normal = basalt_vec3Sub(point, sphere->center);
			float d = sqrtf(basalt_vec3Dot(normal, normal));
			float inv_d = 1.0f / d;

			penetration->x = normal.x * inv_d;
			penetration->y = normal.y * inv_d;
			penetration->z = normal.z * inv_d;
			penetration->w = d - sphere->radius;

			return BASALT_SUCCESS;
		}

		case BASALT_SHAPE_TYPE_CAPSULE:
		{
			Impl_ShapeCapsule *capsule = &shape_ptr->data.capsule;
			assert(capsule);

			float p[3] = {point.x, point.y, point.z};
			float half_height = capsule->height * 0.5f;
			float proj = basalt_floatClamp(p[capsule->axis], -half_height, half_height);

			float c[3] = {0.0f, 0.0f, 0.0f};
			c[capsule->axis] = proj;

			Basalt_Vec3 center = {c[0], c[1], c[2]};
			Basalt_Vec3 normal = basalt_vec3Sub(point, center);
			float d = sqrtf(basalt_vec3Dot(normal, normal));
			float inv_d = 1.0f / d;

			penetration->x = normal.x * inv_d;
			penetration->y = normal.y * inv_d;
			penetration->z = normal.z * inv_d;
			penetration->w = d - capsule->radius;

			return BASALT_SUCCESS;
		}
		break;

		case BASALT_SHAPE_TYPE_BOX:
		{
			Impl_ShapeBox *box = &shape_ptr->data.box;
			assert(box);

			Basalt_Vec3 diff = basalt_vec3Sub(point, box->center);
			
			float distance_x = fabsf(diff.x) - box->sizes.x * 0.5f;
			float distance_y = fabsf(diff.y) - box->sizes.y * 0.5f;
			float distance_z = fabsf(diff.z) - box->sizes.z * 0.5f;

			float closest_distance = distance_x;
			Basalt_Vec3 normal = (Basalt_Vec3){diff.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f};

			if (fabsf(closest_distance) < fabsf(distance_y))
			{
				closest_distance = distance_y;
				normal = (Basalt_Vec3){0.0f, diff.y < 0.0f ? -1.0f : 1.0f, 0.0f};
			}

			if (fabsf(closest_distance) < fabsf(distance_z))
			{
				closest_distance = distance_z;
				normal = (Basalt_Vec3){0.0f, 0.0f, diff.y < 0.0f ? -1.0f : 1.0f};
			}
			
			penetration->x = normal.x;
			penetration->y = normal.y;
			penetration->z = normal.z;
			penetration->w = closest_distance;

			return BASALT_SUCCESS;
		}
	}

	return BASALT_NOT_IMPLEMENTED;
}

Basalt_Result impl_instanceDestroyShape(Basalt_Instance this, Basalt_Shape shape)
{
	assert(this);
	assert(shape);

	Basalt_PoolHandle handle = (Basalt_PoolHandle)shape;
	assert(handle != BASALT_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, handle);
	assert(shape_ptr);

	basalt_poolRemoveElement(&instance_ptr->shapes, handle);

	impl_destroyShape(instance_ptr, shape_ptr);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceDestroy(Basalt_Instance this)
{
	assert(this);

	Impl_Instance *ptr = (Impl_Instance *)this;

	{
		uint32_t head = basalt_poolGetHeadIndex(&ptr->shapes);
		while (head != BASALT_POOL_HANDLE_NULL)
		{
			Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElementByIndex(&ptr->shapes, head);
			impl_destroyShape(ptr, shape_ptr);

			head = basalt_poolGetNextIndex(&ptr->shapes, head);
		}

		basalt_poolShutdown(&ptr->shapes);
	}

	free(ptr);
	return BASALT_SUCCESS;
}

/*
 */
static Basalt_InstanceTable instance_vtbl =
{
	impl_instanceCreateShapeSphere,
	impl_instanceCreateShapeCapsule,
	impl_instanceCreateShapeBox,

	impl_instanceGetPenetration,

	impl_instanceDestroyShape,
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
	basalt_poolInitialize(&ptr->shapes, sizeof(Impl_Shape), 32);

	*instance = (Basalt_Instance)ptr;
	return BASALT_SUCCESS;
}
