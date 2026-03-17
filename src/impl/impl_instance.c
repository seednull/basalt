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

static BASALT_INLINE Basalt_Quat basalt_quatConjugate(Basalt_Quat q)
{
	return (Basalt_Quat)
	{
		-q.x,
		-q.y,
		-q.z,
		 q.w,
	};
}

static BASALT_INLINE Basalt_Quat basalt_quatMul(Basalt_Quat a, Basalt_Quat b)
{
	return (Basalt_Quat)
	{
		// linear combination + cross product
		a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y,
		a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z,
		a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x,

		// mul                - dot product
		a.w * b.w             - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static BASALT_INLINE Basalt_Quat basalt_quatMulVec3(Basalt_Quat a, Basalt_Vec3 b)
{
	return (Basalt_Quat)
	{
		// linear combination + cross product
		a.w * b.x             + a.y * b.z - a.z * b.y,
		a.w * b.y             + a.z * b.x - a.x * b.z,
		a.w * b.z             + a.x * b.y - a.y * b.x,

		//                    - dot product
		                      - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_quatRotateVec3(Basalt_Quat a, Basalt_Vec3 v)
{
	Basalt_Quat t = basalt_quatMulVec3(a, v);
	t = basalt_quatMul(t, basalt_quatConjugate(a));

	return (Basalt_Vec3)
	{
		t.x,
		t.y,
		t.z
	};
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
	result.info.type = BASALT_SHAPE_TYPE_SPHERE;
	result.info.data.sphere = (Basalt_ShapeDataSphere){center, radius};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceCreateShapeCapsule(Basalt_Instance this, Basalt_Vec3 center, float radius, float height, Basalt_CapsuleAxis axis, Basalt_Shape *shape)
{
	assert(this);
	assert(shape);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Shape result = {0};
	result.info.type = BASALT_SHAPE_TYPE_CAPSULE;
	result.info.data.capsule = (Basalt_ShapeDataCapsule){center, radius, height, axis};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceCreateShapeBox(Basalt_Instance this, Basalt_Vec3 center, Basalt_Vec3 sizes, Basalt_Shape *shape)
{
	assert(this);
	assert(shape);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;

	Impl_Shape result = {0};
	result.info.type = BASALT_SHAPE_TYPE_BOX;
	result.info.data.box = (Basalt_ShapeDataBox){center, sizes};
	
	*shape = (Basalt_Shape)basalt_poolAddElement(&instance_ptr->shapes, &result);
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceShapeGetInfo(Basalt_Instance this, Basalt_Shape shape, Basalt_ShapeInfo *info)
{
	assert(this);
	assert(shape);
	assert(info);

	Basalt_PoolHandle handle = (Basalt_PoolHandle)shape;
	assert(handle != BASALT_POOL_HANDLE_NULL);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, handle);
	assert(shape_ptr);

	memcpy(info, &shape_ptr->info, sizeof(Basalt_ShapeInfo));
	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceShapeIntersectPoint(Basalt_Instance this, Basalt_Shape shape, Basalt_Transform transform, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	assert(this);
	assert(shape);
	assert(manifold);

	Basalt_PoolHandle handle = (Basalt_PoolHandle)shape;
	assert(handle != BASALT_POOL_HANDLE_NULL);

	Basalt_Quat rotation_inv = basalt_quatConjugate(transform.rotation);
	Basalt_Vec3 local_point = basalt_quatRotateVec3(rotation_inv, basalt_vec3Sub(point, transform.position));

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, handle);
	assert(shape_ptr);

	switch (shape_ptr->info.type)
	{
		case BASALT_SHAPE_TYPE_SPHERE:
		{
			Basalt_ShapeDataSphere *sphere = &shape_ptr->info.data.sphere;
			assert(sphere);

			Basalt_Vec3 normal = basalt_vec3Sub(local_point, sphere->center);
			float d = sqrtf(basalt_vec3Dot(normal, normal));
			float penetration = d - sphere->radius;

			if (penetration > 0.0f)
				return BASALT_NO_INTERSECTION;

			float inv_d = 1.0f / d;
			normal.x *= inv_d;
			normal.y *= inv_d;
			normal.z *= inv_d;

			manifold->normal = basalt_quatRotateVec3(transform.rotation, normal);
			manifold->num_contacts = 1;
			manifold->contacts[0].position = point;
			manifold->contacts[0].penetration = penetration;
			manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
			manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

			return BASALT_SUCCESS;
		}

		case BASALT_SHAPE_TYPE_CAPSULE:
		{
			Basalt_ShapeDataCapsule *capsule = &shape_ptr->info.data.capsule;
			assert(capsule);

			float half_height = capsule->height * 0.5f;
			Basalt_Vec3 diff = basalt_vec3Sub(local_point, capsule->center);

			float p[3] = {diff.x, diff.y, diff.z};
			float proj = basalt_floatClamp(p[capsule->axis], -half_height, half_height);
			p[capsule->axis] -= proj;

			Basalt_Vec3 normal = {p[0], p[1], p[2]};
			float d = sqrtf(basalt_vec3Dot(normal, normal));
			float penetration = d - capsule->radius;

			if (penetration > 0.0f)
				return BASALT_NO_INTERSECTION;

			float inv_d = 1.0f / d;
			normal.x *= inv_d;
			normal.y *= inv_d;
			normal.z *= inv_d;

			manifold->normal = basalt_quatRotateVec3(transform.rotation, normal);
			manifold->num_contacts = 1;
			manifold->contacts[0].position = point;
			manifold->contacts[0].penetration = penetration;
			manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, 0};
			manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

			return BASALT_SUCCESS;
		}

		case BASALT_SHAPE_TYPE_BOX:
		{
			Basalt_ShapeDataBox *box = &shape_ptr->info.data.box;
			assert(box);

			Basalt_Vec3 diff = basalt_vec3Sub(local_point, box->center);
			
			float distance_x = fabsf(diff.x) - box->sizes.x * 0.5f;
			float distance_y = fabsf(diff.y) - box->sizes.y * 0.5f;
			float distance_z = fabsf(diff.z) - box->sizes.z * 0.5f;

			float penetration = distance_x;
			uint32_t face = (diff.x < 0.0f) ? 0 : 1;
			Basalt_Vec3 normal = (Basalt_Vec3){diff.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f};

			if (penetration < distance_y)
			{
				face = (diff.y < 0.0f) ? 2 : 3;
				penetration = distance_y;
				normal = (Basalt_Vec3){0.0f, diff.y < 0.0f ? -1.0f : 1.0f, 0.0f};
			}

			if (penetration < distance_z)
			{
				face = (diff.z < 0.0f) ? 4 : 5;
				penetration = distance_z;
				normal = (Basalt_Vec3){0.0f, 0.0f, diff.z < 0.0f ? -1.0f : 1.0f};
			}

			if (penetration > 0.0f)
				return BASALT_NO_INTERSECTION;

			manifold->normal = basalt_quatRotateVec3(transform.rotation, normal);
			manifold->num_contacts = 1;
			manifold->contacts[0].position = point;
			manifold->contacts[0].penetration = penetration;
			manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, face};
			manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

			return BASALT_SUCCESS;
		}
	}

	return BASALT_NOT_IMPLEMENTED;
}

Basalt_Result impl_instanceShapeIntersectShape(Basalt_Instance this, Basalt_Shape shape_a, Basalt_Transform transform_a, Basalt_Shape shape_b, Basalt_Transform transform_b, Basalt_ContactManifold *manifold)
{
	BASALT_UNUSED(this);
	BASALT_UNUSED(shape_a);
	BASALT_UNUSED(transform_a);
	BASALT_UNUSED(shape_b);
	BASALT_UNUSED(transform_b);
	BASALT_UNUSED(manifold);

	return BASALT_NOT_IMPLEMENTED;
}

Basalt_Result impl_instanceShapeRaycast(Basalt_Instance this, Basalt_Shape shape, Basalt_Transform transform, Basalt_Ray ray, Basalt_RayHit *hit)
{
	BASALT_UNUSED(this);
	BASALT_UNUSED(shape);
	BASALT_UNUSED(transform);
	BASALT_UNUSED(ray);
	BASALT_UNUSED(hit);

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

	impl_instanceShapeGetInfo,

	impl_instanceShapeIntersectPoint,
	impl_instanceShapeIntersectShape,
	impl_instanceShapeRaycast,

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
