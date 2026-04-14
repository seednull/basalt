#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/*
 */
static BASALT_INLINE float basalt_floatMin(float a, float b)
{
	return a < b ? a : b;
}

static BASALT_INLINE float basalt_floatMax(float a, float b)
{
	return a > b ? a : b;
}

static BASALT_INLINE float basalt_floatClamp(float value, float min_val, float max_val)
{
	return value < min_val ? min_val : (value > max_val ? max_val : value);
}

static BASALT_INLINE float basalt_vec3Dot(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
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

static BASALT_INLINE Basalt_Vec3 basalt_vec3Normalize(Basalt_Vec3 v)
{
	float l_rcp = 1.0f / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

	return (Basalt_Vec3)
	{
		v.x * l_rcp,
		v.y * l_rcp,
		v.z * l_rcp
	};
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
static BASALT_INLINE Basalt_Result basalt_spherePointIntersection(const Basalt_ShapeDataSphere *sphere, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	assert(sphere);
	assert(manifold);

	Basalt_Vec3 normal = basalt_vec3Sub(point, sphere->center);
	float d_sqr = basalt_vec3Dot(normal, normal);

	if (d_sqr > sphere->radius * sphere->radius)
		return BASALT_NO_INTERSECTION;

	float d = sqrtf(d_sqr);
	float penetration = d - sphere->radius;

	float inv_d = 1.0f / d;
	normal.x *= inv_d;
	normal.y *= inv_d;
	normal.z *= inv_d;

	manifold->normal = normal;
	manifold->num_contacts = 1;
	manifold->contacts[0].position = point;
	manifold->contacts[0].penetration = penetration;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_capsulePointIntersection(const Basalt_ShapeDataCapsule *capsule, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	assert(capsule);
	assert(manifold);

	float half_height = capsule->height * 0.5f;
	Basalt_Vec3 diff = basalt_vec3Sub(point, capsule->center);

	float p[3] = {diff.x, diff.y, diff.z};
	float proj = basalt_floatClamp(p[capsule->axis], -half_height, half_height);
	p[capsule->axis] -= proj;

	Basalt_Vec3 normal = {p[0], p[1], p[2]};
	float d_sqr = basalt_vec3Dot(normal, normal);

	if (d_sqr > capsule->radius * capsule->radius)
		return BASALT_NO_INTERSECTION;

	float d = sqrtf(d_sqr);
	float penetration = d - capsule->radius;

	float inv_d = 1.0f / d;
	normal.x *= inv_d;
	normal.y *= inv_d;
	normal.z *= inv_d;

	manifold->normal = normal;
	manifold->num_contacts = 1;
	manifold->contacts[0].position = point;
	manifold->contacts[0].penetration = penetration;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, 0};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxPointIntersection(const Basalt_ShapeDataBox *box, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	assert(box);
	assert(manifold);

	static Basalt_Vec3 local_normals[6] =
	{
		{-1.0f,  0.0f,  0.0f},
		{ 1.0f,  0.0f,  0.0f},
		{ 0.0f, -1.0f,  0.0f},
		{ 0.0f,  1.0f,  0.0f},
		{ 0.0f,  0.0f, -1.0f},
		{ 0.0f,  0.0f,  1.0f},
	};

	Basalt_Vec3 diff = basalt_vec3Sub(point, box->center);
	
	float distance_x = fabsf(diff.x) - box->sizes.x * 0.5f;
	float distance_y = fabsf(diff.y) - box->sizes.y * 0.5f;
	float distance_z = fabsf(diff.z) - box->sizes.z * 0.5f;

	float penetration = distance_x;
	uint32_t face = (diff.x < 0.0f) ? 0 : 1;

	if (penetration < distance_y)
	{
		face = (diff.y < 0.0f) ? 2 : 3;
		penetration = distance_y;
	}

	if (penetration < distance_z)
	{
		face = (diff.z < 0.0f) ? 4 : 5;
		penetration = distance_z;
	}

	if (penetration > 0.0f)
		return BASALT_NO_INTERSECTION;

	manifold->normal = local_normals[face];
	manifold->num_contacts = 1;
	manifold->contacts[0].position = point;
	manifold->contacts[0].penetration = penetration;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, face};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};

	return BASALT_SUCCESS;
}

/*
 */
static BASALT_INLINE Basalt_Result basalt_sphereRaycast(const Basalt_ShapeDataSphere *sphere, Basalt_Ray ray, Basalt_RayHit *hit)
{
	assert(sphere);
	assert(hit);

	Basalt_Vec3 origin = basalt_vec3Sub(ray.origin, sphere->center);
	float dot_od = basalt_vec3Dot(origin, ray.direction);
	float dot_oo = basalt_vec3Dot(origin, origin);
	float dot_dd = basalt_vec3Dot(ray.direction, ray.direction);

	float r_sqr = sphere->radius * sphere->radius;

	float d = dot_od * dot_od - dot_dd * (dot_oo - r_sqr);
	if (d < 0.0f)
		return BASALT_NO_INTERSECTION;

	float d_sqrt = sqrtf(d);

	assert(dot_dd > 0.0f);
	float dot_dd_rcp = 1.0f / dot_dd;

	float results[2] = {0};
	results[0] = (-dot_od - d_sqrt) * dot_dd_rcp;
	results[1] = (-dot_od + d_sqrt) * dot_dd_rcp;

	float distance = FLT_MAX;
	uint32_t num_valid_results = 0;

	for (uint32_t i = 0; i < 2; ++i)
	{
		if (results[i] < 0.0f)
			continue;

		num_valid_results++;
		distance = basalt_floatMin(distance, results[i]);
	}

	if (num_valid_results == 0)
		return BASALT_NO_INTERSECTION;

	Basalt_Vec3 local_point = basalt_vec3Mad(ray.direction, distance, ray.origin);
	Basalt_Vec3 local_normal = basalt_vec3Normalize(basalt_vec3Sub(local_point, sphere->center));

	hit->point = local_point;
	hit->normal = local_normal;
	hit->feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	hit->distance = distance;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_capsuleRaycast(const Basalt_ShapeDataCapsule *capsule, Basalt_Ray ray, Basalt_RayHit *hit)
{
	assert(capsule);
	assert(hit);

	static Basalt_Vec3 capsule_axes[3] =
	{
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};

	Basalt_Vec3 axis = capsule_axes[capsule->axis];
	float radius_sqr = capsule->radius * capsule->radius;
	float half_height = capsule->height * 0.5f;

	float distance = FLT_MAX;

	uint32_t feature = UINT32_MAX;
	Basalt_Vec3 origin = basalt_vec3Sub(ray.origin, capsule->center);

	// spheres
	float sphere_offsets[2] = {-half_height, half_height};
	for (uint32_t sphere_id = 0; sphere_id < 2; ++sphere_id)
	{
		Basalt_Vec3 sphere_origin = basalt_vec3Mad(axis, sphere_offsets[sphere_id], origin);

		float dot_od = basalt_vec3Dot(sphere_origin, ray.direction);
		float dot_oo = basalt_vec3Dot(sphere_origin, sphere_origin);
		float dot_dd = basalt_vec3Dot(ray.direction, ray.direction);

		float d = dot_od * dot_od - dot_dd * (dot_oo - radius_sqr);
		if (d < 0.0f)
			continue;

		float d_sqrt = sqrtf(d);

		assert(dot_dd > 0.0f);
		float dot_dd_rcp = 1.0f / dot_dd;

		float candidates[2] = {0};
		candidates[0] = (-dot_od - d_sqrt) * dot_dd_rcp;
		candidates[1] = (-dot_od + d_sqrt) * dot_dd_rcp;

		for (uint32_t i = 0; i < 2; ++i)
		{
			float candidate = candidates[i];
			if (candidate < 0.0f)
				continue;

			Basalt_Vec3 test = basalt_vec3Mad(ray.direction, candidate, origin);
			float projection = basalt_vec3Dot(test, axis);
			if (fabs(projection) < half_height)
				continue;

			if (distance > candidate)
			{
				distance = candidate;
				feature = sphere_id;
			}
		}
	}

	// cylinder
	do {
		Basalt_Vec3 cylinder_origin = origin;
		Basalt_Vec3 direction = ray.direction;

		switch (capsule->axis)
		{
			case BASALT_CAPSULE_AXIS_X: cylinder_origin.x = 0.0f; direction.x = 0.0f; break;
			case BASALT_CAPSULE_AXIS_Y: cylinder_origin.y = 0.0f; direction.y = 0.0f; break;
			case BASALT_CAPSULE_AXIS_Z: cylinder_origin.z = 0.0f; direction.z = 0.0f; break;
		}

		float dot_od = basalt_vec3Dot(cylinder_origin, direction);
		float dot_oo = basalt_vec3Dot(cylinder_origin, cylinder_origin);
		float dot_dd = basalt_vec3Dot(direction, direction);

		float d = dot_od * dot_od - dot_dd * (dot_oo - radius_sqr);
		if (d < 0.0f)
			continue;

		float d_sqrt = sqrtf(d);

		assert(dot_dd > 0.0f);
		float dot_dd_rcp = 1.0f / dot_dd;

		float intersections[2] = {0};
		intersections[0] = (-dot_od - d_sqrt) * dot_dd_rcp;
		intersections[1] = (-dot_od + d_sqrt) * dot_dd_rcp;

		for (uint32_t i = 0; i < 2; ++i)
		{
			float candidate = intersections[i];
			if (candidate < 0.0f)
				continue;

			Basalt_Vec3 test = basalt_vec3Mad(ray.direction, candidate, origin);
			float projection = basalt_vec3Dot(test, axis);
			if (fabs(projection) > half_height)
				continue;

			if (distance > candidate)
			{
				distance = candidate;
				feature = 2;
			}
		}
	} while(0);

	if (distance == FLT_MAX)
		return BASALT_NO_INTERSECTION;

	Basalt_Vec3 local_point = basalt_vec3Mad(ray.direction, distance, ray.origin);

	float projection = basalt_vec3Dot(local_point, axis);
	float center_projection = basalt_vec3Dot(capsule->center, axis);

	projection = basalt_floatClamp(projection - center_projection, -half_height, half_height);
	Basalt_Vec3 projected_point = basalt_vec3Mad(axis, projection, capsule->center);

	Basalt_Vec3 local_normal = basalt_vec3Sub(local_point, projected_point);

	hit->point = local_point;
	hit->normal = basalt_vec3Normalize(local_normal);
	hit->feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, feature};
	hit->distance = distance;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxRaycast(const Basalt_ShapeDataBox *box, Basalt_Ray ray, Basalt_RayHit *hit)
{
	assert(box);
	assert(hit);

	float tmin = -FLT_MAX;
	float tmax = FLT_MAX;

	Basalt_Vec3 box_min = basalt_vec3Mad(box->sizes, -0.5f, box->center);
	Basalt_Vec3 box_max = basalt_vec3Mad(box->sizes, 0.5f, box->center);

	uint32_t fmin = UINT32_MAX;
	uint32_t fmax = UINT32_MAX;

	float d_rcp[3] =
	{
		1.0f / ray.direction.x,
		1.0f / ray.direction.y,
		1.0f / ray.direction.z,
	};

	float results[6] =
	{
		(box_min.x - ray.origin.x) * d_rcp[0], // -x
		(box_max.x - ray.origin.x) * d_rcp[0], // +x
		(box_min.y - ray.origin.y) * d_rcp[1], // -y
		(box_max.y - ray.origin.y) * d_rcp[1], // +y
		(box_min.z - ray.origin.z) * d_rcp[2], // -z
		(box_max.z - ray.origin.z) * d_rcp[2], // +z
	};

	static Basalt_Vec3 local_normals[6] =
	{
		{-1.0f,  0.0f,  0.0f},
		{ 1.0f,  0.0f,  0.0f},
		{ 0.0f, -1.0f,  0.0f},
		{ 0.0f,  1.0f,  0.0f},
		{ 0.0f,  0.0f, -1.0f},
		{ 0.0f,  0.0f,  1.0f},
	};

	for (uint32_t i = 0; i < 3; ++i)
	{
		float tcmin = FLT_MAX;
		float tcmax = -FLT_MAX;
		uint32_t fcmin = UINT32_MAX;
		uint32_t fcmax = UINT32_MAX;

		for (uint32_t j = 0; j < 2; ++j)
		{
			float result = results[2 * i + j];

			if (tcmin > result)
			{
				fcmin = 2 * i + j;
				tcmin = result;
			}

			if (tcmax < result)
			{
				fcmax = 2 * i + j;
				tcmax = result;
			}
		}

		assert(tcmin != FLT_MAX);
		assert(tcmax != -FLT_MAX);
		assert(fcmin != UINT32_MAX);
		assert(fcmax != UINT32_MAX);

		if (tmin < tcmin)
		{
			tmin = tcmin;
			fmin = fcmin;
		}

		if (tmax > tcmax)
		{
			tmax = tcmax;
			fmax = fcmax;
		}
	}

	if (tmin > tmax || tmax < 0.0f)
		return BASALT_NO_INTERSECTION;

	float distance = (tmin < 0.0f) ? tmax : tmin;
	uint32_t face = (tmin < 0.0f) ? fmax : fmin;

	Basalt_Vec3 local_point = basalt_vec3Mad(ray.direction, distance, ray.origin);

	hit->point = local_point;
	hit->normal = local_normals[face];
	hit->feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, face};
	hit->distance = distance;

	return BASALT_SUCCESS;
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

	Basalt_Result result = BASALT_SUCCESS;
	switch (shape_ptr->info.type)
	{
		case BASALT_SHAPE_TYPE_SPHERE:
		{
			Basalt_ShapeDataSphere *sphere = &shape_ptr->info.data.sphere;
			result = basalt_spherePointIntersection(sphere, local_point, manifold);
		}
		break;

		case BASALT_SHAPE_TYPE_CAPSULE:
		{
			Basalt_ShapeDataCapsule *capsule = &shape_ptr->info.data.capsule;
			result = basalt_capsulePointIntersection(capsule, local_point, manifold);
		}
		break;

		case BASALT_SHAPE_TYPE_BOX:
		{
			Basalt_ShapeDataBox *box = &shape_ptr->info.data.box;
			result = basalt_boxPointIntersection(box, local_point, manifold);
		}
		break;

		default:
			return BASALT_NOT_IMPLEMENTED;
	}

	if (result != BASALT_SUCCESS)
		return result;

	assert(manifold->num_contacts == 1);

	manifold->normal = basalt_quatRotateVec3(transform.rotation, manifold->normal);
	manifold->contacts[0].position = point;

	return BASALT_SUCCESS;
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
	assert(this);
	assert(shape);
	assert(hit);

	Basalt_PoolHandle handle = (Basalt_PoolHandle)shape;
	assert(handle != BASALT_POOL_HANDLE_NULL);

	Basalt_Quat rotation_inv = basalt_quatConjugate(transform.rotation);
	Basalt_Ray local_ray = (Basalt_Ray)
	{
		basalt_quatRotateVec3(rotation_inv, basalt_vec3Sub(ray.origin, transform.position)),
		basalt_quatRotateVec3(rotation_inv, ray.direction),
	};

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, handle);
	assert(shape_ptr);

	Basalt_Result result = BASALT_SUCCESS;

	switch (shape_ptr->info.type)
	{
		case BASALT_SHAPE_TYPE_SPHERE:
		{
			const Basalt_ShapeDataSphere *sphere = &shape_ptr->info.data.sphere;
			result = basalt_sphereRaycast(sphere, local_ray, hit);
		}
		break;

		case BASALT_SHAPE_TYPE_CAPSULE:
		{
			const Basalt_ShapeDataCapsule *capsule = &shape_ptr->info.data.capsule;
			result = basalt_capsuleRaycast(capsule, local_ray, hit);
		}
		break;

		case BASALT_SHAPE_TYPE_BOX:
		{
			const Basalt_ShapeDataBox *box = &shape_ptr->info.data.box;
			result = basalt_boxRaycast(box, local_ray, hit);
		}
		break;

		default:
			return BASALT_NOT_IMPLEMENTED;
	}

	if (result != BASALT_SUCCESS)
		return result;

	hit->point = basalt_vec3Mad(ray.direction, hit->distance, ray.origin);
	hit->normal = basalt_quatRotateVec3(transform.rotation, hit->normal);

	return BASALT_SUCCESS;
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
