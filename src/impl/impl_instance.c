#include "impl_internal.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/*
 */
typedef Basalt_Result (*PFN_basalt_shapeShapeIntersection)(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold);

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

static BASALT_INLINE void basalt_floatSwap(float *a, float *b)
{
	assert(a);
	assert(b);

	float temp = *a;
	*a = *b;
	*b = temp;
}

static BASALT_INLINE float basalt_floatClamp(float value, float min_val, float max_val)
{
	return value < min_val ? min_val : (value > max_val ? max_val : value);
}

static BASALT_INLINE float basalt_vec3Dot(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Lerp(Basalt_Vec3 a, Basalt_Vec3 b, float t)
{
	return (Basalt_Vec3)
	{
		a.x * (1.0f - t) + b.x * t,
		a.y * (1.0f - t) + b.y * t,
		a.z * (1.0f - t) + b.z * t,
	};
}

static BASALT_INLINE Basalt_Vec3 basalt_vec3Cross(Basalt_Vec3 a, Basalt_Vec3 b)
{
	return (Basalt_Vec3)
	{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
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

static BASALT_INLINE Basalt_Transform basalt_invertTransform(Basalt_Transform t)
{
	Basalt_Quat rotation = basalt_quatConjugate(t.rotation);
	Basalt_Vec3 position = basalt_quatRotateVec3(rotation, (Basalt_Vec3){-t.position.x, -t.position.y, -t.position.z});

	return (Basalt_Transform)
	{
		position,
		rotation,
	};
}

static BASALT_INLINE Basalt_Transform basalt_mulTransform(Basalt_Transform a, Basalt_Transform b)
{
	Basalt_Quat rotation = basalt_quatMul(a.rotation, b.rotation);
	Basalt_Vec3 position = basalt_vec3Add(a.position, basalt_quatRotateVec3(a.rotation, b.position));

	return (Basalt_Transform)
	{
		position,
		rotation,
	};
}

/*
 */
typedef struct Basalt_SegmentManifold_t
{
	float results_a[2];
	float results_b[2];
	uint32_t num_results;
} Basalt_SegmentManifold;

static BASALT_INLINE Basalt_SegmentManifold basalt_segmentSegmentIntersection(
	Basalt_Vec3 center_a,
	Basalt_Vec3 center_b,
	Basalt_Vec3 axis_a,
	Basalt_Vec3 axis_b,
	float half_height_a,
	float half_height_b
)
{
	Basalt_Vec3 center_diff = basalt_vec3Sub(center_b, center_a);

	float dot_ab = basalt_vec3Dot(axis_a, axis_b);
	float dot_ca = basalt_vec3Dot(center_diff, axis_a);
	float dot_cb = basalt_vec3Dot(center_diff, axis_b);

	float det = dot_ab * dot_ab - 1.0f;

	Basalt_SegmentManifold manifold =
	{
		FLT_MAX, FLT_MAX,
		FLT_MAX, FLT_MAX,
		0
	};

	// crossing case
	if (fabs(det) > 0.05f)
	{
		float det_inv = 1.0f / det;
		float t_a = (dot_ab * dot_cb - dot_ca) * det_inv;
		float t_b = (dot_cb - dot_ab * dot_ca) * det_inv;

		if (fabs(t_a) > half_height_a || fabs(t_b) > half_height_b)
		{
			float tmin_a = -half_height_a;
			float tmax_a =  half_height_a;
			float tmin_b = -half_height_b;
			float tmax_b =  half_height_b;

			float boundaries_a[4] =
			{
				tmin_a,
				tmax_a,
				basalt_floatClamp(dot_ca + tmin_b * dot_ab, tmin_a, tmax_a),
				basalt_floatClamp(dot_ca + tmax_b * dot_ab, tmin_a, tmax_a),
			};

			float boundaries_b[4] =
			{
				basalt_floatClamp(-dot_cb + tmin_a * dot_ab, tmin_b, tmax_b),
				basalt_floatClamp(-dot_cb + tmax_a * dot_ab, tmin_b, tmax_b),
				tmin_b,
				tmax_b,
			};

			float closest_distance_sqr = FLT_MAX;
			for (uint32_t i = 0; i < 4; ++i)
			{
				float tbound_a = boundaries_a[i];
				float tbound_b = boundaries_b[i];

				Basalt_Vec3 p_a = basalt_vec3Mad(axis_a, tbound_a, center_a);
				Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, tbound_b, center_b);

				Basalt_Vec3 diff = basalt_vec3Sub(p_b, p_a);
				float l_sqr = basalt_vec3Dot(diff, diff);

				if (l_sqr < closest_distance_sqr)
				{
					closest_distance_sqr = l_sqr;
					t_a = tbound_a;
					t_b = tbound_b;
				}
			}
		}

		manifold.results_a[0] = t_a;
		manifold.results_b[0] = t_b;
		manifold.num_results = 1;
	}
	// near parallel case
	else
	{
		float tmin_a = -half_height_a;
		float tmax_a =  half_height_a;

		float tmin_b = dot_ca - half_height_b * dot_ab;
		float tmax_b = dot_ca + half_height_b * dot_ab;

		if (tmin_b > tmax_b)
			basalt_floatSwap(&tmin_b, &tmax_b);

		if (tmax_a < tmin_b)
		{
			manifold.results_a[0] = tmax_a;
			manifold.results_b[0] = (tmin_b - dot_ca) / dot_ab;

			manifold.num_results = 1;
		}
		else if (tmin_a > tmax_b)
		{
			manifold.results_a[0] = tmin_a;
			manifold.results_b[0] = (tmax_b - dot_ca) / dot_ab;

			manifold.num_results = 1;
		}
		else
		{
			float tmin = basalt_floatMax(tmin_a, tmin_b);
			float tmax = basalt_floatMin(tmax_a, tmax_b);
			float dot_ab_inv = 1.0f / dot_ab;

			manifold.results_a[0] = tmin;
			manifold.results_a[1] = tmax;

			manifold.results_b[0] = (tmin - dot_ca) * dot_ab_inv;
			manifold.results_b[1] = (tmax - dot_ca) * dot_ab_inv;

			manifold.num_results = 2;
		}
	}

	return manifold;
}

typedef struct Basalt_BoxFeatureData_t
{
	Basalt_ShapeFeature feature;
	Basalt_Vec3 surface;
	Basalt_Vec3 normal;
	float penetration;
} Basalt_BoxFeatureData;

static BASALT_INLINE Basalt_BoxFeatureData basalt_boxFeature(Basalt_Vec3 point, Basalt_Vec3 half_sizes)
{
	Basalt_BoxFeatureData result = {0};

	static Basalt_Vec3 local_normals[6] =
	{
		{-1.0f,  0.0f,  0.0f},
		{ 1.0f,  0.0f,  0.0f},
		{ 0.0f, -1.0f,  0.0f},
		{ 0.0f,  1.0f,  0.0f},
		{ 0.0f,  0.0f, -1.0f},
		{ 0.0f,  0.0f,  1.0f},
	};

	float distance_x = fabsf(point.x) - half_sizes.x;
	float distance_y = fabsf(point.y) - half_sizes.y;
	float distance_z = fabsf(point.z) - half_sizes.z;

	float penetration = distance_x;
	uint32_t face = (point.x < 0.0f) ? 0 : 1;
	float offset = half_sizes.x;

	if (penetration < distance_y)
	{
		face = (point.y < 0.0f) ? 2 : 3;
		offset = half_sizes.y;
		penetration = distance_y;
	}

	if (penetration < distance_z)
	{
		face = (point.z < 0.0f) ? 4 : 5;
		offset = half_sizes.z;
		penetration = distance_z;
	}

	Basalt_Vec3 normal = local_normals[face];
	Basalt_Vec3 surface = basalt_vec3Mad(normal, -penetration, point);

	if (penetration > 0.0f)
	{
		surface.x = basalt_floatClamp(point.x, -half_sizes.x, half_sizes.x);
		surface.y = basalt_floatClamp(point.y, -half_sizes.y, half_sizes.y);
		surface.z = basalt_floatClamp(point.z, -half_sizes.z, half_sizes.z);

		normal = basalt_vec3Normalize(basalt_vec3Sub(point, surface));
		offset = basalt_vec3Dot(normal, surface);
	}

	result.feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, face};
	result.normal = normal;
	result.surface = surface;
	result.penetration = basalt_vec3Dot(point, normal) - offset;

	return result;
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
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};
	manifold->contacts[0].position_a = basalt_vec3Mad(manifold->normal, penetration, point);
	manifold->contacts[0].position_b = point;
	manifold->contacts[0].penetration = penetration;

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
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, 0};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};
	manifold->contacts[0].position_a = basalt_vec3Mad(manifold->normal, penetration, point);
	manifold->contacts[0].position_b = point;
	manifold->contacts[0].penetration = penetration;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxPointIntersection(const Basalt_ShapeDataBox *box, Basalt_Vec3 point, Basalt_ContactManifold *manifold)
{
	assert(box);
	assert(manifold);

	Basalt_Vec3 diff = basalt_vec3Sub(point, box->center);
	Basalt_Vec3 half_sizes = {box->sizes.x * 0.5f, box->sizes.y * 0.5f, box->sizes.z * 0.5f};
	
	Basalt_BoxFeatureData feature_data = basalt_boxFeature(diff, half_sizes);

	if (feature_data.penetration > 0.0f)
		return BASALT_NO_INTERSECTION;

	manifold->normal = feature_data.normal;
	manifold->num_contacts = 1;
	manifold->contacts[0].feature_a = feature_data.feature;
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_POINT, 0};
	manifold->contacts[0].position_a = basalt_vec3Add(feature_data.surface, box->center);
	manifold->contacts[0].position_b = point;
	manifold->contacts[0].penetration = feature_data.penetration;

	return BASALT_SUCCESS;
}

/*
 */
static BASALT_INLINE Basalt_Result basalt_sphereSphereIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_SPHERE);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_SPHERE);
	assert(manifold);

	Basalt_ShapeDataSphere sphere_a = info_a->data.sphere;
	Basalt_ShapeDataSphere sphere_b = info_b->data.sphere;

	sphere_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, sphere_b.center));

	Basalt_Vec3 delta = basalt_vec3Sub(sphere_b.center, sphere_a.center);
	float r_sum = sphere_a.radius + sphere_b.radius;

	float l_sqr = basalt_vec3Dot(delta, delta);
	if (l_sqr > r_sum * r_sum)
		return BASALT_NO_INTERSECTION;

	float l = sqrtf(l_sqr);
	float l_inv = 1.0f / l;

	float penetration = l - r_sum;

	manifold->normal = (Basalt_Vec3){delta.x * l_inv, delta.y * l_inv, delta.z * l_inv};
	manifold->num_contacts = 1;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].position_a = basalt_vec3Mad(manifold->normal, sphere_a.radius, sphere_a.center);
	manifold->contacts[0].position_b = basalt_vec3Mad(manifold->normal, -sphere_b.radius, sphere_b.center);
	manifold->contacts[0].penetration = penetration;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_capsuleSphereIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_CAPSULE);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_SPHERE);
	assert(manifold);

	Basalt_ShapeDataCapsule capsule_a = info_a->data.capsule;
	Basalt_ShapeDataSphere sphere_b = info_b->data.sphere;

	sphere_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, sphere_b.center));

	float half_height = capsule_a.height * 0.5f;

	float origin[3] =
	{
		sphere_b.center.x - capsule_a.center.x,
		sphere_b.center.y - capsule_a.center.y,
		sphere_b.center.z - capsule_a.center.z
	};
	float sphere_center_proj = origin[capsule_a.axis];

	uint32_t feature_index = 2;
	if (sphere_center_proj < -half_height)
	{
		sphere_center_proj = -half_height;
		feature_index = 0;
	}
	else if (sphere_center_proj > half_height)
	{
		sphere_center_proj = half_height;
		feature_index = 1;
	}

	float offset[3] = {capsule_a.center.x, capsule_a.center.y, capsule_a.center.z};
	offset[capsule_a.axis] += sphere_center_proj;

	Basalt_Vec3 closest_point = {offset[0], offset[1], offset[2]};
	Basalt_Vec3 delta = basalt_vec3Sub(sphere_b.center, closest_point);

	float r_sum = capsule_a.radius + sphere_b.radius;

	float l_sqr = basalt_vec3Dot(delta, delta);
	if (l_sqr > r_sum * r_sum)
		return BASALT_NO_INTERSECTION;

	float l = sqrtf(l_sqr);
	float l_inv = 1.0f / l;

	float penetration = l - r_sum;

	manifold->normal = (Basalt_Vec3){delta.x * l_inv, delta.y * l_inv, delta.z * l_inv};
	manifold->num_contacts = 1;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, feature_index};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].position_a = basalt_vec3Mad(manifold->normal, capsule_a.radius, closest_point);
	manifold->contacts[0].position_b = basalt_vec3Mad(manifold->normal, -sphere_b.radius, sphere_b.center);
	manifold->contacts[0].penetration = penetration;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxSphereIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_BOX);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_SPHERE);
	assert(manifold);

	Basalt_ShapeDataBox box_a = info_a->data.box;
	Basalt_ShapeDataSphere sphere_b = info_b->data.sphere;

	Basalt_Vec3 half_sizes_a = {box_a.sizes.x * 0.5f, box_a.sizes.y * 0.5f, box_a.sizes.z * 0.5f};

	sphere_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, sphere_b.center));
	Basalt_Vec3 diff = basalt_vec3Sub(sphere_b.center, box_a.center);

	Basalt_BoxFeatureData feature_data = basalt_boxFeature(diff, half_sizes_a);

	float penetration = feature_data.penetration - sphere_b.radius;
	if (penetration > 0.0f)
		return BASALT_NO_INTERSECTION;

	manifold->normal = feature_data.normal;
	manifold->num_contacts = 1;
	manifold->contacts[0].feature_a = feature_data.feature;
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};
	manifold->contacts[0].position_a = basalt_vec3Add(feature_data.surface, box_a.center);
	manifold->contacts[0].position_b = basalt_vec3Mad(manifold->normal, -sphere_b.radius, sphere_b.center);
	manifold->contacts[0].penetration = penetration;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_capsuleCapsuleIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_CAPSULE);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_CAPSULE);
	assert(manifold);

	Basalt_ShapeDataCapsule capsule_a = info_a->data.capsule;
	Basalt_ShapeDataCapsule capsule_b = info_b->data.capsule;

	capsule_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, capsule_b.center));

	static Basalt_Vec3 capsule_axes[3] =
	{
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};

	Basalt_Vec3 axis_a = capsule_axes[capsule_a.axis];
	Basalt_Vec3 axis_b = capsule_axes[capsule_b.axis];
	axis_b = basalt_quatRotateVec3(transform.rotation, axis_b);

	float half_height_a = capsule_a.height * 0.5f;
	float half_height_b = capsule_b.height * 0.5f;

	Basalt_SegmentManifold candidates = basalt_segmentSegmentIntersection(capsule_a.center, capsule_b.center, axis_a, axis_b, half_height_a, half_height_b);

	manifold->num_contacts = 0;

	for (uint32_t i = 0; i < candidates.num_results; ++i)
	{
		float t_a = candidates.results_a[i];
		float t_b = candidates.results_b[i];

		uint32_t index_a = (t_a == -half_height_a) ? 0 : (t_a == half_height_a) ? 1 : 2;
		uint32_t index_b = (t_b == -half_height_b) ? 0 : (t_b == half_height_b) ? 1 : 2;

		Basalt_Vec3 p_a = basalt_vec3Mad(axis_a, t_a, capsule_a.center);
		Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, capsule_b.center);

		Basalt_Vec3 delta = basalt_vec3Sub(p_b, p_a);
		float r_sum = capsule_a.radius + capsule_b.radius;

		float l_sqr = basalt_vec3Dot(delta, delta);
		if (l_sqr > r_sum * r_sum)
			continue;

		float l = sqrtf(l_sqr);
		float l_inv = 1.0f / l;

		float penetration = l - r_sum;

		manifold->normal = (Basalt_Vec3){delta.x * l_inv, delta.y * l_inv, delta.z * l_inv};

		Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
		contact->feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_a};
		contact->feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_b};
		contact->position_a = basalt_vec3Mad(manifold->normal, capsule_a.radius, p_a);
		contact->position_b = basalt_vec3Mad(manifold->normal, -capsule_b.radius, p_b);
		contact->penetration = penetration;
	}

	if (manifold->num_contacts == 0)
		return BASALT_NO_INTERSECTION;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxCapsuleIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_BOX);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_CAPSULE);
	assert(manifold);

	Basalt_ShapeDataBox box_a = info_a->data.box;
	Basalt_ShapeDataCapsule capsule_b = info_b->data.capsule;

	capsule_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, capsule_b.center));

	static Basalt_Vec3 basis_axes[3] =
	{
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};

	Basalt_Vec3 half_sizes_a = {box_a.sizes.x * 0.5f, box_a.sizes.y * 0.5f, box_a.sizes.z * 0.5f};
	Basalt_Vec3 axis_b = basis_axes[capsule_b.axis];
	axis_b = basalt_quatRotateVec3(transform.rotation, axis_b);

	float half_height_b = capsule_b.height * 0.5f;

	capsule_b.center = basalt_vec3Sub(capsule_b.center, box_a.center);

	float min_penetration = FLT_MAX;
	uint32_t min_axis = UINT32_MAX;
	uint32_t min_axis_sign = UINT32_MAX;

	for (uint32_t i = 0; i < 3; ++i)
	{
		Basalt_Vec3 axis = basis_axes[i];

		float box_radius = fabsf(axis.x) * half_sizes_a.x + fabsf(axis.y) * half_sizes_a.y + fabsf(axis.z) * half_sizes_a.z;

		float box_min = -box_radius;
		float box_max =  box_radius;

		float capsule_center = basalt_vec3Dot(capsule_b.center, axis);
		float capsule_proj = fabsf(basalt_vec3Dot(axis_b, axis));

		float capsule_min = capsule_center - half_height_b * capsule_proj - capsule_b.radius;
		float capsule_max = capsule_center + half_height_b * capsule_proj + capsule_b.radius;

		if (box_max < capsule_min || capsule_max < box_min)
			return BASALT_NO_INTERSECTION;

		float penetration0 = box_max - capsule_min;
		float penetration1 = capsule_max - box_min;

		float penetration = basalt_floatMin(penetration0, penetration1);
		if (penetration < min_penetration)
		{
			min_penetration = penetration;
			min_axis = i;
			min_axis_sign = (penetration0 < penetration1) ? 1 : 0;
		}
	}

	assert(min_axis != UINT32_MAX);
	assert(min_axis_sign != UINT32_MAX);

	float face_sign = min_axis_sign * 2.0f - 1.0f;
	Basalt_ShapeFeature box_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, 2 * min_axis + min_axis_sign};

	Basalt_Vec3 face_normal = {0};
	float face_offset = 0.0f;

	uint32_t clip_axis0 = (min_axis + 1) % 3;
	uint32_t clip_axis1 = (min_axis + 2) % 3;

	float half_sizes[3] = {half_sizes_a.x, half_sizes_a.y, half_sizes_a.z};

	uint32_t edge_axes[4] = {clip_axis0, clip_axis0, clip_axis1, clip_axis1};
	float edge_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

	{
		float axis[3] = {0};
		axis[min_axis] = face_sign;

		face_normal = (Basalt_Vec3){axis[0], axis[1], axis[2]};
		face_offset = -half_sizes[min_axis];
	}

	float tmin = -half_height_b;
	float tmax =  half_height_b;
	uint32_t fully_clipped = 0;

	for (uint32_t i = 0; i < 4; ++i)
	{
		uint32_t axis_index = edge_axes[i];

		float axis[3] = {0};
		axis[axis_index] = edge_signs[i];

		Basalt_Vec3 clip_normal = {axis[0], axis[1], axis[2]};
		float clip_offset = -half_sizes[axis_index];

		Basalt_Vec3 pmin = basalt_vec3Mad(axis_b, tmin, capsule_b.center);
		Basalt_Vec3 pmax = basalt_vec3Mad(axis_b, tmax, capsule_b.center);

		float s0 = basalt_vec3Dot(clip_normal, pmin) + clip_offset;
		float s1 = basalt_vec3Dot(clip_normal, pmax) + clip_offset;

		if (s0 > 0.0f && s1 > 0.0f)
		{
			fully_clipped = 1;
			break;
		}

		if (s0 * s1 < 0.0f)
		{
			float dot_clipped = basalt_vec3Dot(axis_b, clip_normal);
			assert(fabs(dot_clipped) > 0.0f);

			float t = -(basalt_vec3Dot(clip_normal, capsule_b.center) + clip_offset) / dot_clipped;
			assert(t >= tmin && t <= tmax);

			if (s0 > 0.0f)
				tmin = t;

			if (s1 > 0.0f)
				tmax = t;
		}
	}

	manifold->num_contacts = 0;

	if (!fully_clipped)
	{
		assert(tmin <= tmax);

		float candidates[2] = {tmin, tmax};
		float r_sqr = capsule_b.radius * capsule_b.radius;

		manifold->normal = face_normal;

		for (uint32_t i = 0; i < 2; ++i)
		{
			float t_b = candidates[i];
			uint32_t index_b = (t_b == -half_height_b) ? 0 : (t_b == half_height_b) ? 1 : 2;

			Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, capsule_b.center);
			float d_b = basalt_vec3Dot(p_b, face_normal) + face_offset;

			Basalt_Vec3 p_a = basalt_vec3Mad(face_normal, -d_b, p_b);

			float ray_distance = FLT_MAX;
			Basalt_Vec3 ray_origin = p_a;
			Basalt_Vec3 ray_direction = {-face_normal.x, -face_normal.y, -face_normal.z};
			Basalt_Vec3 ray_origin_shifted = basalt_vec3Sub(ray_origin, capsule_b.center);

			// ray-sphere
			float sphere_offsets[2] = {-half_height_b, half_height_b};
			for (uint32_t sphere_idx = 0; sphere_idx < 2; ++sphere_idx)
			{
				float t_s = sphere_offsets[sphere_idx];
				Basalt_Vec3 sphere_center = basalt_vec3Mad(axis_b, t_s, capsule_b.center);
				Basalt_Vec3 delta = basalt_vec3Sub(ray_origin, sphere_center);

				float dot_od = basalt_vec3Dot(delta, ray_direction);
				float dot_oo = basalt_vec3Dot(delta, delta);
				float dot_dd = basalt_vec3Dot(ray_direction, ray_direction);

				float d = dot_od * dot_od - dot_dd * (dot_oo - r_sqr);
				if (d > 0.0f)
				{
					float d_sqrt = sqrtf(d);
					
					assert(dot_dd > 0.0f);
					float dot_dd_rcp = 1.0f / dot_dd;
					
					float intersections[2] = {0};
					intersections[0] = (-dot_od - d_sqrt) * dot_dd_rcp;
					intersections[1] = (-dot_od + d_sqrt) * dot_dd_rcp;

					for (uint32_t j = 0; j < 2; ++j)
					{
						float candidate = intersections[j];
						if (candidate < 0.0f)
							continue;

						Basalt_Vec3 test = basalt_vec3Mad(ray_direction, candidate, ray_origin_shifted);
						float projection = basalt_vec3Dot(test, axis_b);
						if (t_s < 0.0f && projection > t_s)
							continue;

						if (t_s > 0.0f && projection < t_s)
							continue;

						ray_distance = basalt_floatMin(ray_distance, candidate);
					}
				}
			}

			// ray-cylinder
			{
				float d_proj = basalt_vec3Dot(ray_direction, axis_b);
				float o_proj = basalt_vec3Dot(ray_origin_shifted, axis_b);

				Basalt_Vec3 ray_origin_proj = basalt_vec3Mad(axis_b, -o_proj, ray_origin_shifted);
				Basalt_Vec3 ray_direction_proj = basalt_vec3Mad(axis_b, -d_proj, ray_direction);

				float dot_od = basalt_vec3Dot(ray_origin_proj, ray_direction_proj);
				float dot_oo = basalt_vec3Dot(ray_origin_proj, ray_origin_proj);
				float dot_dd = basalt_vec3Dot(ray_direction_proj, ray_direction_proj);

				float d = dot_od * dot_od - dot_dd * (dot_oo - r_sqr);
				if (d > 0.0f)
				{
					float d_sqrt = sqrtf(d);

					assert(dot_dd > 0.0f);
					float dot_dd_rcp = 1.0f / dot_dd;

					float intersections[2] = {0};
					intersections[0] = (-dot_od - d_sqrt) * dot_dd_rcp;
					intersections[1] = (-dot_od + d_sqrt) * dot_dd_rcp;

					for (uint32_t j = 0; j < 2; ++j)
					{
						float candidate = intersections[j];
						if (candidate < 0.0f)
							continue;

						Basalt_Vec3 test = basalt_vec3Mad(ray_direction, candidate, ray_origin_shifted);
						float projection = basalt_vec3Dot(test, axis_b);
						if (fabs(projection) > half_height_b)
							continue;

						ray_distance = basalt_floatMin(ray_distance, candidate);
					}
				}
			}

			if (ray_distance == FLT_MAX)
				continue;

			p_a = basalt_vec3Add(p_a, box_a.center);
			p_b = basalt_vec3Add(p_b, box_a.center);

			float penetration = -ray_distance;

			Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
			contact->feature_a = box_feature;
			contact->feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_b};
			contact->position_a = p_a;
			contact->position_b = basalt_vec3Mad(manifold->normal, penetration, p_a);
			contact->penetration = penetration;
		}
	}
	else
	{
		float min_edge_distance = FLT_MAX;

		Basalt_SegmentManifold candidates = {0};
		Basalt_Vec3 edge_axis = {0};
		Basalt_Vec3 edge_center = {0};
		float edge_half_length = 0.0f;

		for (uint32_t i = 0; i < 4; ++i)
		{
			uint32_t center_index = edge_axes[i];
			uint32_t axis_index = edge_axes[(i + 2) % 4];

			float center[3] = {0};
			center[center_index] = edge_signs[i] * half_sizes[center_index];
			center[min_axis] = face_sign * half_sizes[min_axis];

			float axis[3] = {0};
			axis[axis_index] = 1.0f;

			Basalt_Vec3 center_a = {center[0], center[1], center[2]};
			Basalt_Vec3 axis_a = {axis[0], axis[1], axis[2]};
			float half_height_a = half_sizes[axis_index];

			Basalt_SegmentManifold segment_candidates = basalt_segmentSegmentIntersection(center_a, capsule_b.center, axis_a, axis_b, half_height_a, half_height_b);

			float t_a = segment_candidates.results_a[0];
			float t_b = segment_candidates.results_b[0];

			Basalt_Vec3 p_a = basalt_vec3Mad(axis_a, t_a, center_a);
			Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, capsule_b.center);

			Basalt_Vec3 delta = basalt_vec3Sub(p_b, p_a);
			float l_sqr = basalt_vec3Dot(delta, delta);

			if (min_edge_distance > l_sqr)
			{
				edge_center = center_a;
				edge_axis = axis_a;
				edge_half_length = half_height_a;

				min_edge_distance = l_sqr;
				candidates = segment_candidates;
			}
		}

		for (uint32_t i = 0; i < candidates.num_results; ++i)
		{
			float t_a = candidates.results_a[i];
			float t_b = candidates.results_b[i];

			uint32_t index_b = (t_b == -half_height_b) ? 0 : (t_b == half_height_b) ? 1 : 2;

			Basalt_Vec3 p_a = basalt_vec3Mad(edge_axis, t_a, edge_center);
			Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, capsule_b.center);

			Basalt_Vec3 delta = basalt_vec3Sub(p_b, p_a);
			float dot_sign = basalt_vec3Dot(delta, face_normal) < 0.0f ? -1.0f : 1.0f;

			float l_sqr = basalt_vec3Dot(delta, delta);
			float r = capsule_b.radius;

			if (dot_sign > 0.0f && l_sqr > r * r)
				continue;

			float penetration = sqrtf(l_sqr);

			if ((fabsf(penetration) > 1e-06))
			{
				float penetration_inv = 1.0f / penetration;

				delta.x *= penetration_inv * dot_sign;
				delta.y *= penetration_inv * dot_sign;
				delta.z *= penetration_inv * dot_sign;

				penetration -= r * dot_sign;
				penetration *= dot_sign;
			}
			else
			{
				delta = face_normal;
				penetration -= r;
			}

			p_a = basalt_vec3Add(p_a, box_a.center);
			p_b = basalt_vec3Add(p_b, box_a.center);

			manifold->normal = (Basalt_Vec3){delta.x, delta.y, delta.z};

			Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
			contact->feature_a = box_feature;
			contact->feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_b};
			contact->position_a = p_a;
			contact->position_b = basalt_vec3Mad(manifold->normal, -capsule_b.radius, p_b);
			contact->penetration = penetration;
		}
	}

	if (manifold->num_contacts == 0)
		return BASALT_NO_INTERSECTION;

	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxBoxIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	assert(info_a);
	assert(info_a->type == BASALT_SHAPE_TYPE_BOX);
	assert(info_b);
	assert(info_b->type == BASALT_SHAPE_TYPE_BOX);
	assert(manifold);

	Basalt_ShapeDataBox box_a = info_a->data.box;
	Basalt_ShapeDataBox box_b = info_b->data.box;

	box_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, box_b.center));

	static Basalt_Vec3 box_a_axes[3] =
	{
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};

	Basalt_Vec3 box_b_axes[3] = {0};
	for (uint32_t i = 0; i < 3; ++i)
		box_b_axes[i] = basalt_quatRotateVec3(transform.rotation, box_a_axes[i]);

	float half_sizes_a[3] = {box_a.sizes.x * 0.5f, box_a.sizes.y * 0.5f, box_a.sizes.z * 0.5f};
	float half_sizes_b[3] = {box_b.sizes.x * 0.5f, box_b.sizes.y * 0.5f, box_b.sizes.z * 0.5f};

	Basalt_Vec3 sat_axes[15] = {0};

	uint32_t num_axes = 0;
	sat_axes[num_axes++] = box_a_axes[0];
	sat_axes[num_axes++] = box_a_axes[1];
	sat_axes[num_axes++] = box_a_axes[2];
	sat_axes[num_axes++] = box_b_axes[0];
	sat_axes[num_axes++] = box_b_axes[1];
	sat_axes[num_axes++] = box_b_axes[2];

	for (uint32_t i = 0; i < 3; ++i)
	{
		for (uint32_t j = 0; j < 3; ++j)
		{
			Basalt_Vec3 axis = basalt_vec3Cross(box_a_axes[i], box_b_axes[j]);
			float l_sqr = basalt_vec3Dot(axis, axis);
			if (l_sqr == 0.0f)
				continue;

			float l_inv = 1.0f / sqrtf(l_sqr);
			axis.x *= l_inv;
			axis.y *= l_inv;
			axis.z *= l_inv;

			sat_axes[num_axes++] = axis;
		}
	}

	box_b.center = basalt_vec3Sub(box_b.center, box_a.center);

	float min_penetration = FLT_MAX;
	uint32_t min_axis_sign = UINT32_MAX;
	uint32_t min_axis = UINT32_MAX;

	for (uint32_t i = 0; i < num_axes; ++i)
	{
		Basalt_Vec3 axis = sat_axes[i];

		float box_a_radius = fabsf(axis.x) * half_sizes_a[0] + fabsf(axis.y) * half_sizes_a[1] + fabsf(axis.z) * half_sizes_a[2];

		float box_a_min = -box_a_radius;
		float box_a_max =  box_a_radius;

		Basalt_Vec3 box_b_proj =
		{
			fabsf(basalt_vec3Dot(box_b_axes[0], axis)),
			fabsf(basalt_vec3Dot(box_b_axes[1], axis)),
			fabsf(basalt_vec3Dot(box_b_axes[2], axis)),
		};

		float box_b_center = basalt_vec3Dot(box_b.center, axis);
		float box_b_radius = box_b_proj.x * half_sizes_b[0] + box_b_proj.y * half_sizes_b[1] + box_b_proj.z * half_sizes_b[2];

		float box_b_min = box_b_center - box_b_radius;
		float box_b_max = box_b_center + box_b_radius;

		if (box_a_max < box_b_min || box_b_max < box_a_min)
			return BASALT_NO_INTERSECTION;

		float penetration0 = box_a_max - box_b_min;
		float penetration1 = box_b_max - box_a_min;

		float penetration = basalt_floatMin(penetration0, penetration1);
		if (penetration < min_penetration)
		{
			min_penetration = penetration;
			min_axis = i;
			min_axis_sign = (penetration0 < penetration1) ? 1 : 0;
		}
	}

	assert(min_axis != UINT32_MAX);
	assert(min_axis_sign != UINT32_MAX);

	// face-face contact
	if (min_axis < 6)
	{
		// find reference & incident faces
		const Basalt_Vec3 *axes[2] = {box_a_axes, box_b_axes};
		const float *half_sizes[2] = {half_sizes_a, half_sizes_b};
		Basalt_Vec3 centers[2] = {0};
		centers[1] = box_b.center;

		uint32_t faces[2] = {UINT32_MAX};
		float signs[2] = {FLT_MAX};

		uint32_t src_index = (min_axis < 3) ? 0 : 1;
		uint32_t dst_index = (src_index + 1) % 2;

		const Basalt_Vec3 *src_axes = axes[src_index];
		const Basalt_Vec3 *dst_axes = axes[dst_index];

		float reference_sign = (min_axis_sign > 0) ? 1.0f : -1.0f;
		if (src_index != 0)
			reference_sign *= -1.0f;

		faces[src_index] = min_axis % 3;
		signs[src_index] = reference_sign;

		Basalt_Vec3 src_normal = src_axes[faces[src_index]];
		src_normal.x *= signs[src_index];
		src_normal.y *= signs[src_index];
		src_normal.z *= signs[src_index];

		float min_dot = FLT_MAX;

		for (uint32_t i = 0; i < 3; ++i)
		{
			Basalt_Vec3 dst_normal = dst_axes[i];

			float current_dot = basalt_vec3Dot(dst_normal, src_normal);
			if (min_dot > current_dot)
			{
				faces[dst_index] = i;
				signs[dst_index] = 1.0f;
				min_dot = current_dot;
			}

			dst_normal = (Basalt_Vec3){-dst_normal.x, -dst_normal.y, -dst_normal.z};

			current_dot = basalt_vec3Dot(dst_normal, src_normal);
			if (min_dot > current_dot)
			{
				faces[dst_index] = i;
				signs[dst_index] = -1.0f;
				min_dot = current_dot;
			}
		}

		assert(faces[0] != UINT32_MAX);
		assert(faces[1] != UINT32_MAX);
		assert(signs[0] != FLT_MAX);
		assert(signs[1] != FLT_MAX);

		// clip incident face by reference face
		uint32_t reference_axis0 = faces[src_index];
		uint32_t reference_axis1 = (reference_axis0 + 1) % 3;
		uint32_t reference_axis2 = (reference_axis0 + 2) % 3;

		uint32_t reference_plane_axes[5] = {reference_axis0, reference_axis1, reference_axis1, reference_axis2, reference_axis2};
		float reference_plane_signs[5] = {signs[src_index], -1.0f, 1.0f, -1.0f, 1.0f};

		const Basalt_Vec3 *reference_axes = axes[src_index];
		const float *reference_half_sizes = half_sizes[src_index];

		float buffer[48] = {0};
		float *src_vertices = &buffer[0];
		float *dst_vertices = &buffer[24];

		uint32_t incident_axis0 = faces[dst_index];
		uint32_t incident_axis1 = (incident_axis0 + 1) % 3;
		uint32_t incident_axis2 = (incident_axis0 + 2) % 3;

		const Basalt_Vec3 *incident_axes = axes[dst_index];
		const float *incident_half_sizes = half_sizes[dst_index];

		float offsets[8] = {1.0f, 1.0f, 1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f};

		for (uint32_t i = 0; i < 4; ++i)
		{
			Basalt_Vec3 p = basalt_vec3Mad(incident_axes[incident_axis0], signs[dst_index] * incident_half_sizes[incident_axis0], centers[dst_index]);
			p = basalt_vec3Mad(incident_axes[incident_axis1], offsets[2 * i + 0] * incident_half_sizes[incident_axis1], p);
			p = basalt_vec3Mad(incident_axes[incident_axis2], offsets[2 * i + 1] * incident_half_sizes[incident_axis2], p);

			src_vertices[i * 3 + 0] = p.x;
			src_vertices[i * 3 + 1] = p.y;
			src_vertices[i * 3 + 2] = p.z;
		}

		uint32_t src_count = 4;
		uint32_t dst_count = 0;

		for (uint32_t i = 0; i < 5; ++i)
		{
			uint32_t axis_index = reference_plane_axes[i];

			Basalt_Vec3 clip_normal = reference_axes[axis_index];
			clip_normal.x *= reference_plane_signs[i];
			clip_normal.y *= reference_plane_signs[i];
			clip_normal.z *= reference_plane_signs[i];

			float clip_offset = -reference_half_sizes[axis_index] - basalt_vec3Dot(clip_normal, centers[src_index]);

			for (uint32_t j = 0; j < src_count; ++j)
			{
				uint32_t j0 = j;
				uint32_t j1 = (j + 1) % src_count;

				Basalt_Vec3 v0 = (Basalt_Vec3){src_vertices[3 * j0 + 0], src_vertices[3 * j0 + 1], src_vertices[3 * j0 + 2]};
				Basalt_Vec3 v1 = (Basalt_Vec3){src_vertices[3 * j1 + 0], src_vertices[3 * j1 + 1], src_vertices[3 * j1 + 2]};

				float s0 = basalt_vec3Dot(clip_normal, v0) + clip_offset;
				float s1 = basalt_vec3Dot(clip_normal, v1) + clip_offset;

				if (s0 > 0.0f && s1 > 0.0f)
					continue;

				if (s0 <= 0.0f)
				{
					dst_vertices[dst_count * 3 + 0] = v0.x;
					dst_vertices[dst_count * 3 + 1] = v0.y;
					dst_vertices[dst_count * 3 + 2] = v0.z;
					dst_count++;
				}

				if (s0 * s1 < 0.0f)
				{
					Basalt_Vec3 edge = basalt_vec3Sub(v1, v0);

					float dot_clipped = basalt_vec3Dot(edge, clip_normal);
					assert(fabs(dot_clipped) > 0.0f);

					float t = -(basalt_vec3Dot(clip_normal, v0) + clip_offset) / dot_clipped;
					assert(t >= 0.0f && t <= 1.0f);

					Basalt_Vec3 p = basalt_vec3Mad(edge, t, v0);

					dst_vertices[dst_count * 3 + 0] = p.x;
					dst_vertices[dst_count * 3 + 1] = p.y;
					dst_vertices[dst_count * 3 + 2] = p.z;
					dst_count++;
				}
			}

			src_count = dst_count;
			dst_count = 0;

			float *temp = src_vertices;
			src_vertices = dst_vertices;
			dst_vertices = temp;
		}

		// fill manifold
		uint32_t box_a_face = 2 * faces[0] + ((signs[0] < 0.0f) ? 0 : 1);
		uint32_t box_b_face = 2 * faces[1] + ((signs[1] < 0.0f) ? 0 : 1);

		Basalt_ShapeFeature box_a_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, box_a_face};
		Basalt_ShapeFeature box_b_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, box_b_face};

		uint32_t axis_index = reference_plane_axes[0];

		Basalt_Vec3 reference_normal = reference_axes[axis_index];
		reference_normal.x *= reference_plane_signs[0];
		reference_normal.y *= reference_plane_signs[0];
		reference_normal.z *= reference_plane_signs[0];

		float clip_offset = -reference_half_sizes[axis_index] - basalt_vec3Dot(reference_normal, centers[src_index]);

		manifold->num_contacts = 0;
		manifold->normal = reference_normal;

		for (uint32_t i = 0; i < src_count; ++i)
		{
			Basalt_Vec3 incident_point = {src_vertices[3 * i + 0], src_vertices[3 * i + 1], src_vertices[3 * i + 2]};
			float incident_distance = basalt_vec3Dot(reference_normal, incident_point) + clip_offset;

			Basalt_Vec3 reference_point = basalt_vec3Mad(reference_normal, -incident_distance, incident_point);

			reference_point = basalt_vec3Add(reference_point, box_a.center);
			incident_point = basalt_vec3Add(incident_point, box_a.center);

			Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
			contact->feature_a = box_a_feature;
			contact->feature_b = box_b_feature;
			contact->position_a = (src_index == 0) ? reference_point : incident_point;
			contact->position_b = (src_index == 0) ? incident_point : reference_point;
			contact->penetration = incident_distance;
		}

		if (src_index != 0)
			manifold->normal = (Basalt_Vec3){-manifold->normal.x, -manifold->normal.y, -manifold->normal.z};
	}
	// edge-edge contact
	else
	{
		float sign = (min_axis_sign > 0) ? 1.0f : -1.0f;
		Basalt_Vec3 normal = sat_axes[min_axis];
		normal.x *= sign;
		normal.y *= sign;
		normal.z *= sign;

		uint32_t box_a_axis0 = (min_axis - 6) / 3;
		uint32_t box_a_axis1 = (box_a_axis0 + 1) % 3;
		uint32_t box_a_axis2 = (box_a_axis0 + 2) % 3;

		float sign_a_axis1 = (basalt_vec3Dot(box_a_axes[box_a_axis1], normal) > 0.0f) ? 1.0f : -1.0f;
		float sign_a_axis2 = (basalt_vec3Dot(box_a_axes[box_a_axis2], normal) > 0.0f) ? 1.0f : -1.0f;

		float half_size_a = half_sizes_a[box_a_axis0];
		Basalt_Vec3 axis_a = box_a_axes[box_a_axis0];
		Basalt_Vec3 center_a = {0};
		center_a = basalt_vec3Mad(box_a_axes[box_a_axis1], sign_a_axis1 * half_sizes_a[box_a_axis1], center_a);
		center_a = basalt_vec3Mad(box_a_axes[box_a_axis2], sign_a_axis2 * half_sizes_a[box_a_axis2], center_a);

		uint32_t box_b_axis0 = (min_axis - 6) % 3;
		uint32_t box_b_axis1 = (box_b_axis0 + 1) % 3;
		uint32_t box_b_axis2 = (box_b_axis0 + 2) % 3;

		float sign_b_axis1 = (basalt_vec3Dot(box_b_axes[box_b_axis1], normal) > 0.0f) ? -1.0f : 1.0f;
		float sign_b_axis2 = (basalt_vec3Dot(box_b_axes[box_b_axis2], normal) > 0.0f) ? -1.0f : 1.0f;

		float half_size_b = half_sizes_b[box_b_axis0];
		Basalt_Vec3 axis_b = box_b_axes[box_b_axis0];
		Basalt_Vec3 center_b = box_b.center;
		center_b = basalt_vec3Mad(box_b_axes[box_b_axis1], sign_b_axis1 * half_sizes_b[box_b_axis1], center_b);
		center_b = basalt_vec3Mad(box_b_axes[box_b_axis2], sign_b_axis2 * half_sizes_b[box_b_axis2], center_b);

		Basalt_SegmentManifold candidates = basalt_segmentSegmentIntersection(center_a, center_b, axis_a, axis_b, half_size_a, half_size_b);

		uint32_t box_a_edge = 4 * box_a_axis0 + 2 * (sign_a_axis1 > 0.0f) + (sign_a_axis2 > 0.0f);
		uint32_t box_b_edge = 4 * box_b_axis0 + 2 * (sign_b_axis1 > 0.0f) + (sign_b_axis2 > 0.0f);

		Basalt_ShapeFeature box_a_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_EDGE, box_a_edge};
		Basalt_ShapeFeature box_b_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_EDGE, box_b_edge};

		manifold->num_contacts = 0;
		for (uint32_t i = 0; i < candidates.num_results; ++i)
		{
			float t_a = candidates.results_a[i];
			float t_b = candidates.results_b[i];

			Basalt_Vec3 p_a = basalt_vec3Mad(axis_a, t_a, center_a);
			Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, center_b);

			Basalt_Vec3 delta = basalt_vec3Sub(p_b, p_a);

			float l_sqr = basalt_vec3Dot(delta, delta);
			
			float dot_sign = basalt_vec3Dot(delta, normal) < 0.0f ? -1.0f : 1.0f;

			float penetration = sqrtf(l_sqr);
			float penetration_inv = 1.0f / penetration;
			delta.x *= penetration_inv * dot_sign;
			delta.y *= penetration_inv * dot_sign;
			delta.z *= penetration_inv * dot_sign;

			p_a = basalt_vec3Add(p_a, box_a.center);
			p_b = basalt_vec3Add(p_b, box_a.center);

			manifold->normal = (Basalt_Vec3){delta.x, delta.y, delta.z};

			Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
			contact->feature_a = box_a_feature;
			contact->feature_b = box_b_feature;
			contact->position_a = p_a;
			contact->position_b = p_b;
			contact->penetration = -penetration;
		}
	}

	if (manifold->num_contacts == 0)
		return BASALT_NO_INTERSECTION;

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
		float t_s = sphere_offsets[sphere_id];
		Basalt_Vec3 sphere_origin = basalt_vec3Mad(axis, t_s, origin);

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

			if (t_s < 0.0f && projection > t_s)
				continue;

			if (t_s > 0.0f && projection < t_s)
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
		uint32_t index = 2 * i + 0;
		float result = results[index];

		float tcmin = result;
		float tcmax =  result;
		uint32_t fcmin = index;
		uint32_t fcmax = index;

		index = 2 * i + 1;
		result = results[index];

		if (tcmin > result)
		{
			fcmin = index;
			tcmin = result;
		}

		if (tcmax < result)
		{
			fcmax = index;
			tcmax = result;
		}

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

	// postprocess
	manifold->normal = basalt_quatRotateVec3(transform.rotation, manifold->normal);

	for (uint32_t i = 0; i < manifold->num_contacts; ++i)
	{
		Basalt_Contact *contact = &manifold->contacts[i];

		contact->position_a = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, contact->position_a));
		contact->position_b = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, contact->position_b));
	}

	return BASALT_SUCCESS;
}

Basalt_Result impl_instanceShapeIntersectShape(Basalt_Instance this, Basalt_Shape shape_a, Basalt_Transform transform_a, Basalt_Shape shape_b, Basalt_Transform transform_b, Basalt_ContactManifold *manifold)
{
	assert(this);
	assert(shape_a);
	assert(shape_b);
	assert(manifold);

	Impl_Instance *instance_ptr = (Impl_Instance *)this;
	Impl_Shape *shape_a_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, (Basalt_PoolHandle)shape_a);
	assert(shape_a_ptr);

	Impl_Shape *shape_b_ptr = (Impl_Shape *)basalt_poolGetElement(&instance_ptr->shapes, (Basalt_PoolHandle)shape_b);
	assert(shape_b_ptr);

	static PFN_basalt_shapeShapeIntersection handlers[] =
	{
		basalt_sphereSphereIntersection,
		basalt_capsuleSphereIntersection,
		basalt_boxSphereIntersection,
		NULL, // swapped case
		basalt_capsuleCapsuleIntersection,
		basalt_boxCapsuleIntersection,
		NULL, // swapped case
		NULL, // swapped case
		basalt_boxBoxIntersection,
	};

	Basalt_ShapeInfo *info_a = &shape_a_ptr->info;
	Basalt_ShapeInfo *info_b = &shape_b_ptr->info;

	uint32_t swapped = 0;
	if (info_a->type < info_b->type)
	{
		Basalt_ShapeInfo *temp_info = info_a;
		info_a = info_b;
		info_b = temp_info;
		swapped = 1;

		Basalt_Transform temp_transform = transform_a;
		transform_a = transform_b;
		transform_b = temp_transform;
	}

	Basalt_Transform transform = basalt_mulTransform(basalt_invertTransform(transform_a), transform_b);

	PFN_basalt_shapeShapeIntersection handler = handlers[info_b->type * BASALT_SHAPE_TYPE_ENUM_MAX + info_a->type];
	assert(handler != NULL);

	memset(manifold, 0, sizeof(Basalt_ContactManifold));
	Basalt_Result result = handler(info_a, info_b, transform, manifold);
	if (result != BASALT_SUCCESS)
		return result;

	// postprocess
	{
		manifold->normal = basalt_quatRotateVec3(transform_a.rotation, manifold->normal);

		for (uint32_t i = 0; i < manifold->num_contacts; ++i)
		{
			Basalt_Contact *contact = &manifold->contacts[i];
			contact->position_a = basalt_vec3Add(transform_a.position, basalt_quatRotateVec3(transform_a.rotation, contact->position_a));
			contact->position_b = basalt_vec3Add(transform_a.position, basalt_quatRotateVec3(transform_a.rotation, contact->position_b));
		}
	}

	if (swapped)
	{
		manifold->normal = (Basalt_Vec3){-manifold->normal.x, -manifold->normal.y, -manifold->normal.z};

		for (uint32_t i = 0; i < manifold->num_contacts; ++i)
		{
			Basalt_Contact *contact = &manifold->contacts[i];

			Basalt_ShapeFeature temp_feature = contact->feature_a;
			contact->feature_a = contact->feature_b;
			contact->feature_b = temp_feature;

			Basalt_Vec3 temp_position = contact->position_a;
			contact->position_a = contact->position_b;
			contact->position_b = temp_position;
		}
	}

	return BASALT_SUCCESS;
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
