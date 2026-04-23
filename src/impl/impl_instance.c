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
	manifold->contacts[0].position = basalt_vec3Mad(manifold->normal, penetration, sphere_b.center);
	manifold->contacts[0].penetration = penetration * 0.5f;

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

	float closest_point[3] = {capsule_a.center.x, capsule_a.center.y, capsule_a.center.z};
	closest_point[capsule_a.axis] += sphere_center_proj;

	Basalt_Vec3 delta =
	{
		sphere_b.center.x - closest_point[0],
		sphere_b.center.y - closest_point[1],
		sphere_b.center.z - closest_point[2],
	};
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
	manifold->contacts[0].position = basalt_vec3Mad(manifold->normal, penetration, sphere_b.center);
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

	Basalt_Vec3 half_sizes = {box_a.sizes.x * 0.5f, box_a.sizes.y * 0.5f, box_a.sizes.z * 0.5f};
	Basalt_Vec3 box_a_min = basalt_vec3Sub(box_a.center, half_sizes);
	Basalt_Vec3 box_a_max = basalt_vec3Add(box_a.center, half_sizes);

	sphere_b.center = basalt_vec3Add(transform.position, basalt_quatRotateVec3(transform.rotation, sphere_b.center));

	Basalt_Vec3 clipped = sphere_b.center;
	clipped.x = basalt_floatClamp(clipped.x, box_a_min.x, box_a_max.x);
	clipped.y = basalt_floatClamp(clipped.y, box_a_min.y, box_a_max.y);
	clipped.z = basalt_floatClamp(clipped.z, box_a_min.z, box_a_max.z);

	Basalt_Vec3 delta = basalt_vec3Sub(sphere_b.center, clipped);
	float l_sqr = basalt_vec3Dot(delta, delta);

	if (l_sqr > sphere_b.radius * sphere_b.radius)
		return BASALT_NO_INTERSECTION;

	float l = sqrtf(l_sqr);
	float l_inv = 1.0f / l;

	float penetration = l - sphere_b.radius;

	Basalt_Vec3 diff = basalt_vec3Sub(clipped, box_a.center);
	float distance_x = fabsf(diff.x) - half_sizes.x;
	float distance_y = fabsf(diff.y) - half_sizes.y;
	float distance_z = fabsf(diff.z) - half_sizes.z;

	float closest_feature = distance_x;
	uint32_t face = (diff.x < 0.0f) ? 0 : 1;

	if (closest_feature < distance_y)
	{
		face = (diff.y < 0.0f) ? 2 : 3;
		closest_feature = distance_y;
	}

	if (closest_feature < distance_z)
	{
		face = (diff.z < 0.0f) ? 4 : 5;
		closest_feature = distance_z;
	}

	manifold->normal = (Basalt_Vec3){delta.x * l_inv, delta.y * l_inv, delta.z * l_inv};
	manifold->num_contacts = 1;
	manifold->contacts[0].position = clipped;
	manifold->contacts[0].penetration = penetration;
	manifold->contacts[0].feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, face};
	manifold->contacts[0].feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_SPHERE_SURFACE, 0};

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
		contact->position = basalt_vec3Mad(manifold->normal, penetration, p_b);
		contact->penetration = penetration;
		contact->feature_a = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_a};
		contact->feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index_b};
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

	Basalt_Vec3 sat_axes[7] = {0};

	uint32_t num_axes = 0;
	sat_axes[num_axes++] = basis_axes[0];
	sat_axes[num_axes++] = basis_axes[1];
	sat_axes[num_axes++] = basis_axes[2];
	sat_axes[num_axes++] = axis_b;

	for (uint32_t i = 0; i < 3; ++i)
	{
		Basalt_Vec3 axis = basalt_vec3Cross(basis_axes[i], axis_b);
		float l_sqr = basalt_vec3Dot(axis, axis);
		if (l_sqr == 0.0f)
			continue;

		float l_inv = 1.0f / sqrtf(l_sqr);
		axis.x *= l_inv;
		axis.y *= l_inv;
		axis.z *= l_inv;

		sat_axes[num_axes++] = axis;
	}

	capsule_b.center = basalt_vec3Sub(capsule_b.center, box_a.center);

	float min_penetration = FLT_MAX;
	int min_axis_sign = 0;
	uint32_t min_axis = UINT32_MAX;

	for (uint32_t i = 0; i < num_axes; ++i)
	{
		Basalt_Vec3 axis = sat_axes[i];

		float box_radius = fabsf(axis.x) * half_sizes_a.x + fabsf(axis.y) * half_sizes_a.y + fabsf(axis.z) * half_sizes_a.z;

		float box_min = -box_radius;
		float box_max =  box_radius;

		float capsule_center = basalt_vec3Dot(capsule_b.center, axis);
		float capsule_proj = fabsf(basalt_vec3Dot(axis_b, axis));

		float capsule_min = capsule_center - half_height_b * capsule_proj - capsule_b.radius;
		float capsule_max = capsule_center + half_height_b * capsule_proj + capsule_b.radius;

		if (box_max < capsule_min || capsule_max < box_min)
			return BASALT_NO_INTERSECTION;

		float penetration = basalt_floatMin(box_max, capsule_max) - basalt_floatMax(box_min, capsule_min);
		if (penetration < min_penetration)
		{
			min_penetration = penetration;
			min_axis_sign = (capsule_min < box_min) ? 0 : 1;
			min_axis = i;
		}
	}

	uint32_t num_candidates = 0;
	float candidates[2] = {FLT_MAX, FLT_MAX};
	float plane[4] = {0};
	Basalt_ShapeFeature box_feature = {0};

	// capsule-face contact
	if (min_axis < 3)
	{
		int clip_axis0 = (min_axis + 1) % 3;
		int clip_axis1 = (min_axis + 2) % 3;

		float half_sizes[3] = {half_sizes_a.x, half_sizes_a.y, half_sizes_a.z};

		float clip_planes[5][4] = {0};
		clip_planes[0][min_axis] = (min_axis_sign == 0) ? -1.0f : 1.0f;
		clip_planes[1][clip_axis0] = -1.0f;
		clip_planes[2][clip_axis0] =  1.0f;
		clip_planes[3][clip_axis1] = -1.0f;
		clip_planes[4][clip_axis1] =  1.0f;

		clip_planes[0][3] = -half_sizes[min_axis] - capsule_b.radius;
		clip_planes[1][3] = -half_sizes[clip_axis0];
		clip_planes[2][3] = -half_sizes[clip_axis0];
		clip_planes[3][3] = -half_sizes[clip_axis1];
		clip_planes[4][3] = -half_sizes[clip_axis1];

		float tmin = -half_height_b;
		float tmax =  half_height_b;

		for (uint32_t i = 0; i < 5; ++i)
		{
			Basalt_Vec3 clip_normal = {clip_planes[i][0], clip_planes[i][1], clip_planes[i][2]};
			float clip_offset = clip_planes[i][3];

			float dot_clipped = basalt_vec3Dot(axis_b, clip_normal);
			if (dot_clipped == 0)
				continue;

			float t = (-basalt_vec3Dot(clip_normal, capsule_b.center) - clip_offset) / dot_clipped;

			if (dot_clipped > 0.0f)
				tmax = basalt_floatMin(tmax, t);
			else
				tmin = basalt_floatMax(tmin, t);
		}

		assert(tmin <= tmax);
		candidates[num_candidates++] = tmin;
		candidates[num_candidates++] = tmax;

		box_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_FACE, 2 * min_axis + min_axis_sign};
		for (uint32_t i = 0; i < 4; ++i)
			plane[i] = clip_planes[0][i];
	}
	// capsule-edge contact
	else
	{
		Basalt_Vec3 test_axis = sat_axes[min_axis];
		if (basalt_vec3Dot(test_axis, capsule_b.center) < 0.0f)
			test_axis = (Basalt_Vec3){-test_axis.x, -test_axis.y, -test_axis.z};

		float axis[3] = {test_axis.x, test_axis.y, test_axis.z};

		float signs[3] =
		{
			(axis[0] < 0.0f) ? -1.0f : 1.0f,
			(axis[1] < 0.0f) ? -1.0f : 1.0f,
			(axis[2] < 0.0f) ? -1.0f : 1.0f,
		};

		float half_sizes[3] = {half_sizes_a.x, half_sizes_a.y, half_sizes_a.z};

		uint32_t min_index = 0;
		float min_distance = fabsf(axis[0]);

		for (uint32_t i = 1; i < 2; ++i)
		{
			float distance = fabsf(axis[i]);
			if (min_distance > distance)
			{
				min_distance = distance;
				min_index = i;
			}
		}

		uint32_t index_1 = (min_index + 1) % 3;
		uint32_t index_2 = (min_index + 2) % 3;

		float edge_center[3] = {0};
		edge_center[index_1] = half_sizes[index_1] * signs[index_1];
		edge_center[index_2] = half_sizes[index_2] * signs[index_2];

		float edge_axis[3] = {0};
		edge_axis[min_index] = 1.0f;

		Basalt_Vec3 center_a = {edge_center[0], edge_center[1], edge_center[2]};
		Basalt_Vec3 axis_a = {edge_axis[0], edge_axis[1], edge_axis[2]};
		float half_height_a = half_sizes[min_index];

		Basalt_SegmentManifold segment_candidates = basalt_segmentSegmentIntersection(center_a, capsule_b.center, axis_a, axis_b, half_height_a, half_height_b);

		for (uint32_t i = 0; i < segment_candidates.num_results; ++i)
		{
			float t_a = segment_candidates.results_a[i];
			float t_b = segment_candidates.results_b[i];

			Basalt_Vec3 p_a = basalt_vec3Mad(axis_a, t_a, center_a);
			Basalt_Vec3 p_b = basalt_vec3Mad(axis_b, t_b, capsule_b.center);

			Basalt_Vec3 delta = basalt_vec3Sub(p_b, p_a);
			float r_sqr = capsule_b.radius * capsule_b.radius;

			float l_sqr = basalt_vec3Dot(delta, delta);
			if (l_sqr > r_sqr)
				continue;

			candidates[num_candidates++] = t_b;
		}

		assert(num_candidates > 0);

		uint32_t face1_id = (axis[index_1] < 0.0f) ? index_1 * 2 + 0 : index_1 * 2 + 1;
		uint32_t face2_id = (axis[index_2] < 0.0f) ? index_2 * 2 + 0 : index_2 * 2 + 1;
		uint32_t edge_id = face1_id + face2_id * 6;

		box_feature = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CONVEX_EDGE, edge_id};
		plane[0] = test_axis.x;
		plane[1] = test_axis.y;
		plane[2] = test_axis.z;
		plane[3] = -basalt_vec3Dot(test_axis, center_a);
	}

	manifold->num_contacts = 0;
	manifold->normal = (Basalt_Vec3){plane[0], plane[1], plane[2]};

	for (uint32_t i = 0; i < num_candidates; ++i)
	{
		float t = candidates[i];
		uint32_t index = (t == -half_height_b) ? 0 : (t == half_height_b) ? 1 : 2;

		Basalt_Vec3 p = basalt_vec3Mad(axis_b, t, capsule_b.center);
		float penetration = basalt_vec3Dot(manifold->normal, p) + plane[3];
		assert(penetration <= 0.0f);

		p = basalt_vec3Add(p, box_a.center);

		Basalt_Contact *contact = &manifold->contacts[manifold->num_contacts++];
		contact->position = basalt_vec3Mad(manifold->normal, penetration, p);
		contact->penetration = penetration;
		contact->feature_a = box_feature;
		contact->feature_b = (Basalt_ShapeFeature){BASALT_SHAPE_FEATURE_TYPE_CAPSULE_SURFACE, index};
	}

	assert(manifold->num_contacts > 0);
	return BASALT_SUCCESS;
}

static BASALT_INLINE Basalt_Result basalt_boxBoxIntersection(const Basalt_ShapeInfo *info_a, const Basalt_ShapeInfo *info_b, Basalt_Transform transform, Basalt_ContactManifold *manifold)
{
	return BASALT_NOT_IMPLEMENTED;
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

	Basalt_Result result = handler(info_a, info_b, transform, manifold);
	if (result != BASALT_SUCCESS)
		return result;

	// postprocess
	{
		manifold->normal = basalt_quatRotateVec3(transform_a.rotation, manifold->normal);

		for (uint32_t i = 0; i < manifold->num_contacts; ++i)
		{
			Basalt_Contact *contact = &manifold->contacts[i];
			contact->position = basalt_vec3Add(transform_a.position, basalt_quatRotateVec3(transform_a.rotation, contact->position));
		}
	}

	if (swapped)
	{
		manifold->normal = (Basalt_Vec3){-manifold->normal.x, -manifold->normal.y, -manifold->normal.z};

		for (uint32_t i = 0; i < manifold->num_contacts; ++i)
		{
			Basalt_Contact *contact = &manifold->contacts[i];

			Basalt_ShapeFeature temp = contact->feature_a;
			contact->feature_a = contact->feature_b;
			contact->feature_b = temp;
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
