#pragma once

#include <stdint.h> // TODO: get rid of this dependency later

// Version
#define BASALT_VERSION_MAJOR 1
#define BASALT_VERSION_MINOR 0
#define BASALT_VERSION_PATCH 0
#define BASALT_VERSION "1.0.0-dev"

// Platform specific defines
#if defined(_WIN32)
	#define BASALT_EXPORT	__declspec(dllexport)
	#define BASALT_IMPORT	__declspec(dllimport)
	#define BASALT_INLINE	__forceinline
	#define BASALT_RESTRICT	__restrict
#else
	#define BASALT_EXPORT	__attribute__((visibility("default")))
	#define BASALT_IMPORT
	#define BASALT_INLINE	__inline__
	#define BASALT_RESTRICT	__restrict
#endif

#if defined(BASALT_SHARED_LIBRARY)
	#define BASALT_APIENTRY extern BASALT_EXPORT
#else
	#define BASALT_APIENTRY extern BASALT_IMPORT
#endif

#if !defined(BASALT_NULL_HANDLE)
	#define BASALT_NULL_HANDLE 0
#endif

#define BASALT_DEFINE_HANDLE(TYPE) typedef uint64_t TYPE

#ifdef __cplusplus
extern "C" {
#endif

// Constants

// Opaque handles
BASALT_DEFINE_HANDLE(Basalt_Instance);
BASALT_DEFINE_HANDLE(Basalt_Shape);

// Enums
typedef enum Basalt_Result_t
{
	BASALT_SUCCESS = 0,
	BASALT_NOT_IMPLEMENTED,
	BASALT_INVALID_INSTANCE,
	BASALT_INVALID_OUTPUT_ARGUMENT,

	// FIXME: add more error codes for internal errors
	BASALT_INTERNAL_ERROR,

	BASALT_RESULT_ENUM_MAX,
	BASALT_RESULT_ENUM_FORCE32 = 0x7FFFFFFF,
} Basalt_Result;

typedef enum Basalt_ShapeType_t
{
	BASALT_SHAPE_TYPE_SPHERE = 0,
	BASALT_SHAPE_TYPE_CAPSULE,
	BASALT_SHAPE_TYPE_BOX,

	BASALT_SHAPE_TYPE_ENUM_MAX,
	BASALT_SHAPE_TYPE_ENUM_FORCE32 = 0x7FFFFFFF,
} Basalt_ShapeType;

typedef enum Basalt_CapsuleAxis_t
{
	BASALT_CAPSULE_AXIS_X = 0,
	BASALT_CAPSULE_AXIS_Y,
	BASALT_CAPSULE_AXIS_Z,

	BASALT_CAPSULE_AXIS_ENUM_MAX,
	BASALT_CAPSULE_AXIS_ENUM_FORCE32 = 0x7FFFFFFF,
} Basalt_CapsuleAxis;

// Structs
typedef struct Basalt_Vec3_t
{
	float x, y, z;
} Basalt_Vec3;

typedef struct Basalt_Vec4_t
{
	float x, y, z, w;
} Basalt_Vec4;

typedef struct Basalt_Quat_t
{
	float x, y, z, w;
} Basalt_Quat;

typedef struct Basalt_InstanceDesc_t
{
	uint32_t reserved;
	// TODO: allocator context
	// TOOD: flags?
} Basalt_InstanceDesc;

// Function pointers
typedef Basalt_Result (*PFN_basaltDestroyInstance)(Basalt_Instance instance);
typedef Basalt_Result (*PFN_basaltCreateShapeSphere)(Basalt_Instance instance, Basalt_Vec3 center, float radius, Basalt_Shape *shape);
typedef Basalt_Result (*PFN_basaltCreateShapeCapsule)(Basalt_Instance instance, Basalt_Vec3 center, float radius, float height, Basalt_CapsuleAxis axis, Basalt_Shape *shape);
typedef Basalt_Result (*PFN_basaltCreateShapeBox)(Basalt_Instance instance, Basalt_Vec3 center, Basalt_Vec3 sizes, Basalt_Shape *shape);
typedef Basalt_Result (*PFN_basaltGetPenetration)(Basalt_Instance instance, Basalt_Shape shape, Basalt_Vec3 point, Basalt_Vec4 *penetration);
typedef Basalt_Result (*PFN_basaltDestroyShape)(Basalt_Instance instance, Basalt_Shape shape);


typedef struct Basalt_InstanceTable_t
{
	PFN_basaltCreateShapeSphere createShapeSphere;
	PFN_basaltCreateShapeCapsule createShapeCapsule;
	PFN_basaltCreateShapeBox createShapeBox;

	PFN_basaltGetPenetration getPenetration;

	PFN_basaltDestroyShape destroyShape;
	PFN_basaltDestroyInstance destroyInstance;
} Basalt_InstanceTable;

// API
#if !defined(BASALT_NO_PROTOTYPES)
BASALT_APIENTRY Basalt_Result basaltCreateInstance(const Basalt_InstanceDesc *desc, Basalt_Instance* instance);
BASALT_APIENTRY Basalt_Result basaltGetInstanceTable(Basalt_Instance instance, Basalt_InstanceTable *instance_table);

BASALT_APIENTRY Basalt_Result basaltCreateShapeSphere(Basalt_Instance instance, Basalt_Vec3 center, float radius, Basalt_Shape *shape);
BASALT_APIENTRY Basalt_Result basaltCreateShapeCapsule(Basalt_Instance instance, Basalt_Vec3 center, float radius, float height, Basalt_CapsuleAxis axis, Basalt_Shape *shape);
BASALT_APIENTRY Basalt_Result basaltCreateShapeBox(Basalt_Instance instance, Basalt_Vec3 center, Basalt_Vec3 sizes, Basalt_Shape *shape);

BASALT_APIENTRY Basalt_Result basaltGetPenetration(Basalt_Instance instance, Basalt_Shape shape, Basalt_Vec3 point, Basalt_Vec4 *penetration);

BASALT_APIENTRY Basalt_Result basaltDestroyShape(Basalt_Instance instance, Basalt_Shape shape);
BASALT_APIENTRY Basalt_Result basaltDestroyInstance(Basalt_Instance instance);
#endif

#ifdef __cplusplus
}
#endif
