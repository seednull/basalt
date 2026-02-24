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

// Structs
typedef struct Basalt_Vec3_t
{
	float x, y, z;
} Basalt_Vec3;

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


typedef struct Basalt_InstanceTable_t
{
	PFN_basaltDestroyInstance destroyInstance;
} Basalt_InstanceTable;

// API
#if !defined(BASALT_NO_PROTOTYPES)
BASALT_APIENTRY Basalt_Result basaltCreateInstance(const Basalt_InstanceDesc *desc, Basalt_Instance* instance);
BASALT_APIENTRY Basalt_Result basaltGetInstanceTable(Basalt_Instance instance, Basalt_InstanceTable *instance_table);

BASALT_APIENTRY Basalt_Result basaltDestroyInstance(Basalt_Instance instance);
#endif

#ifdef __cplusplus
}
#endif
