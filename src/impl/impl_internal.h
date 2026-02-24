#pragma once

#include "basalt_internal.h"

#include "common/pool.h"

typedef struct Impl_Instance_t
{
	Basalt_InstanceTable *vtbl;
	Basalt_Pool shapes;
} Impl_Instance;

typedef struct Impl_ShapeSphere_t
{
	Basalt_Vec3 center;
	float radius;
} Impl_ShapeSphere;

typedef struct Impl_ShapeCapsule_t
{
	Basalt_Vec3 center;
	float radius;
	float height;
	Basalt_CapsuleAxis axis;
} Impl_ShapeCapsule;

typedef struct Impl_ShapeBox_t
{
	Basalt_Vec3 center;
	Basalt_Vec3 sizes;
} Impl_ShapeBox;

typedef union Impl_ShapeData_t
{
	Impl_ShapeSphere sphere;
	Impl_ShapeCapsule capsule;
	Impl_ShapeBox box;
} Impl_ShapeData;

typedef struct Impl_Shape_t
{
	Basalt_ShapeType type;
	Impl_ShapeData data;
} Impl_Shape;
