#pragma once

#include "basalt_internal.h"

#include "common/pool.h"

typedef struct Impl_Instance_t
{
	Basalt_InstanceTable *vtbl;
	Basalt_Pool shapes;
} Impl_Instance;

typedef struct Impl_Shape_t
{
	Basalt_ShapeInfo info;
} Impl_Shape;
