#pragma once

#include <basalt.h>

#define BASALT_UNUSED(x) do { (void)(x); } while(0)

Basalt_Result impl_createInstance(const Basalt_InstanceDesc *desc, Basalt_Instance *instance);
