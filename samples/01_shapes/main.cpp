#include <basalt.h>
#include <cassert>
#include <iostream>

void testShapes(Basalt_Instance instance)
{
	Basalt_Shape shape = BASALT_NULL_HANDLE;
	Basalt_Vec3 center = {0.0f, 0.0f, 0.0f};
	
	Basalt_Result result = basaltCreateShapeSphere(instance, center, 1.0f, &shape);
	assert(result == BASALT_SUCCESS);
	
	Basalt_Transform shape_transform = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	Basalt_Vec3 point = {0.5f, 0.0f, 0.0f};
	Basalt_ContactManifold manifold = {};

	result = basaltShapeIntersectPoint(instance, shape, shape_transform, point, &manifold);
	assert(result == BASALT_SUCCESS);
	assert(fabs(manifold.normal.x - 1.0f) < 1e-06);
	assert(manifold.normal.y == 0.0f);
	assert(manifold.normal.z == 0.0f);
	assert(manifold.num_contacts == 1);
	assert(fabs(manifold.contacts->penetration + 0.5f) < 1e-06);

	result = basaltDestroyShape(instance, shape);
	assert(result == BASALT_SUCCESS);
}

int main()
{
	Basalt_Instance instance = BASALT_NULL_HANDLE;

	Basalt_InstanceDesc instance_desc =
	{
	};

	Basalt_Result result = basaltCreateInstance(&instance_desc, &instance);
	assert(result == BASALT_SUCCESS);

	testShapes(instance);

	result = basaltDestroyInstance(instance);
	assert(result == BASALT_SUCCESS);

	return 0;
}
