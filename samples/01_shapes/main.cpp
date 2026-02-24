#include <basalt.h>
#include <cassert>
#include <iostream>

void testShapes(Basalt_Instance instance)
{
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
