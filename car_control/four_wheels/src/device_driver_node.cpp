#include "device_driver.h"

int main(int argc, char **argv)
{
    device_driver device_driver_node(argc, argv);

	device_driver_node.main();

    return 0;
}

