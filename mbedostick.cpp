#include "mbedostick.h"
#include "mbed.h"

extern Timer timeKeeper;

uint32_t elapseTimeMS()
{

    return (timeKeeper.elapsed_time().count())/1000;

}