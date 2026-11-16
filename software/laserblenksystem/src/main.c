#include "app/application.h"
#include "board/timebase.h"

int main(void)
{
    application_initialize();

    for (;;)
    {
        application_service();
        timebase_idle();
    }
}
