#include "Lib_Common.h"
#include "task.h"
void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
void *operator new(size_t size) throw(std::bad_alloc)
{
    return pvPortMalloc(size);
}

void operator delete(void *pointer) throw()
{
    vPortFree(pointer);
}