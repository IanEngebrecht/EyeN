#pragma once

#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace rtos
{

template <size_t StackBytes>
class Task
{
  public:
    void create(const char *name, UBaseType_t priority, TaskFunction_t fn, void *arg, BaseType_t core)
    {
        handle_ = xTaskCreateStaticPinnedToCore(fn, name, StackBytes, arg, priority, stack_, &tcb_, core);
    }

    TaskHandle_t handle() const { return handle_; }

  private:
    StackType_t stack_[StackBytes]{};
    StaticTask_t tcb_{};
    TaskHandle_t handle_{};
};

} // namespace rtos
