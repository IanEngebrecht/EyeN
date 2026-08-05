#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace rtos
{

class Mutex
{
  public:
    void create() { handle_ = xSemaphoreCreateMutexStatic(&storage_); }

    bool lock(TickType_t timeout = portMAX_DELAY)
    {
        return xSemaphoreTake(handle_, timeout) == pdTRUE;
    }

    void unlock() { xSemaphoreGive(handle_); }

    SemaphoreHandle_t handle() const { return handle_; }

  private:
    StaticSemaphore_t storage_{};
    SemaphoreHandle_t handle_{};
};

class LockGuard
{
  public:
    explicit LockGuard(Mutex &m) : m_(m) { m_.lock(); }
    ~LockGuard() { m_.unlock(); }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

  private:
    Mutex &m_;
};

} // namespace rtos
