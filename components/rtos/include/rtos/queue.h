#pragma once

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace rtos
{

template <typename T, size_t Depth>
class Queue
{
  public:
    void create() { handle_ = xQueueCreateStatic(Depth, sizeof(T), storage_, &control_); }

    bool send(const T &item, TickType_t timeout) { return xQueueSend(handle_, &item, timeout) == pdTRUE; }

    bool receive(T &item, TickType_t timeout) { return xQueueReceive(handle_, &item, timeout) == pdTRUE; }

    bool overwrite(const T &item)
    {
        static_assert(Depth == 1, "overwrite requires depth 1");
        return xQueueOverwrite(handle_, &item) == pdTRUE;
    }

    QueueHandle_t handle() const { return handle_; }

  private:
    uint8_t storage_[Depth * sizeof(T)]{};
    StaticQueue_t control_{};
    QueueHandle_t handle_{};
};

} // namespace rtos
