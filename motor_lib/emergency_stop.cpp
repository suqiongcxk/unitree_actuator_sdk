#include "emergency_stop.h"

#include <atomic>

namespace {
std::atomic<bool> g_emergency_stop{false};
}

void requestEmergencyStop() noexcept
{
    g_emergency_stop.store(true, std::memory_order_release);
}

bool isEmergencyStopRequested() noexcept
{
    return g_emergency_stop.load(std::memory_order_acquire);
}

void resetEmergencyStop() noexcept
{
    g_emergency_stop.store(false, std::memory_order_release);
}
