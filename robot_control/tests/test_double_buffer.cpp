#include "shared_data.h"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {

constexpr uint64_t kIterations = 300000;
constexpr uint64_t kGuard = 0x9e3779b97f4a7c15ULL;

struct Payload {
    uint64_t sequence = 0;
    uint64_t begin_guard = 0;
    uint64_t values[128] = {0};
    uint64_t end_guard = 0;
};
//jijijijijiijiji
bool validPayload(const Payload& payload) {
    if (payload.sequence == 0
            || payload.begin_guard != (payload.sequence ^ kGuard)
            || payload.end_guard != (payload.sequence + kGuard)) {
        return false;
    }
    for (uint64_t index = 0; index < 128; ++index) {
        if (payload.values[index] != payload.sequence * 131ULL + index)
            return false;
    }
    return true;
}

}  // namespace

int main() {
    DoubleBuffer<Payload> buffer;
    std::atomic<bool> producer_done{false};
    std::atomic<bool> failed{false};

    auto reader = [&](bool deliberately_slow) {
        DoubleBuffer<Payload>::Sequence cursor = 0;
        uint64_t previous_payload_sequence = 0;
        Payload snapshot{};
        while (!producer_done.load(std::memory_order_acquire)
                || previous_payload_sequence < kIterations) {
            if (!buffer.tryRead(snapshot, cursor)) {
                std::this_thread::yield();
                continue;
            }
            if (!validPayload(snapshot)
                    || snapshot.sequence <= previous_payload_sequence) {
                failed.store(true, std::memory_order_release);
                return;
            }
            previous_payload_sequence = snapshot.sequence;
            if (deliberately_slow && (snapshot.sequence % 97ULL) == 0)
                std::this_thread::yield();
        }
    };

    std::thread fast_reader(reader, false);
    std::thread slow_reader(reader, true);
    std::thread producer([&] {
        for (uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
            Payload& payload = buffer.acquireWriteSlot();
            payload.sequence = sequence;
            payload.begin_guard = sequence ^ kGuard;
            for (uint64_t index = 0; index < 128; ++index)
                payload.values[index] = sequence * 131ULL + index;
            payload.end_guard = sequence + kGuard;
            buffer.commitWrite();
        }
        producer_done.store(true, std::memory_order_release);
    });

    producer.join();
    fast_reader.join();
    slow_reader.join();

    if (failed.load(std::memory_order_acquire)) {
        std::cerr << "[FAIL] SnapshotBuffer出现撕裂、倒序或多读者干扰\n";
        return 1;
    }
    std::cout << "[PASS] SnapshotBuffer 300000帧双读者压力测试\n";
    return 0;
}
