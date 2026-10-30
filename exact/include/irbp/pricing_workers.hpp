/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace RouteOpt::Application::IRBP {
    class PricingWorkers {
        std::vector<std::thread> threads;
        std::mutex mutex;
        std::condition_variable ready, done;
        std::function<void(int, int)> task;
        std::atomic<int> next{};
        std::atomic<bool> cancelled{};
        int count{}, pending{}, generation{};
        bool stopping{};
        std::exception_ptr failure;

        void execute(int worker) noexcept {
            try {
                while (!cancelled.load(std::memory_order_relaxed)) {
                    int job = next.fetch_add(1, std::memory_order_relaxed);
                    if (job >= count)
                        break;
                    task(worker, job);
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!failure)
                    failure = std::current_exception();
                cancelled.store(true, std::memory_order_relaxed);
            }
        }

      public:
        explicit PricingWorkers(int workers) {
            try {
                for (int id = 1; id < workers; ++id)
                    threads.emplace_back([this, id] {
                        int seen = 0;
                        std::unique_lock<std::mutex> lock(mutex);
                        while (true) {
                            ready.wait(lock, [&] { return stopping || generation != seen; });
                            if (stopping)
                                return;
                            seen = generation;
                            lock.unlock();
                            execute(id);
                            lock.lock();
                            if (--pending == 0)
                                done.notify_one();
                        }
                    });
            } catch (...) {
                stop();
                throw;
            }
        }
        ~PricingWorkers() { stop(); }
        PricingWorkers(const PricingWorkers &) = delete;
        PricingWorkers &operator=(const PricingWorkers &) = delete;

        void stop() noexcept {
            {
                std::lock_guard<std::mutex> lock(mutex);
                stopping = true;
            }
            ready.notify_all();
            for (auto &thread : threads)
                if (thread.joinable())
                    thread.join();
        }
        void run(int jobs, std::function<void(int, int)> work) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                task = std::move(work);
                count = jobs;
                next = 0;
                cancelled = false;
                failure = nullptr;
                pending = threads.size();
                ++generation;
            }
            ready.notify_all();
            execute(0);
            std::unique_lock<std::mutex> lock(mutex);
            done.wait(lock, [&] { return pending == 0; });
            task = {};
            if (failure)
                std::rethrow_exception(failure);
        }
    };
} // namespace RouteOpt::Application::IRBP
