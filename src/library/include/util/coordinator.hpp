// Copyright (C) 2020-2024 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#ifndef MAP_MATCHING_2_UTIL_COORDINATOR_HPP
#define MAP_MATCHING_2_UTIL_COORDINATOR_HPP

#include <semaphore>

#include <boost/thread.hpp>
#include <boost/thread/executors/basic_thread_pool.hpp>
#include <boost/thread/concurrent_queues/sync_bounded_queue.hpp>

namespace map_matching_2::util {

    template<typename Task>
    class coordinator {

    public:
        using task_type = Task;

        explicit coordinator(const std::uint32_t threads = boost::thread::hardware_concurrency())
            : coordinator{threads, threads == 1 ? threads : 2 * threads} {}

        coordinator(const std::uint32_t threads, const std::uint32_t queue_size)
            : _single_threaded{threads == 1}, _queue{static_cast<std::size_t>(queue_size)},
            _semaphore{static_cast<std::ptrdiff_t>(queue_size)}, _thread_pool{threads} {}

        virtual ~coordinator() {
            if (not _queue.closed()) {
                _queue.close();
            }
            _coordinator.join();
        }

        void queue(const task_type &task) {
            _queue.wait_push_back(task);
        }

        void queue(task_type &&task) {
            _queue.wait_push_back(std::move(task));
        }

        void start() {
            _coordinator = boost::thread([this]() {
                _start();
            });
        }

        void join() {
            _coordinator.join();
        }

        void stop() {
            _queue.close();
        }

    protected:
        bool _single_threaded;
        boost::thread _coordinator;
        boost::sync_bounded_queue<task_type> _queue;
        std::counting_semaphore<> _semaphore;
        boost::basic_thread_pool _thread_pool;

        virtual void execute(task_type &&task) = 0;

        void _start() {
            while (not _queue.empty() or not _queue.closed()) {
                task_type task;
                auto queue_op = _queue.wait_pull_front(task);
                if (queue_op == boost::queue_op_status::success) {
                    _semaphore.acquire();
                    boost::async(_thread_pool, [this, task = std::move(task)]() mutable {
                        try {
                            execute(std::move(task));
                            _semaphore.release();
                        } catch (...) {
                            _semaphore.release();
                            throw;
                        }
                    });
                }
            }

            _thread_pool.close();
            _thread_pool.join();
        }

    };

}

#endif //MAP_MATCHING_2_UTIL_COORDINATOR_HPP
