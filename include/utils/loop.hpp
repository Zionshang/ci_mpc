#pragma once
#include <chrono>
#include <thread>
#include <boost/function.hpp>
#include <iostream>

namespace loop
{
    /**
     * @brief A timer to ensure the frequency of thread execution.
     */
    class Timer
    {
    public:
        /**
         * @brief Construct a new Timer object
         * @param period_sec time, Unit: second
         */
        Timer(double period_sec) : _period(period_sec) { start(); }

        double period() { return _period; }

        /**
         * @brief Update the beginning time.
         */
        void start() { start_time_ = std::chrono::steady_clock::now(); }

        /**
         * @brief Caculate the time from the beginning to the present.
         * @return second
         */
        double elasped_time()
        {
            auto elasped_time = std::chrono::steady_clock::now() - start_time_;
            size_t t = std::chrono::duration_cast<std::chrono::microseconds>(elasped_time).count();
            return (double)t / 1000000.0;
        }

        /**
         * @brief Caculate the remaining time in a period
         * @return second
         */
        double wait_time() { return _period - elasped_time(); }

        /**
         * @brief Sleep for wait_time() until a period finished. If it has timeout, do nothing.
         */
        void sleep()
        {
            double waitTime = wait_time();
            if (waitTime > 0)
                std::this_thread::sleep_for(std::chrono::microseconds(size_t(waitTime * 1000000)));
            start(); // If the last one ends and then start a new timer.
        }

    private:
        double _period;
        std::chrono::steady_clock::time_point start_time_;
    };

    /**
     * @brief Maintains a thread to run once every period.
     */
    class LoopFunc
    {
    public:
        /**
         * @brief Construct a new Loop object
         * @param name Indicate what the loop aims to
         * @param period time, Unit: second
         * @param callback the running function pointer
         */
        LoopFunc(std::string name, double period, std::function<void()> callback)
            : _name(name), _cbFunction(callback) { _timer = std::make_shared<Timer>(period); }
        ~LoopFunc() { shutdown(); }

        void start()
        {
            if (!_isrunning)
            {
                _isrunning = true;
                _thread = std::thread(&LoopFunc::running_impl, this);
            }
        }

    private:
        void spinOnce()
        {
            _timer->start();
            ++_run_times;
            _cbFunction();

            if (_timer->wait_time() > 0)
                _timer->sleep();
            else
                ++_timeout_times;
        }

        void running_impl()
        {
            while (_isrunning)
            {
                spinOnce();
            }
        }

        void shutdown()
        {
            if (_isrunning)
            {
                _isrunning = false;
                _thread.join();
            }
            std::cout << "run times of " << _name << ":\t" << _run_times << std::endl;
            std::cout << "timeout times of " << _name << ":\t" << _timeout_times << std::endl;
        }
        
        std::string _name{};
        bool _isrunning = false;
        std::shared_ptr<Timer> _timer;

        std::function<void()> _cbFunction;
        std::thread _thread;

        size_t _run_times = 0;
        size_t _timeout_times = 0;
    };
} // namespace idto