#ifndef SAFE_THREAD_QUEUE_HPP
#define SAFE_THREAD_QUEUE_HPP

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <utility>

namespace tools
{

template<typename T, bool PopWhenFull = true>
class ThreadSafeQueue
{
public:
  explicit ThreadSafeQueue(
    std::size_t max_size,
    std::function<void()> full_handler = {})
  : max_size_(max_size),
    full_handler_(std::move(full_handler))
  {
  }

  bool push(T value)
  {
    bool queue_full = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (closed_) {
        return false;
      }

      if (max_size_ == 0) {
        queue_full = true;
      } else if (queue_.size() >= max_size_) {
        if constexpr (PopWhenFull) {
          queue_.pop();
        } else {
          queue_full = true;
        }
      }

      if (!queue_full) {
        queue_.push(std::move(value));
      }
    }

    if (queue_full) {
      if (full_handler_) {
        full_handler_();
      }
      return false;
    }

    not_empty_condition_.notify_one();
    return true;
  }

  // 返回 false 表示队列已经关闭且没有剩余任务。
  bool waitPop(T &value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] {
      return closed_ || !queue_.empty();
    });

    if (queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }

    not_empty_condition_.notify_all();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    while (!queue_.empty()) {
      queue_.pop();
    }
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  std::queue<T> queue_;
  const std::size_t max_size_;

  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void()> full_handler_;

  bool closed_{false};
};

}  // namespace tools

#endif  // SAFE_THREAD_QUEUE_HPP