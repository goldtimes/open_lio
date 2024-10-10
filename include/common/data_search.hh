#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <type_traits>
#include <vector>
#include "sensors/data.hh"

namespace lio {
template <typename DataType>
class DataSearcher {
   public:
    static_assert(std::is_base_of<DataBase, DataType>::value, "The DataType must inherit from DataBase");

    explicit DataSearcher(size_t data_size) : data_size_(data_size) {
    }

    explicit DataSearcher(const std::vector<DataType>& datas) {
        data_size_ = datas.size();
        for (const auto data : datas) {
            CacheData(data);
        }
    }

    ~DataSearcher() = default;

    bool CacheData(const DataType& data) {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (!data_queue_.empty() && data.timestamped_ < data_queue_.back().timestamped_) {
            return false;
        }

        data_queue_.emplace_back(std::move(data));

        if (data_queue_.size() > data_size_) {
            data_queue_.pop_front();
        }
        return true;
    }

    /**
     * @brief 找到time最接近的数据
     * 如果time 比最老的数据还老，那么直接返回false
     * 如果time 比最新的数据还要新，那么直接返回true
     */
    bool SearchNearestData(const uint64_t time, DataType& data) {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (time < data_queue_.front().timestamped_) {
            return false;
        }
        if (time > data_queue_.back().timestamped_) {
            data = data_queue_.back();
            return true;
        }
        auto r_it = data_queue_.rbegin();
        // 从尾部开始遍历，找到第一个小于time的时间
        while (time < r_it->timestamped_) {
            r_it++;
        }
        auto r_it_next = r_it - 1;
        // 根据delta_time找到最近的数据
        if ((time - r_it->timestamped_) < (r_it_next->timestamped_ - time)) {
            data = *r_it;
        } else {
            data = *r_it_next;
        }
        return true;
    }

    bool SearchNearestTwoData(const uint64_t time, DataType& data_l, DataType& data_r) {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (data_queue_.empty()) {
            return false;
        }
        if (time < data_queue_.front().timestamped_ || time > data_queue_.back().timestamped_) {
            return false;
        }

        if (time == data_queue_.front().timestamped_) {
            data_l = *data_queue_.begin();
            data_r = *(data_queue_.begin() + 1);
            return true;
        }
        if (time == data_queue_.back().timestamped_) {
            data_r = *data_queue_.rbegin();
            data_l = *(data_queue_.rbegin() + 1);
            return true;
        }
        // time 在 beign-end之间
        auto r_it = data_queue_.rbegin();
        while (time < r_it->timestamped_) {
            r_it++;
        }
        auto r_it_next = r_it - 1;
        data_l = *r_it;
        data_r = *r_it_next;
        return true;
    }

    bool IsInBuffer(const uint64_t& data_time) {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (data_queue_.empty()) {
            return false;
        }
        if (data_time > data_queue_.front().timestamped_ && data_time < data_queue_.back().timestamped_) {
            return true;
        }
        return false;
    }

    std::optional<DataType> LatestData() {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (data_queue_.empty()) {
            return {};
        } else {
            return data_queue_.back();
        }
    }

    std::optional<DataType> OldestData() {
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (data_queue_.empty()) {
            return {};
        } else {
            return data_queue_.front();
        }
    }

   protected:
    std::deque<DataType> data_queue_;
    std::mutex mtx_queue_;
    size_t data_size_;
};
}  // namespace lio