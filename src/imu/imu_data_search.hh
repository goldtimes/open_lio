#pragma once
#include "common/data_search.hh"
#include "common/interpolator.hh"
#include "sensors/imu.hh"

namespace lio {
// final 防止IMUDataSearcher类被继承
class IMUDataSearcher final : public DataSearcher<IMUData> {
   public:
    explicit IMUDataSearcher(size_t data_size) : DataSearcher<IMUData>(data_size) {
    }
    ~IMUDataSearcher() = default;

    // 获取指定时间段内的数据
    std::vector<IMUData> GetDataSegment(TimeStampUs time_left, TimeStampUs time_right) {
        std::vector<IMUData> imu_datas;
        std::lock_guard<std::mutex> lck(mtx_queue_);
        if (time_left >= time_right || data_queue_.empty()) {
            return imu_datas;
        }

        if (data_queue_.front().timestamped_ > time_left || data_queue_.back().timestamped_ < time_right) {
            return imu_datas;
        }
        // 插值
        IMUData data_left_inter, data_right_inter;
        // t_l-1,time_left, time_left+1----time_right-1, time_right, time_right + 1
        // 找到左边的起点，并需要做imu数据的插值
        size_t left_index = 0;
        size_t data_size = data_queue_.size();
        for (size_t i = 0; i < data_size; ++i) {
            if (time_left > data_queue_[i].timestamped_) {
                // 找到了左边的起点
                left_index = i;
                break;
            }
        }
        // 判断left_index,如果是末尾,肯定找不到了合适的数据了
        if (left_index == data_size - 1) {
            data_left_inter = data_queue_[left_index];
        } else {
            IMUData data_l, data_r;
            data_l = data_queue_[left_index - 1];
            data_r = data_queue_[left_index];
            // 插值
            data_left_inter.acc_ = Interpolator::InterpolateVectorLerp(data_l.acc_, data_r.acc_, data_l.timestamped_,
                                                                       data_r.timestamped_, time_left);
            data_left_inter.gyro_ = Interpolator::InterpolateVectorLerp(data_l.gyro_, data_r.gyro_, data_l.timestamped_,
                                                                        data_r.timestamped_, time_left);
            data_left_inter.timestamped_ = time_left;
            // orientatio slerp
        }
        // 找到右边的终点，并需要做imu数据的插值
        size_t right_index = 0;
        for (size_t j = data_size - 1; j > 0; --j) {
            if (time_right > data_queue_[j].timestamped_) {
                right_index = j;
                break;
            }
        }
        if (right_index == data_size - 1) {
            data_right_inter = data_queue_[right_index];
        } else {
            IMUData data_l, data_r;
            data_l = data_queue_[right_index];
            data_r = data_queue_[right_index + 1];
            // 插值
            data_right_inter.acc_ = Interpolator::InterpolateVectorLerp(data_l.acc_, data_r.acc_, data_l.timestamped_,
                                                                        data_r.timestamped_, time_right);
            data_right_inter.gyro_ = Interpolator::InterpolateVectorLerp(
                data_l.gyro_, data_r.gyro_, data_l.timestamped_, data_r.timestamped_, time_right);
            data_right_inter.timestamped_ = time_right;
        }
        // left->right之间的数据
        imu_datas.push_back(data_left_inter);
        while (left_index != right_index) {
            imu_datas.push_back(data_queue_[left_index]);
            left_index++;
        }
        imu_datas.push_back(data_right_inter);
        return imu_datas;
    }
};
}  // namespace lio