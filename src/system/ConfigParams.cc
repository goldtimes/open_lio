#include "system/ConfigParams.hh"

namespace lio {
std::unique_ptr<ConfigParams> ConfigParams::instance_ = nullptr;

ConfigParams& ConfigParams::GetInstance() {
    if (instance_ == nullptr) {
        std::once_flag flag;
        std::call_once(flag, [&]() { instance_.reset(new (std::nothrow) ConfigParams()); });
    }
    return *instance_;
}

}  // namespace lio