#pragma once
#include "registration/icp_matcher.hh"
#include "system/SystemManager.hh"

namespace lio {
class Localization {
   public:
    explicit Localization(SystemManager* sys) : sys_(sys) {
    }
    void Run();

   private:
    SystemManager* sys_ = nullptr;
};
}  // namespace lio