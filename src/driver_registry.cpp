#include "actuator_bridge/driver_registry.h"

namespace actuator_bridge {

DriverRegistry& DriverRegistry::instance() {
  static DriverRegistry inst;
  return inst;
}

void DriverRegistry::registerFactory(const std::string& vendor, FactoryFn fn) {
  map_[vendor] = std::move(fn);
}

std::shared_ptr<DriverBase> DriverRegistry::create(const std::string& vendor) const {
  auto it = map_.find(vendor);
  if (it == map_.end()) return nullptr;
  return it->second();
}

} // namespace actuator_bridge
