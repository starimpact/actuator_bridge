#pragma once
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include "actuator_bridge/driver_base.h"

namespace actuator_bridge {

// A simple registry for Driver factories keyed by vendor name.
class DriverRegistry {
 public:
  using FactoryFn = std::function<std::shared_ptr<DriverBase>()>;

  static DriverRegistry& instance();

  // Register a factory for a vendor. Overwrites existing.
  void registerFactory(const std::string& vendor, FactoryFn fn);

  // Create a driver by vendor. Returns nullptr if not found.
  std::shared_ptr<DriverBase> create(const std::string& vendor) const;

 private:
  std::unordered_map<std::string, FactoryFn> map_;
};

// Helper RAII registrar used from driver implementation files.
class DriverRegistrar {
 public:
  DriverRegistrar(const std::string& vendor, DriverRegistry::FactoryFn fn) {
    DriverRegistry::instance().registerFactory(vendor, fn);
  }
};

} // namespace actuator_bridge
