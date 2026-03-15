#include "base/Base.hpp"

#include <iostream>

namespace base
{

bool Base::_initialize()
{
  return initialize();
}

bool Base::_execute()
{
  if (failed_) {
    return false;
  }

  return execute();
}

void Base::fail(const std::string & reason)
{
  std::cerr << "Base node has failed. Reason: " << reason << std::endl;
  failed_ = true;
}

}  // namespace base

