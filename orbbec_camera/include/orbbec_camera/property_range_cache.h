/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#pragma once

#include <libobsensor/h/Property.h>

#include <map>
#include <mutex>

namespace orbbec_camera {

// Cache range metadata only. A range's SDK "cur" field is not a live property value.
// Failed queries are not cached, so a transient SDK error can recover on the next request.
template <class Range>
class PropertyRangeCache {
 public:
  template <class Query>
  Range get(OBPropertyID id, Query query) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = ranges_.find(id);
    if (found != ranges_.end()) return found->second;
    const auto range = query();
    ranges_.emplace(id, range);
    return range;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    ranges_.clear();
  }

 private:
  std::mutex mutex_;
  std::map<OBPropertyID, Range> ranges_;
};
}  // namespace orbbec_camera
