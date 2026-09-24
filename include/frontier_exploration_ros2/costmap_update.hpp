/*
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <cstddef>
#include <memory>

#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace frontier_exploration_ros2
{

// Returns a patched copy of `grid`, or nullptr when the update does not fit it.
// The copy matters: the core detects costmap changes by comparing grid contents,
// so patching the grid it already holds would hide the change.
[[nodiscard]] inline nav_msgs::msg::OccupancyGrid::SharedPtr apply_occupancy_grid_update(
  const nav_msgs::msg::OccupancyGrid & grid,
  const map_msgs::msg::OccupancyGridUpdate & update)
{
  const auto & info = grid.info;
  const std::size_t x0 = update.x < 0 ? 0U : static_cast<std::size_t>(update.x);
  const std::size_t y0 = update.y < 0 ? 0U : static_cast<std::size_t>(update.y);
  const std::size_t width = update.width;
  const std::size_t height = update.height;
  if (update.x < 0 || update.y < 0 ||
    x0 + width > info.width || y0 + height > info.height ||
    update.data.size() != width * height ||
    grid.data.size() != static_cast<std::size_t>(info.width) * info.height)
  {
    return nullptr;
  }

  auto patched = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
  patched->header.stamp = update.header.stamp;
  for (std::size_t row = 0; row < height; ++row) {
    const auto src = update.data.begin() + static_cast<std::ptrdiff_t>(row * width);
    const auto dst = patched->data.begin() +
      static_cast<std::ptrdiff_t>((y0 + row) * info.width + x0);
    std::copy(src, src + static_cast<std::ptrdiff_t>(width), dst);
  }
  return patched;
}

}  // namespace frontier_exploration_ros2
