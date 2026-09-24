#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "frontier_exploration_ros2/costmap_update.hpp"

namespace frontier_exploration_ros2
{
namespace
{

nav_msgs::msg::OccupancyGrid make_grid(uint32_t width, uint32_t height)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.05F;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

map_msgs::msg::OccupancyGridUpdate make_update(
  int32_t x, int32_t y, uint32_t width, uint32_t height, int8_t value)
{
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.stamp.sec = 42;
  update.x = x;
  update.y = y;
  update.width = width;
  update.height = height;
  update.data.assign(static_cast<std::size_t>(width) * height, value);
  return update;
}

TEST(CostmapUpdate, PatchesTheUpdatedWindowInACopy)
{
  const auto grid = make_grid(5, 4);
  const auto patched = apply_occupancy_grid_update(grid, make_update(1, 2, 3, 2, 100));

  ASSERT_NE(patched, nullptr);
  const std::vector<int8_t> expected{
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 100, 100, 100, 0,
    0, 100, 100, 100, 0,
  };
  EXPECT_EQ(patched->data, expected);
  EXPECT_EQ(patched->header.stamp.sec, 42);
  // The source grid is untouched so the core still sees the change.
  EXPECT_EQ(grid.data, std::vector<int8_t>(20, 0));
}

TEST(CostmapUpdate, RejectsUpdatesThatDoNotFitTheGrid)
{
  const auto grid = make_grid(5, 4);

  EXPECT_EQ(apply_occupancy_grid_update(grid, make_update(3, 0, 3, 1, 100)), nullptr);
  EXPECT_EQ(apply_occupancy_grid_update(grid, make_update(0, 3, 1, 2, 100)), nullptr);
  EXPECT_EQ(apply_occupancy_grid_update(grid, make_update(-1, 0, 1, 1, 100)), nullptr);

  auto short_data = make_update(0, 0, 2, 2, 100);
  short_data.data.pop_back();
  EXPECT_EQ(apply_occupancy_grid_update(grid, short_data), nullptr);
}

}  // namespace
}  // namespace frontier_exploration_ros2
