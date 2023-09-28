/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "static_layer_setting.pb.h"
#include "static_layer.h"

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
  ros::NodeHandle nh;
  nh.getParam("self_color", robot_color_);
  is_current_ = true;
  ParaStaticLayer para_static_layer;

  std::string static_map = ros::package::getPath("roborts_costmap") + \
      "/config/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_-> GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);
  ros::Rate temp_rate(10);
  while(!map_received_) {
    ros::spinOnce();
    temp_rate.sleep();
  }
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;

  // left bottom positions
  bonus_penalty_position_[0][0] = 7.31; bonus_penalty_position_[0][1] = 1.45;
  bonus_penalty_position_[1][0] = 5.91; bonus_penalty_position_[1][1] = 2.59;
  bonus_penalty_position_[2][0] = 3.77; bonus_penalty_position_[2][1] = 0.20;
  bonus_penalty_position_[3][0] = 3.77; bonus_penalty_position_[3][1] = 3.79;
  bonus_penalty_position_[4][0] = 1.63; bonus_penalty_position_[4][1] = 1.41;
  bonus_penalty_position_[5][0] = 0.23; bonus_penalty_position_[5][1] = 2.55;
  for(int i = 0; i < 6; i++) game_zone_array_active_[i] = false;
  game_zone_sub_ = nh.subscribe("game_zone_array_status", 1, &StaticLayer::GameZoneArrayCallback, this);
  robot_status_sub_ = nh.subscribe("robot_status", 3, &StaticLayer::RobotStatusCallback, this);
}

void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
              master->GetOriginX(), master->GetOriginY());
  }
}

void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  //std::cout << "map width " << size_x << " " << size_y << std::endl;
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if(!layered_costmap_->IsRolling() && (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
      master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x || master_map->GetOriginY() != origin_y ||
      !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
  } else if(size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x || origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }
  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

// 原pgm图255值会被改成LETHAL_OBSTACLE（254）
unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() {
  OnInitialize();
}

void StaticLayer::Deactivate() {
//    delete cost_map_;
  //shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if(first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double *min_x,
                               double *min_y,
                               double *max_x,
                               double *max_y) {
  double wx, wy;
  if(!layered_costmap_->IsRollingWindow()) {
    if(!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  //just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_+ width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  //std::cout << "static updatebounds: " << *min_x << " " << *min_y << " " << *max_x << " " << *max_y << std::endl;
  has_updated_data_ = false;
}

void StaticLayer::GameZoneArrayCallback(const roborts_msgs::GameZoneArray &gamezone) {
  // std::cout << "Received game_zone_array_status!\ns";
  game_zone_array_received_ = true;
  has_updated_data_ = true;
  for(int i = 0; i < 6; i++) {
    game_zone_array_type_[robot_color_ == 1 ? (5 - i) : i] = gamezone.zone[i].type;
    game_zone_array_active_[robot_color_ == 1 ? (5 - i) : i] = gamezone.zone[i].active;
  }
}

void StaticLayer::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr& robot_status) {
  robot_color_ = robot_status -> id < 10 ? 0 : 1;
}

void StaticLayer::UpdateGameZoneByType(Costmap2D& master_grid, double min_x, double min_y, char value) {
  double max_x = min_x + 0.27 + bonus_fill_width_/ 2, max_y = min_y + 0.24 + bonus_fill_height_ / 2 ;
  // unsigned int min_cell_x, min_cell_y, max_cell_x, max_cell_y;
  unsigned int min_cell_x = min_x * 20, min_cell_y = min_y * 20, max_cell_x = max_x * 20, max_cell_y = max_y * 20;
  //World2Map(min_x, min_y, min_cell_x, min_cell_y);
  //World2Map(max_x, max_y, max_cell_x, max_cell_y);
  //std::cout << "min_cell_x = " << min_cell_x << " min_cel_y = " << min_cell_y << '\n';
  for(int j = min_cell_y; j <= max_cell_y; j++) {
    //std::cout << "size_x_ = " << size_x_ << "\n";
    for(int i = min_cell_x; i <= max_cell_x; i++){
      //master_grid.SetCost(i ,j, value);
      int index = GetIndex(i, j);
      //std::cout << "size_x_ " << size_x_ << std::endl;  
      costmap_[index] = value;
      //index ++;
    }
  }

}

char StaticLayer::DetermineCost(int robot_color, int game_zone_type) {
  return (robot_color_ == 1 && (game_zone_type == 3 || game_zone_type == 4)) || (robot_color_ == 0 && (game_zone_type == 1 || game_zone_type == 2 )) ? FREE_SPACE : LETHAL_OBSTACLE;
}

void StaticLayer::UpdateGameZoneCosts(Costmap2D& master_grid) {
  for(int i = 0; i < 6; i++) {
    //std::cout << game_zone_array_type_[i] << " " << game_zone_array_active_[i] << " " << bonus_penalty_position_[i][0] + 0.27 - bonus_fill_width_ / 2 << " " << bonus_penalty_position_[i][1] + 0.24 - bonus_fill_height_ /2 << std::endl;
    UpdateGameZoneByType(master_grid, bonus_penalty_position_[i][0] + 0.27 - bonus_fill_width_ / 2, bonus_penalty_position_[i][1] + 0.24 - bonus_fill_height_ /2, game_zone_array_active_[i] ? DetermineCost(robot_color_, game_zone_array_type_[i]): FREE_SPACE);
  }
  //std::cout << std::endl;
}

void StaticLayer::UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  if(!map_received_) {
    return;
  }

  if(game_zone_array_received_) UpdateGameZoneCosts(master_grid);

  // iisrollingWindows仅在local_plan中为true (其目的在于local_plan只需要其中一小块地图，且边界平行于大地图边界，此时需要对矩形区域做tf坐标转换再更新)
  if(!layered_costmap_->IsRollingWindow()) {
    if(!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      //std::cout << "use_maximum " << min_i << " " << min_j << " " << max_i << " " << max_j << std::endl;
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      // map_frame_为map， global_frame在local_plan中是odom，在global_plan中是map（local中为了实现边界平行的子地图，需要实现tf转换以计算在odom坐标系下的坐标）
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for(auto i = min_i; i < max_i; ++i) {
      for(auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if(World2Map(p.x(), p.y(), mx, my)){
          if(!use_maximum_) { //use_maximum_为true
            master_grid.SetCost(i, j, GetCost(mx, my));
          }
          else {
            master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

} //namespace roborts_costmap
