/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_engine.h"
#include <iostream>


#include <utility>

//#include "modules/common/util/file.h"
//#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
//#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

//using apollo::common::util::GetAbsolutePath;

// void check_mod(bool b) {
//   if (!b) { std::cout<<"check_mod FAILED!!!"<<std::endl; exit(0); }
// }

bool MlfEngine::Init() {
  //auto config_manager = lib::ConfigManager::Instance();
  //const lib::ModelConfig* model_config = nullptr;
  //check_mod(config_manager->GetModelConfig(Name(), &model_config));
  //const std::string work_root = config_manager->work_root();
  //std::string config_file;
  //std::string root_path;
  //check_mod(model_config->get_value("root_path", &root_path));
  //config_file = GetAbsolutePath(work_root, root_path);
  //config_file = GetAbsolutePath(config_file, "mlf_engine.conf");
  //MlfEngineConfig config;
//  check_mod(apollo::common::util::GetProtoFromFile(config_file, &config));

  //done-todoConf
  int mainSensorSize = 1;
  main_sensor_.clear();
  for (int i = 0; i < mainSensorSize; ++i) {
    main_sensor_.emplace("velodyne64"); //done-todoConf
  }

  //done-todoConf use_histogram_for_match_ = config.use_histogram_for_match();
  use_histogram_for_match_ = true;
  //done-todoConf histogram_bin_size_ = config.histogram_bin_size();
  histogram_bin_size_ = 10;
  //done-todoConf output_predict_objects_ = config.output_predict_objects();
  output_predict_objects_ = false;
  //done-todoConf reserved_invisible_time_ = config.reserved_invisible_time();
  reserved_invisible_time_ = 0.3;
  //done-tofoConf use_frame_timestamp_ = config.use_frame_timestamp();
  use_frame_timestamp_ = true;

  foreground_objects_.clear();
  background_objects_.clear();
  foreground_track_data_.clear();
  background_track_data_.clear();
  if (main_sensor_.size() == 0) {
    main_sensor_.emplace("velodyne64");  // default value
  }

  matcher_.reset(new MlfTrackObjectMatcher);
  MlfTrackObjectMatcherInitOptions matcher_init_options;
  check_mod(matcher_->Init(matcher_init_options));

  tracker_.reset(new MlfTracker);
  MlfTrackerInitOptions tracker_init_options;
  check_mod(tracker_->Init(tracker_init_options));
  return true;
}

bool MlfEngine::Track(
                      LidarFrame* frame) {
  // 0. modify objects timestamp if necessary
  //AERROR << "MlfEngine SegmentationObjects: "<<frame->segmented_objects.size();
  if (use_frame_timestamp_) {
    for (auto& object : frame->segmented_objects) {
      object->latest_tracked_time = frame->timestamp;
    }
  }
  std::cout<<"start track,  number of objects: " << frame->segmented_objects.size()<<std::endl;
  // 1. add global offset to pose (only when no track exists)
  if (foreground_track_data_.size() == 0 &&
      background_track_data_.size() == 0) {
    global_to_local_offset_ = -frame->lidar2world_pose.translation();
  }
  sensor_to_local_pose_ = frame->lidar2world_pose;
  sensor_to_local_pose_.pretranslate(global_to_local_offset_);
  // 2. split fg and bg objects, and transform to tracked objects
  SplitAndTransformToTrackedObjects(frame->segmented_objects,
                                    frame->sensor_info);
  // 3. assign tracked objects to tracks
  MlfTrackObjectMatcherOptions match_options;
  TrackObjectMatchAndAssign(match_options, foreground_objects_, "foreground",
                            &foreground_track_data_);
  TrackObjectMatchAndAssign(match_options, background_objects_, "background",
                            &background_track_data_);
  // 4. state filter in tracker if is main sensor
//  AERROR<<"frame sensor name  "<<frame->sensor_info.name;
  bool is_main_sensor =
      (main_sensor_.find(frame->sensor_info.name) != main_sensor_.end());
  if (is_main_sensor) {
    TrackStateFilter(foreground_track_data_, frame->timestamp);
    TrackStateFilter(background_track_data_, frame->timestamp);
  }
  // 5. track to object if is main sensor
  frame->tracked_objects.clear();
  if (is_main_sensor) {
    //AERROR<<"is main sensor";
    CollectTrackedResult(frame);
  }
  // 6. remove stale data
  RemoveStaleTrackData("foreground", frame->timestamp, &foreground_track_data_);
  RemoveStaleTrackData("background", frame->timestamp, &background_track_data_);
  std::cout << "MlfEngine publish objects: " << frame->tracked_objects.size()
        << " sensor_name: " << frame->sensor_info.name
        << " at timestamp: " << std::to_string(frame->timestamp);
  return true;
}

void MlfEngine::SplitAndTransformToTrackedObjects(
    const std::vector<base::ObjectPtr>& objects,
    const base::SensorInfo& sensor_info) {
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(objects.size(), &tracked_objects);
  foreground_objects_.clear();
  background_objects_.clear();
  for (size_t i = 0; i < objects.size(); ++i) {
    tracked_objects[i]->AttachObject(objects[i], sensor_to_local_pose_,
                                     global_to_local_offset_, sensor_info);
    if (!objects[i]->lidar_supplement.is_background &&
        use_histogram_for_match_) {
      tracked_objects[i]->histogram_bin_size = histogram_bin_size_;
      tracked_objects[i]->ComputeShapeFeatures();
    }
    if (objects[i]->lidar_supplement.is_background) {
      background_objects_.push_back(tracked_objects[i]);
    } else {
      foreground_objects_.push_back(tracked_objects[i]);
    }
  }
  std::cout << "MlfEngine: " << sensor_info.name
        << " foreground: " << foreground_objects_.size()
        << " background: " << background_objects_.size();
}

void MlfEngine::TrackObjectMatchAndAssign(
    const MlfTrackObjectMatcherOptions& match_options,
    const std::vector<TrackedObjectPtr>& objects, const std::string& name,
    std::vector<MlfTrackDataPtr>* tracks) {
  std::vector<std::pair<size_t, size_t> > assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  matcher_->Match(match_options, objects, *tracks, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  std::cout << "MlfEngine: " + name + " assignments " << assignments.size()
        << " unassigned_tracks " << unassigned_tracks.size()
        << " unassigned_objects " << unassigned_objects.size();
  // 1. for assignment, push object to cache of track_data
  for (auto& pair : assignments) {
    const size_t& track_id = pair.first;
    const size_t& object_id = pair.second;
    tracks->at(track_id)->PushTrackedObjectToCache(objects[object_id]);
  }
  // 2. for unassigned_objects, create new tracks
  for (auto& id : unassigned_objects) {
    MlfTrackDataPtr track_data = MlfTrackDataPool::Instance().Get();
    tracker_->InitializeTrack(track_data, objects[id]);
    tracks->push_back(track_data);
  }
  // 3. for unassigned_tracks, do nothing
}

void MlfEngine::TrackStateFilter(const std::vector<MlfTrackDataPtr>& tracks,
                                 double frame_timestamp) {
  std::vector<TrackedObjectPtr> objects;
  for (auto& track_data : tracks) {
    track_data->GetAndCleanCachedObjectsInTimeInterval(&objects);
    for (auto& obj : objects) {
      tracker_->UpdateTrackDataWithObject(track_data, obj);
    }
    if (objects.size() == 0) {
      tracker_->UpdateTrackDataWithoutObject(frame_timestamp, track_data);
    }
  }
}

void MlfEngine::CollectTrackedResult(LidarFrame* frame) {
  auto& tracked_objects = frame->tracked_objects;
  tracked_objects.clear();
  size_t num_objects =
      foreground_track_data_.size() + background_track_data_.size();
  apollo::perception::base::ConcurrentObjectPool<apollo::perception::base::Object>::Instance().BatchGet(num_objects, &tracked_objects);
  size_t pos = 0;
  size_t num_predict = 0;
  auto collect = [&](std::vector<MlfTrackDataPtr> * tracks) {
    for (auto& track_data : *tracks) {
      if (!output_predict_objects_ && track_data->is_current_state_predicted_) {
        ++num_predict;
      } else {
        check_mod(track_data->ToObject(-global_to_local_offset_, frame->timestamp,
                                   tracked_objects[pos]));
        ++pos;
      }
    }
  };
  collect(&foreground_track_data_);
  collect(&background_track_data_);
  if (num_predict != 0) {
    std::cout << "MlfEngine, num_predict: " << num_predict
          << " num_objects: " << num_objects;
    check_mod(num_predict <= num_objects);
    tracked_objects.resize(num_objects - num_predict);
  }
}

void MlfEngine::RemoveStaleTrackData(const std::string& name, double timestamp,
                                     std::vector<MlfTrackDataPtr>* tracks) {
  size_t pos = 0;
  for (size_t i = 0; i < tracks->size(); ++i) {
    if (tracks->at(i)->latest_visible_time_ + reserved_invisible_time_ >=
        timestamp) {
      if (i != pos) {
        tracks->at(pos) = tracks->at(i);
      }
      ++pos;
    }
  }
  std::cout << "MlfEngine: " << name << " remove stale tracks, from "
        << tracks->size() << " to " << pos;
  tracks->resize(pos);
}

//PERCEPTION_REGISTER_MULTITARGET_TRACKER(MlfEngine);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
