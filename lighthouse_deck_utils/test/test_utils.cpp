// Copyright 2026 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "lighthouse_deck_utils/utils.hpp"

namespace lighthouse_deck_utils
{

class LoadStationsMapTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a temporary directory for test files
    test_dir_ = "/tmp/lighthouse_deck_utils_test";
    std::system(("mkdir -p " + test_dir_).c_str());
  }

  void TearDown() override
  {
    // Clean up test files
    std::system(("rm -rf " + test_dir_).c_str());
  }

  std::string test_dir_;
};

TEST_F(LoadStationsMapTest, LoadValidFile)
{
  // Create a valid test file
  std::string filepath = test_dir_ + "/valid_map.csv";
  std::ofstream file(filepath);
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << "1,1.0,2.0,3.0,0.0,0.0,0.0,1.0\n";
  file << "2,-1.5,2.5,0.5,0.1,0.2,0.3,0.9273618495\n";
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_TRUE(result.has_value());

  auto & [poses, ids] = *result;
  ASSERT_EQ(poses.size(), 2);
  ASSERT_EQ(ids.size(), 2);

  // Check first station
  EXPECT_EQ(ids[0], 1);
  auto t1 = poses[0].translation();
  EXPECT_DOUBLE_EQ(t1.x(), 1.0);
  EXPECT_DOUBLE_EQ(t1.y(), 2.0);
  EXPECT_DOUBLE_EQ(t1.z(), 3.0);
  auto q1 = poses[0].unit_quaternion();
  EXPECT_DOUBLE_EQ(q1.x(), 0.0);
  EXPECT_DOUBLE_EQ(q1.y(), 0.0);
  EXPECT_DOUBLE_EQ(q1.z(), 0.0);
  EXPECT_DOUBLE_EQ(q1.w(), 1.0);

  // Check second station
  EXPECT_EQ(ids[1], 2);
  auto t2 = poses[1].translation();
  EXPECT_DOUBLE_EQ(t2.x(), -1.5);
  EXPECT_DOUBLE_EQ(t2.y(), 2.5);
  EXPECT_DOUBLE_EQ(t2.z(), 0.5);
}

TEST_F(LoadStationsMapTest, FileNotFound)
{
  auto result = load_stations_map("/nonexistent/file.csv");
  ASSERT_FALSE(result.has_value());
}

TEST_F(LoadStationsMapTest, EmptyFile)
{
  std::string filepath = test_dir_ + "/empty.csv";
  std::ofstream file(filepath);
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_FALSE(result.has_value());
}

TEST_F(LoadStationsMapTest, InvalidHeader)
{
  std::string filepath = test_dir_ + "/invalid_header.csv";
  std::ofstream file(filepath);
  file << "wrong,header,format\n";
  file << "1,1.0,2.0,3.0,0.0,0.0,0.0,1.0\n";
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_FALSE(result.has_value());
}

TEST_F(LoadStationsMapTest, SkipInvalidLines)
{
  std::string filepath = test_dir_ + "/some_invalid_lines.csv";
  std::ofstream file(filepath);
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << "1,1.0,2.0,3.0,0.0,0.0,0.0,1.0\n";
  file << "invalid,line,with,wrong,format\n";  // Invalid line - should be skipped
  file << "2,-1.5,2.5,0.5,0.0,0.0,0.0,1.0\n";
  file << "\n";  // Empty line - should be skipped
  file << "3,0.0,0.0,1.0,0.0,0.0,0.0,1.0\n";
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_TRUE(result.has_value());

  auto & [poses, ids] = *result;
  ASSERT_EQ(poses.size(), 3);
  ASSERT_EQ(ids.size(), 3);
  EXPECT_EQ(ids[0], 1);
  EXPECT_EQ(ids[1], 2);
  EXPECT_EQ(ids[2], 3);
}

TEST_F(LoadStationsMapTest, NoValidData)
{
  std::string filepath = test_dir_ + "/no_valid_data.csv";
  std::ofstream file(filepath);
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << "invalid,line,only\n";
  file << "\n";
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_FALSE(result.has_value());
}

TEST_F(LoadStationsMapTest, ClearsExistingData)
{
  // Create a valid test file with one station
  std::string filepath = test_dir_ + "/clear_test.csv";
  std::ofstream file(filepath);
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << "5,5.0,5.0,5.0,0.0,0.0,0.0,1.0\n";
  file.close();

  auto result = load_stations_map(filepath);
  ASSERT_TRUE(result.has_value());

  auto & [poses, ids] = *result;
  // Should only have new data
  ASSERT_EQ(poses.size(), 1);
  ASSERT_EQ(ids.size(), 1);
  EXPECT_EQ(ids[0], 5);
}

TEST_F(LoadStationsMapTest, SaveAndLoadRoundTrip)
{
  // Create test data
  std::vector<Sophus::SE3d> original_poses;
  std::vector<lighthouse_geometry_utils::StationId> original_ids;

  // Station 1: Identity pose
  original_ids.push_back(1);
  original_poses.push_back(Sophus::SE3d());

  // Station 2: Translation only
  Eigen::Vector3d translation2(1.5, -2.3, 0.8);
  Eigen::Quaterniond quaternion2(1.0, 0.0, 0.0, 0.0);
  original_ids.push_back(2);
  original_poses.push_back(Sophus::SE3d(quaternion2, translation2));

  // Station 3: Rotation and translation
  Eigen::Vector3d translation3(-0.5, 1.2, 3.4);
  Eigen::Quaterniond quaternion3(0.7071068, 0.7071068, 0.0, 0.0);  // 90 deg around X
  quaternion3.normalize();
  original_ids.push_back(3);
  original_poses.push_back(Sophus::SE3d(quaternion3, translation3));

  // Save to file
  std::string filepath = test_dir_ + "/roundtrip_test.csv";
  ASSERT_TRUE(save_stations_map(filepath, original_poses, original_ids));

  // Load from file
  auto result = load_stations_map(filepath);
  ASSERT_TRUE(result.has_value());

  auto & [loaded_poses, loaded_ids] = *result;

  // Verify data matches
  ASSERT_EQ(loaded_poses.size(), original_poses.size());
  ASSERT_EQ(loaded_ids.size(), original_ids.size());

  for (std::size_t i = 0; i < original_ids.size(); ++i) {
    // Check station IDs
    EXPECT_EQ(loaded_ids[i], original_ids[i]);

    // Check translations (should be exact with fixed precision)
    auto orig_t = original_poses[i].translation();
    auto load_t = loaded_poses[i].translation();
    EXPECT_NEAR(load_t.x(), orig_t.x(), 1e-8);
    EXPECT_NEAR(load_t.y(), orig_t.y(), 1e-8);
    EXPECT_NEAR(load_t.z(), orig_t.z(), 1e-8);

    // Check quaternions (should be exact with fixed precision)
    auto orig_q = original_poses[i].unit_quaternion();
    auto load_q = loaded_poses[i].unit_quaternion();
    EXPECT_NEAR(load_q.x(), orig_q.x(), 1e-8);
    EXPECT_NEAR(load_q.y(), orig_q.y(), 1e-8);
    EXPECT_NEAR(load_q.z(), orig_q.z(), 1e-8);
    EXPECT_NEAR(load_q.w(), orig_q.w(), 1e-8);
  }
}

}  // namespace lighthouse_deck_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
