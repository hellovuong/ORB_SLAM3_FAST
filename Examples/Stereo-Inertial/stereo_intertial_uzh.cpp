//
// Created by vuong on 27/01/2022.
//
//
// Created by vuong on 24/12/2021.
//
/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <System.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "ImuTypes.h"

using namespace std;

void LoadImages(const string& strPathFolder,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimeStamps);

void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro);

int main(int argc, char** argv) {
  if (argc < 4) {
    cerr << endl
         << "Usage: ./stereo_inertial_zed2 path_to_vocabulary "
            "path_to_settings path_to_sequence_folder_1 "
            "(path_to_image_folder_2 ... "
            "path_to_image_folder_N) "
         << endl;
    return EXIT_FAILURE;
  }

  const int num_seq = (argc - 3) / 2;
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = false;
  string file_name;
  if (bFileName) {
    file_name = string(argv[argc - 1]);
    cout << "file name: " << file_name << endl;
  }

  // Load all sequences:
  int seq;
  vector<vector<string> > vstrImageLeft;
  vector<vector<string> > vstrImageRight;
  vector<vector<double> > vTimestampsCam;
  vector<vector<cv::Point3f> > vAcc, vGyro;
  vector<vector<double> > vTimestampsImu;
  vector<int> nImages;
  vector<int> nImu;
  vector<int> first_imu(num_seq, 0);

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathSeq(argv[(2 * seq) + 3]);

    string pathImu = pathSeq + "/imu.txt";

    LoadImages(
        pathSeq, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence " << seq << "...";
    LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    cout << "LOADED!" << endl;

    if (vstrImageLeft[seq].size() != vstrImageRight[seq].size()) {
      cerr << "ERROR: Failed to load images" << seq << endl;
      return EXIT_FAILURE;
    }

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images += nImages[seq];
    nImu[seq] = vTimestampsImu[seq].size();

    if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
      cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
      return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first

    while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
      first_imu[seq]++;
    first_imu[seq]--;  // first imu measurement to be considered
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);

  cv::Mat imLeft, imRight;
  for (seq = 0; seq < num_seq; seq++) {
    double t_rect = 0.f;
    double t_resize = 0.f;
    double t_track = 0.f;
    // Seq loop
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    int proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
             << endl;
        return 1;
      }

      if (imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageRight[seq][ni])
             << endl;
        return 1;
      }

      double tframe = vTimestampsCam[seq][ni];

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0)
        while (
            vTimestampsImu[seq][first_imu[seq]] <=
            vTimestampsCam
                [seq]
                [ni])  // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
        {
          vImuMeas.push_back(
              ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,
                                    vAcc[seq][first_imu[seq]].y,
                                    vAcc[seq][first_imu[seq]].z,
                                    vGyro[seq][first_imu[seq]].x,
                                    vGyro[seq][first_imu[seq]].y,
                                    vGyro[seq][first_imu[seq]].z,
                                    vTimestampsImu[seq][first_imu[seq]]));
          first_imu[seq]++;
        }

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif
      // Pass the images to the SLAM system
      SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
      t_track = t_rect + t_resize +
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli> >(t2 - t1)
                    .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
              .count();

      // vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      // double T = 0;
      // if (ni < nImages[seq] - 1)
      //   T = vTimestampsCam[seq][ni + 1] - tframe;
      // else if (ni > 0)
      //   T = tframe - vTimestampsCam[seq][ni - 1];

      // if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6
    }
    std::cout << "Finish seq " << std::endl;
    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  std::cout << "Trying stop thread " << std::endl;
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (true) {
    std::cout << "Saved trajectory " << std::endl;
    const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryUZH(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    std::cout << "Saved trajectory " << std::endl;
    SLAM.SaveTrajectoryUZH("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  return EXIT_SUCCESS;
}

void LoadImages(const string& strPathFolder,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimeStamps) {
  ifstream fTimes;
  string strPathTimesLeft = strPathFolder + "/left_images.txt";
  string strPathTimesRight = strPathFolder + "/right_images.txt";
  fTimes.open(strPathTimesLeft.c_str());
  fTimes.good() ? std::cout << "Left timestamp path exist\n"
                : std::cerr << "Left timestamp path doesn't exist\n";
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      int id;
      ss >> id;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strLeft;
      ss >> strLeft;
      vstrImageLeft.push_back(strPathFolder + "/" + strLeft);
    }
  }
  fTimes.close();
  fTimes.open(strPathTimesRight.c_str());
  fTimes.good() ? std::cout << "Right timestamp path exist\n"
                : std::cerr << "Right timestamp path doesn't exist\n";
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      int id;
      ss >> id;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strRight;
      ss >> strRight;
      vstrImageRight.push_back(strPathFolder + "/" + strRight);
    }
  }
}

void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro) {
  ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);
  fImu.good() ? std::cout << "IMU path exist\n"
              : std::cerr << "IMU path doesn't exist\n";
  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#') continue;

    if (!s.empty()) {
      double data[7];
      stringstream ss;
      ss << s;
      int id;
      ss >> id;
      ss >> data[0]; // time
      ss >> data[1]; // gyro_x
      ss >> data[2]; // gyro_y
      ss >> data[3]; // gyro_z
      ss >> data[4]; // acel_x
      ss >> data[5]; // acel_y
      ss >> data[6]; // acel_z
      vTimeStamps.push_back(data[0]);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}
