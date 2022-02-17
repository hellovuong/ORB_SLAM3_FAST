//
// Created by vuong on 24/12/2021.
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
#include <iostream>
#include <opencv2/core/core.hpp>

#include "ImuTypes.h"

using namespace std;

void LoadImages(const string& strPathFolder, vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight, vector<double>& vTimeStamps);

int main(int argc, char** argv) {
  if (argc < 5) {
    cerr << endl
         << "Usage: ./stereo_zed2 path_to_vocabulary "
            "path_to_settings path_to_sequence_folder_1 path_to_times_file_dir "
            "(path_to_image_folder_2 path_to_times_file_2 ... "
            "path_to_image_folder_N path_to_times_file_N) "
         << endl;
    return 1;
  }

  const int num_seq = (argc - 3) / 2;
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = (((argc - 3) % 2) == 1);
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
  vector<int> nImages;

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathSeq(argv[(2 * seq) + 3]);
    string pathTimeStamps(argv[(2 * seq) + 4]);

    LoadImages(pathSeq, vstrImageLeft[seq], vstrImageRight[seq],
               vTimestampsCam[seq]);
    cout << "LOADED!" << endl;

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images += nImages[seq];

    if ((nImages[seq] <= 0) || (nImages[seq] != vstrImageRight[seq].size())) {
      cerr << "ERROR: Failed to load images or difference between left and "
              "right camera"
           << seq << endl;
      return EXIT_FAILURE;
    }
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
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

  cv::Mat imLeft, imRight;
  for (seq = 0; seq < num_seq; seq++) {
    // Seq loop
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

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif
      // Pass the images to the SLAM system
      SLAM.TrackStereo(imLeft, imRight, tframe);

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

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6
    }

    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  return 0;
}

void LoadImages(const string& strPathFolder, vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight, vector<double>& vTimeStamps) {
  ifstream fTimes;
  string strPathTimesLeft = strPathFolder + "/zed2_times_left.txt";
  string strPathTimesRight = strPathFolder + "/zed2_times_right.txt";
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
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strRight;
      ss >> strRight;
      vstrImageRight.push_back(strPathFolder + "/" + strRight);
    }
  }
}
