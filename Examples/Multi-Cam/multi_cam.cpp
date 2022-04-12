//
// Created by vuong on 4/8/22.
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

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

bool loadImages(string &strPathFolder, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<string> &vstrImageSideLeft, vector<double> &vTimeStamps);

double ttrack_tot = 0;
int main(int argc, char **argv) {
  const int num_seq = (argc - 4);
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = true;

  if (argc < 5) {
    cerr << endl
         << "Usage: ./multi_cam path_to_vocabulary path_to_settings "
            "path_to_image_folder1 (path_to_image_folder2 ... "
            "path_to_image_folder_N) trajectory_file_name"
         << endl;
    return EXIT_FAILURE;
  }

  // Load all sequences:
  int seq;
  vector<vector<string> > vstrImageLeftFilenames;
  vector<vector<string> > vstrImageRightFilenames;
  vector<vector<string> > vstrImageSideLeftFilenames;
  vector<vector<double> > vTimestampsCam;
  vector<size_t> nImages;

  vstrImageLeftFilenames.resize(num_seq);
  vstrImageRightFilenames.resize(num_seq);
  vstrImageSideLeftFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  size_t tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "... ";
    string strPathFolder = string(argv[(2 * seq) + 3]);
    bool isLoaded =
        loadImages(strPathFolder, vstrImageLeftFilenames[seq],
                   vstrImageRightFilenames[seq],
                   vstrImageSideLeftFilenames[seq], vTimestampsCam[seq]);
    if(isLoaded)
      cout << "LOADED!" << endl;
    else
      return EXIT_FAILURE;

    nImages[seq] = vstrImageLeftFilenames[seq].size();
    tot_images += nImages[seq];

    if (vstrImageLeftFilenames.size() != vstrImageRightFilenames.size() ||
        vstrImageLeftFilenames.size() != vstrImageSideLeftFilenames.size()) {
      cerr << "ERROR: Images is not sync, number of image is not equal" << seq
           << endl;
      return EXIT_FAILURE;
    }

    if ((nImages[seq] <= 0)) {
      cerr << "ERROR: Failed to load images for sequence" << seq << endl;
      return EXIT_FAILURE;
    }
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
  float imageScale = SLAM.GetImageScale();

  cout << endl << "-------" << endl;
  cout.precision(17);

  cv::Mat imLeft;
  cv::Mat imRight;
  cv::Mat imSideLeft;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  double t_resize = 0.F;
  double t_track = 0.F;

  int proccIm = 0;
  for (seq = 0; seq < num_seq; seq++) {
    // Main loop
    proccIm = 0;
    for (size_t ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read image from file
      imLeft =
          cv::imread(vstrImageLeftFilenames[seq][ni], cv::IMREAD_GRAYSCALE);
      imRight =
          cv::imread(vstrImageRightFilenames[seq][ni], cv::IMREAD_GRAYSCALE);
      imSideLeft =
          cv::imread(vstrImageSideLeftFilenames[seq][ni], cv::IMREAD_GRAYSCALE);
      if (imageScale != 1.F) {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Resize =
            std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_Start_Resize =
            std::chrono::monotonic_clock::now();
#endif
#endif
        int width = int(float(imLeft.cols) * imageScale);
        int height = int(float(imLeft.rows) * imageScale);
        cv::resize(imLeft, imLeft, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));
        cv::resize(imSideLeft, imSideLeft, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Resize =
            std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_End_Resize =
            std::chrono::monotonic_clock::now();
#endif
        t_resize = std::chrono::duration_cast<
                       std::chrono::duration<double, std::milli> >(
                       t_End_Resize - t_Start_Resize)
                       .count();
        SLAM.InsertResizeTime(t_resize);
#endif
      }

      // clahe
      clahe->apply(imLeft, imLeft);
      clahe->apply(imRight, imRight);
      clahe->apply(imSideLeft, imSideLeft);
      double tframe = vTimestampsCam[seq][ni];

      if (imLeft.empty() || imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageLeftFilenames[seq][ni]
             << endl;
        return 1;
      }

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      //      SLAM.TrackStereo(imLeft, imRight, tframe);
      SLAM.TrackMulti_Cam(imLeft, imRight, imSideLeft, tframe);
#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
      t_track =
          t_resize + std::chrono::duration_cast<
                         std::chrono::duration<double, std::milli> >(t2 - t1)
                         .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
              .count();
      ttrack_tot += ttrack;
      // std::cout << "ttrack: " << ttrack << std::endl;

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

  // Tracking time statistics

  // Save camera trajectory
  std::chrono::system_clock::time_point scNow =
      std::chrono::system_clock::now();
  std::time_t now = std::chrono::system_clock::to_time_t(scNow);
  std::stringstream ss;
  ss << now;

  if (bFileName) {
    const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (size_t ni = 0; ni < nImages[0]; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages[0] / 2] << endl;
  cout << "mean tracking time: " << totaltime / proccIm << endl;

  return 0;
}

bool loadImages(string &strPathFolder, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<string> &vstrImageSideLeft,
                vector<double> &vTimeStamps) {
  if (strPathFolder.at(strPathFolder.length() - 1) != '/') {
    strPathFolder += '/';
  }

  string strPathTimes = strPathFolder + "zed2_left.txt";

  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  vstrImageSideLeft.reserve(5000);

  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  if (!fTimes.good()) {
    std::cerr << "wrong path left time: " << strPathTimes.c_str() << std::endl;
    return false;
  }
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);

    if (!s.empty()) {
      if (s[0] == '#') continue;

      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimeStamps.push_back(t);
      string strLeft;
      ss >> strLeft;
      vstrImageLeft.push_back(strPathFolder + strLeft);
    }
  }
  fTimes.close();

  // Right
  strPathTimes = strPathFolder + "zed2_right.txt";
  fTimes.open(strPathTimes);
  if (!fTimes.good()) {
    std::cerr << "wrong path right time: " << strPathTimes.c_str() << std::endl;
    return false;
  }
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);

    if (!s.empty()) {
      if (s[0] == '#') continue;

      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      string strRight;
      ss >> strRight;
      vstrImageRight.push_back(strPathFolder + strRight);
    }
  }
  fTimes.close();

  // Mono
  strPathTimes = strPathFolder + "side_left.txt";
  fTimes.open(strPathTimes);
  if (!fTimes.good()) {
    std::cerr << "wrong path side left time: " << strPathTimes.c_str()
              << std::endl;
    return false;
  }
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);

    if (!s.empty()) {
      if (s[0] == '#') continue;

      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      string strMono;
      ss >> strMono;
      vstrImageSideLeft.push_back(strPathFolder + strMono);
    }
  }
  fTimes.close();
  return true;
}
