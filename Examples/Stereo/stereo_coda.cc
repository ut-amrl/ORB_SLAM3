/**
 * Author: Dongmyeong Lee (domlee[at]utexas.edu)
 * Date:   January 30, 2024
 * Description: This file is for running ORB-SLAM3 with the UT Campus Object Dataset
 * (CODa) in the stereo mode.
 */

#include <System.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathLeft,
                const string &strPathRight,
                const string &strPathTimes,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<double> &vTimeStamps);

bool IsNumeric(const string &str) {
  return std::all_of(str.begin(), str.end(), ::isdigit);
}

int main(int argc, char **argv) {
  if (argc < 6) {
    cerr << endl
         << "Usage: ./mono_coda path_to_vocabulary path_to_settings "
            "path_to_images_folder path_to_times_folder "
            "sequence1 [sequence2 ... sequenceN] (trajectory_file_name)"
         << endl;
    return 1;
  }

  string imageBasePath = string(argv[3]);
  string timeBasePath = string(argv[4]);

  vector<string> sequences;
  bool bFileName = false;
  string outputFileName;
  for (int i = 5; i < argc; i++) {
    string arg = argv[i];
    if (i == argc - 1 && !IsNumeric(arg)) {
      bFileName = true;
      outputFileName = arg;
      cout << "file name: " << outputFileName << endl;
    } else {
      sequences.push_back(arg);
    }
  }
  int num_seq = sequences.size();
  cout << "num_seq = " << sequences.size() << endl;

  // Load all sequences:
  vector<vector<string>> vstrImageLeft;
  vector<vector<string>> vstrImageRight;
  vector<vector<double>> vTimestampsCam;
  vector<int> nImages;

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  for (int i = 0; i < num_seq; i++) {
    string seq = sequences[i];
    cout << "Loading images for sequence " << seq << "...";

    string pathCam0 = imageBasePath + "/cam0/" + seq;
    string pathCam1 = imageBasePath + "/cam1/" + seq;
    string pathTimeStamps = timeBasePath + "/" + seq + ".txt";
    LoadImages(pathCam0,
               pathCam1,
               pathTimeStamps,
               vstrImageLeft[i],
               vstrImageRight[i],
               vTimestampsCam[i]);
    cout << "LOADED!" << endl;

    nImages[i] = vstrImageLeft[i].size();
    tot_images += nImages[i];
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to process
  // frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

  double t_resize = 0.f;
  double t_track = 0.f;

  cv::Mat imLeft, imRight;
  for (int i = 0; i < num_seq; i++) {
    // Main loop
    int proccIm = 0;
    for (int ni = 0; ni < nImages[i]; ni++, proccIm++) {
      // Read image from file
      imLeft = cv::imread(vstrImageLeft[i][ni],
                          cv::IMREAD_UNCHANGED);  // CV_LOAD_IMAGE_UNCHANGED);
      imRight = cv::imread(vstrImageRight[i][ni],
                           cv::IMREAD_UNCHANGED);  // CV_LOAD_IMAGE_UNCHANGED);
      double tframe = vTimestampsCam[i][ni];

      if (imLeft.empty()) {
        cerr << endl << "Failed to load image at: " << vstrImageLeft[i][ni] << endl;
        return 1;
      }
      if (imRight.empty()) {
        cerr << endl << "Failed to load image at: " << vstrImageRight[i][ni] << endl;
        return 1;
      }

      // Pass the image to the SLAM system
      auto t1 = std::chrono::steady_clock::now();
      SLAM.TrackStereo(imLeft,
                       imRight,
                       tframe,
                       vector<ORB_SLAM3::IMU::Point>(),
                       vstrImageLeft[i][ni]);
      auto t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track =
          t_resize +
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1)
              .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[i] - 1)
        T = vTimestampsCam[i][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[i][ni - 1];

      if (ttrack < T) {
        usleep((T - ttrack) * 1e6);  // 1e6
      }
    }

    if (i < num_seq - 1) {
      string kf_file_submap = "./SubMaps/CODa/kf_SubMap_" + sequences[i] + ".txt";
      string f_file_submap = "./SubMaps/CODa/f_SubMap_" + sequences[i] + ".txt";
      SLAM.SaveTrajectoryCODa(f_file_submap);
      //SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    const string lost_file = "lost_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryCODa(f_file);
    SLAM.SaveLostFrames(lost_file);
    // SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    SLAM.SaveTrajectoryCODa("CameraTrajectory.txt");
    SLAM.SaveLostFrames("LostFrames.txt");
  }

  return 0;
}

string GetFilePrefix(const string &strImagePath) {
  stringstream ss(strImagePath);
  string item;
  vector<string> elems;
  while (getline(ss, item, '/')) {
    if (item.empty()) continue;
    elems.push_back(item);
  }

  // Example: /Dataset/CODa/2d_rect/cam0/0 => 2d_rect_cam0_0_
  if (elems.size() >= 3) {
    return elems[elems.size() - 3] + "_" + elems[elems.size() - 2] + "_" +
           elems[elems.size() - 1] + "_";
  }

  return "";
}

void LoadImages(const string &strPathLeft,
                const string &strPathRight,
                const string &strPathTimes,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<double> &vTimeStamps) {
  string prefixLeft = GetFilePrefix(strPathLeft);
  string prefixRight = GetFilePrefix(strPathRight);
  string fileExtension = ".png";
  if (prefixLeft.find("2d_rect") != string::npos) {
    fileExtension = ".jpg";
  }

  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(22000);
  vstrImageLeft.reserve(22000);
  vstrImageRight.reserve(22000);
  int frame = 0;
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      vstrImageLeft.push_back(strPathLeft + "/" + prefixLeft + to_string(frame) +
                              fileExtension);
      vstrImageRight.push_back(strPathRight + "/" + prefixRight + to_string(frame) +
                               fileExtension);
      vTimeStamps.push_back(stod(s));
      frame++;
    }
  }
}
