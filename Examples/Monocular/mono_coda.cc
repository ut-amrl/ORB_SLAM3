/**
 * Author: Dongmyeong Lee (domlee[at]utexas.edu)
 * Date:   January 28, 2024
 * Description: This file is for running ORB-SLAM3 with the UT Campus Object Dataset
 * (CODa) in the monocular mode.
 */

#include <System.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strImagePath,
                const string &strPathTimes,
                vector<string> &vstrImages,
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
  vector<vector<string>> vstrImageFilenames;
  vector<vector<double>> vTimestampsCam;
  vector<int> nImages;

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  for (int i = 0; i < num_seq; i++) {
    string seq = sequences[i];
    cout << "Loading images for sequence " << seq << "...";
    LoadImages(imageBasePath + "/cam0/" + seq,
               timeBasePath + "/" + seq + ".txt",
               vstrImageFilenames[i],
               vTimestampsCam[i]);
    cout << "LOADED!" << endl;

    nImages[i] = vstrImageFilenames[i].size();
    tot_images += nImages[i];
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  int fps = 20;
  float dT = 1.f / fps;
  // Create SLAM system. It initializes all system threads and gets ready to process
  // frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
  float imageScale = SLAM.GetImageScale();

  double t_resize = 0.f;
  double t_track = 0.f;

  for (int i = 0; i < num_seq; i++) {
    // Main loop
    cv::Mat im;
    int proccIm = 0;
    for (int ni = 0; ni < nImages[i]; ni++, proccIm++) {
      // Read image from file
      im = cv::imread(vstrImageFilenames[i][ni],
                      cv::IMREAD_UNCHANGED);  // CV_LOAD_IMAGE_UNCHANGED);
      double tframe = vTimestampsCam[i][ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[i][ni] << endl;
        return 1;
      }

      if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
        auto t_Start_Resize = std::chrono::steady_clock::now();
#endif
        int width = im.cols * imageScale;
        int height = im.rows * imageScale;
        cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
        auto t_End_Resize = std::chrono::steady_clock::now();
        t_resize =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                t_End_Resize - t_Start_Resize)
                .count();
        SLAM.InsertResizeTime(t_resize);
#endif
      }

      // Pass the image to the SLAM system
      auto t1 = std::chrono::steady_clock::now();
      SLAM.TrackMonocular(im, tframe);
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
      SLAM.SaveTrajectoryEuRoC(f_file_submap);
      SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

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

void LoadImages(const string &strImagePath,
                const string &strPathTimes,
                vector<string> &vstrImages,
                vector<double> &vTimeStamps) {
  string prefix = GetFilePrefix(strImagePath);
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(22000);
  vstrImages.reserve(22000);
  int frame = 0;
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      string fileExtension = ".png";
      if (prefix.find("2d_rect") != string::npos) {
        fileExtension = ".jpg";
      }

      vstrImages.push_back(strImagePath + "/" + prefix + to_string(frame++) +
                           fileExtension);
      double t = stod(s);
      vTimeStamps.push_back(t);
    }
  }
}
