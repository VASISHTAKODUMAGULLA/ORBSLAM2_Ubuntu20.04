/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

 #include <algorithm>
 #include <chrono>
 #include <fstream>
 #include <iomanip>
 #include <iostream>
 #include <numeric>
 #include <sstream>
 #include <sysexits.h>
 #include <thread>
 #include <boost/filesystem.hpp>
 
 #include <opencv2/core/core.hpp>
 #include <System.h>
 
 namespace fs = ::boost::filesystem;
 using namespace std;
 
 void LoadImages(const string &strPathLeft, const string &strPathRight,
                 const string &strPathTimes, vector<string> &vstrImageLeft,
                 vector<string> &vstrImageRight, vector<double> &vTimeStamps);
 string FindFile(const string& baseFileName, const string& pathHint);
 
 int main(int argc, char **argv) {
     if (argc != 5) {
         cerr << endl
              << "Usage: ./stereo_euroc settings_file path_to_left_folder "
                 "path_to_right_folder path_to_times_file"
              << endl;
         return EX_USAGE;
     }
 
     // Retrieve paths to images and timestamps
     vector<string> vstrImageLeft, vstrImageRight;
     vector<double> vTimeStamp;
     string timeStampsFile = string(DEFAULT_STEREO_SETTINGS_DIR)
                             + string("EuRoC_TimeStamps/")
                             + string(argv[4]);
 
     LoadImages(string(argv[2]), string(argv[3]), timeStampsFile,
                vstrImageLeft, vstrImageRight, vTimeStamp);
 
     if (vstrImageLeft.empty() || vstrImageRight.empty()) {
         cerr << "ERROR: No images in provided path." << endl;
         return EX_NOINPUT;
     }
     if (vstrImageLeft.size() != vstrImageRight.size()) {
         cerr << "ERROR: Different number of left and right images." << endl;
         return EX_DATAERR;
     }
 
     // Read camera intrinsics only
     string settingsFile = FindFile(string(argv[1]),
                                    string(DEFAULT_STEREO_SETTINGS_DIR));
     cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
     if (!fsSettings.isOpened()) {
         cerr << "ERROR: Wrong path to settings" << endl;
         return EX_DATAERR;
     }
 
     cv::Mat K_l, K_r;
     fsSettings["LEFT.K"]  >> K_l;
     fsSettings["RIGHT.K"] >> K_r;
 
     int rows_l = fsSettings["LEFT.height"];
     int cols_l = fsSettings["LEFT.width"];
     int rows_r = fsSettings["RIGHT.height"];
     int cols_r = fsSettings["RIGHT.width"];
 
     cout << "Config: " << rows_l << "," << cols_l
          << "," << rows_r << "," << cols_r << endl;
 
     // Note: Skipping stereo rectification and distortion. Images are pre-rectified.
 
     const int nImages = vstrImageLeft.size();
 
     // Initialize SLAM system in stereo mode
     ORB_SLAM2::System SLAM(DEFAULT_BINARY_ORB_VOCABULARY, settingsFile,
                            ORB_SLAM2::System::STEREO, true);
 
     vector<float> vTimesTrack(nImages);
     cout << endl << "-------" << endl;
     cout << "Start processing sequence ..." << endl;
     cout << "Images in the sequence: " << nImages << endl << endl;
 
     int main_error = EX_OK;
     thread runthread([&]() {
         cv::Mat imLeft, imRight, imLeftRect, imRightRect;
         for (int ni = 0; ni < nImages; ni++) {
             imLeft  = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
             imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
 
             if (imLeft.empty()) {
                 cerr << "Failed to load image: " << vstrImageLeft[ni] << endl;
                 main_error = EX_DATAERR;
                 break;
             }
             if (imRight.empty()) {
                 cerr << "Failed to load image: " << vstrImageRight[ni] << endl;
                 main_error = EX_DATAERR;
                 break;
             }
             if (SLAM.isFinished()) break;
 
             // Use images directly (already rectified)
             imLeftRect  = imLeft;
             imRightRect = imRight;
 
             double tframe = vTimeStamp[ni];
             auto t1 = chrono::steady_clock::now();
             SLAM.TrackStereo(imLeftRect, imRightRect, tframe);
             auto t2 = chrono::steady_clock::now();
 
             vTimesTrack[ni] = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
 
             double T = (ni < nImages - 1) ? (vTimeStamp[ni+1] - tframe)
                                           : (tframe - vTimeStamp[ni-1]);
             if (vTimesTrack[ni] < T)
                 this_thread::sleep_for(chrono::duration<double>(T - vTimesTrack[ni]));
         }
         SLAM.StopViewer();
     });
 
     SLAM.StartViewer();
     cout << "Viewer started, waiting for thread." << endl;
     runthread.join();
     if (main_error != EX_OK) return main_error;
 
     SLAM.Shutdown();
     cout << "System Shutdown" << endl;
 
     sort(vTimesTrack.begin(), vTimesTrack.end());
     float totaltime = accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0.0f);
     cout << "-------" << endl << endl;
     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
     cout << "mean tracking time: " << totaltime / nImages << endl;
 
    //  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
     SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
     return main_error;
 }
 
 void LoadImages(const string &strPathLeft,
                 const string &strPathRight,
                 const string &strPathTimes,
                 vector<string> &vstrImageLeft,
                 vector<string> &vstrImageRight,
                 vector<double> &vTimeStamps) {
     if (!fs::exists(strPathTimes)) {
         cerr << "FATAL: Could not find timestamp file " << strPathTimes << endl;
         exit(EX_DATAERR);
     }
     ifstream fTimes(strPathTimes.c_str());
     string s;
     int idx = 0;
     while (getline(fTimes, s)) {
         if (s.empty()) continue;
         double t = stod(s);
         // Generate filename with zero-padded index (6 digits)
         stringstream fn;
         fn << setw(6) << setfill('0') << idx++;
         string imgName = fn.str();
         vstrImageLeft .push_back(strPathLeft  + "/" + imgName + ".png");
         vstrImageRight.push_back(strPathRight + "/" + imgName + ".png");
         vTimeStamps.push_back(t / 1e9);
     }
 }
 
 string FindFile(const string& baseFileName, const string& pathHint) {
     if (fs::exists(baseFileName)) return baseFileName;
     string candidate = pathHint + baseFileName;
     if (fs::exists(candidate)) return candidate;
     return baseFileName;
 }
 