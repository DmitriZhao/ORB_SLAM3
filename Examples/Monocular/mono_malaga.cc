/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<algorithm>
#include<chrono>
#include<filesystem>
#include<fstream>
#include<iomanip>
#include<iostream>
#include<string>
#include<string_view>
#include<set>

#include<opencv2/core/core.hpp>

#include<System.h>

namespace fs = std::filesystem;

void LoadImages(const fs::path& strPathToSequence, vector<fs::path>& vstrImageLeft, vector<double>& vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_malaga path_to_vocabulary path_to_settings path_to_sequence (trajectory_file_name)" << endl;
        return 1;
    }
    bool bFileName = (argc == 5);
    fs::path file_name;
    if (bFileName)
        file_name = argv[4];
    std::cout << "trajectory_file_name: " << file_name << std::endl;

    fs::path pathToVocabulary(argv[1]);
    fs::path pathToSettings(argv[2]);
    fs::path pathToSequence(argv[3]);

    // Retrieve paths to images
    std::vector<fs::path> vstrImageLeft;
    std::vector<double> vTimestamps;
    LoadImages(pathToSequence, vstrImageLeft, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(pathToVocabulary, pathToSettings, ORB_SLAM3::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);  

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageLeft[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackMonocular(imLeft, tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[ni]);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(file_name.parent_path() / ("f_" + file_name.filename().string() + "_tum.txt"));
    SLAM.SaveKeyFrameTrajectoryTUM(file_name.parent_path() / ("kf_" + file_name.filename().string() + "_tum.txt"));

    return 0;
}

void LoadImages(const fs::path& strPathToSequence, vector<fs::path>& vstrImageLeft, vector<double>& vTimestamps)
{

    if (!fs::exists(strPathToSequence))
    {
        std::cerr << "ERROR: Path to sequence does not exist!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    //--- filenames are unique so we can use a set
    std::set<fs::path> filenamesSorted;
    for (const auto& entry : fs::directory_iterator(strPathToSequence))
    {
        // check if file is an images
        if (entry.path().extension() == ".jpg")
        {
            filenamesSorted.insert(entry.path());
        }
    }

    for (const auto& entry : filenamesSorted)
    {
        // check if file is an images
        if (entry.stem().string().ends_with("_left"sv))
        {
            vstrImageLeft.push_back(entry);
            // extract timestamp from filename. Format: img_CAMERA1_1261228749.918590_left.jpg
            std::string filename = entry.filename();
            filename = filename.substr(0, filename.find_last_of("_"));
            std::string timestamp_string = filename.substr(filename.find_last_of("_") + 1);
            vTimestamps.push_back(std::stod(timestamp_string));
        }
        else if (entry.stem().string().ends_with("_right"sv))
        {
            continue;
        }
        else
        {
            std::cerr << "ERROR: Image file does not end with _left or _right!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    std::cout << "Found " << vstrImageLeft.size() << " images in " << strPathToSequence << std::endl;
    double fps = vstrImageLeft.size() / (vTimestamps.back() - vTimestamps.front());
    std::cout << "fps: " << fps << std::endl;
    assert(vstrImageLeft.size() == vTimestamps.size());
}
