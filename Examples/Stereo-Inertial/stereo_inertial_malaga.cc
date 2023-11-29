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

void LoadImages(const fs::path& strPathToSequence, vector<fs::path>& vstrImageLeft,
                vector<fs::path>& vstrImageRight, vector<double>& vTimestamps);

void LoadIMU(const fs::path& strPathToSequence, vector<double> &vTimestamps,
            vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        std::cerr << std::endl << "Usage: ./stereo_inertial_malaga path_to_vocabulary path_to_settings path_to_sequence path_to_imudata (trajectory_file_name)" << std::endl;
        return 1;
    }
    bool bFileName = (argc == 6);
    fs::path file_name;
    if (bFileName)
        file_name = argv[5];
    std::cout << "trajectory_file_name: " << file_name << std::endl;

    fs::path pathToVocabulary(argv[1]);
    fs::path pathToSettings(argv[2]);
    fs::path pathToSequence(argv[3]);
    fs::path pathImu(argv[4]);

    // Retrieve paths to images
    std::cout << "Loading images for sequence " <<"...";
    std::vector<fs::path> vstrImageLeft;
    std::vector<fs::path> vstrImageRight;
    std::vector<double> vTimestampsCam;
    LoadImages(pathToSequence, vstrImageLeft, vstrImageRight, vTimestampsCam);
    std::cout << "LOADED!" << std::endl;


    // JC Modified: IMU variable
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImu;
    int first_imu=0;

    // JC Modified: IMU Load
    std::cout << "Loading IMU for sequence " << "...";
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    std::cout << "LOADED!" << std::endl;
    nImu = vTimestampsImu.size();

    const int nImages = vstrImageLeft.size();
    if((nImages<=0)||(nImu<=0))
    {
        std::cerr << "ERROR: Failed to load images or IMU for sequence" << std::endl;
        return 1;
    }

    // JC Modified:
    // Find first imu to be considered, supposing imu measurements start first
    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
    {
        first_imu++;
    }
    first_imu--; // first imu measurement to be considered


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(pathToVocabulary, pathToSettings, ORB_SLAM3::System::IMU_STEREO, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);  

    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;   


    // Main loop
    cv::Mat imLeft, imRight;
    // JC Modified 
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_rect = 0;
    int num_rect = 0;
    int proccIm = 0;
    std::cout << "Imu data num in the sequence: " << nImu << std::endl << std::endl;   

    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsCam[ni];

        if(imLeft.empty())
        {
            std::cerr << std::endl << "Failed to load image at: " << vstrImageLeft[ni] << std::endl;
            return 1;
        }

        // JC Modified
        if(imRight.empty())
        {
            std::cerr << std::endl << "Failed to load right image at: "
                    << string(vstrImageRight[ni]) << std::endl;
            return 1;
        }

        // JC Modified: Load imu measurements from previous frame
        //        将相邻帧之间的IMU数据存入vImuMeas，用于Track
        vImuMeas.clear();
        if (ni>0)
        {
            while(vTimestampsImu[first_imu]<=vTimestampsCam[ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                            vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                            vTimestampsImu[first_imu]));
                first_imu++;
            }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // JC Modified：Pass the images to the SLAM system
        // SLAM.TrackStereo(imLeft,imRight,tframe);
        SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni + 1] - tframe;
        else if(ni>0)
            T = tframe - vTimestampsCam[ni - 1];

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
    cout << "-------" << std::endl << std::endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << std::endl;
    cout << "mean tracking time: " << totaltime/nImages << std::endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(file_name.parent_path() / ("f_" + file_name.filename().string() + "_tum.txt"));
    SLAM.SaveKeyFrameTrajectoryTUM(file_name.parent_path() / ("kf_" + file_name.filename().string() + "_tum.txt"));

    return 0;
}

void LoadImages(const fs::path& strPathToSequence, vector<fs::path>& vstrImageLeft,
                vector<fs::path>& vstrImageRight, vector<double>& vTimestamps)
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
            vstrImageRight.push_back(entry);
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
    assert(vstrImageLeft.size() == vstrImageRight.size());
    assert(vstrImageLeft.size() == vTimestamps.size());
}

// format: %           Time              IMU_X_ACC              IMU_Y_ACC              IMU_Z_ACC            IMU_YAW_VEL          IMU_PITCH_VEL           IMU_ROLL_VEL              IMU_X_VEL              IMU_Y_VEL              IMU_Z_VEL                IMU_YAW              IMU_PITCH               IMU_ROLL                  IMU_X                  IMU_Y                  IMU_Z 
void LoadIMU(const fs::path& strImuPath, vector<double> &vTimestamps,
            vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    if (!fs::exists(strImuPath))
    {
        std::cerr << "ERROR: Path to sequence does not exist!" << std::endl;
        exit(EXIT_FAILURE);
    }
    assert(strImuPath.extension() == ".txt");
    assert(strImuPath.stem().string().ends_with("_imu"sv));

    std::ifstream fImu;
    fImu.open(strImuPath);
    vTimestamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;
        std::stringstream ss;
        ss << s;
        double t;  // in seconds
        ss >> t;
        vTimestamps.push_back(t);
        cv::Point3f acc, gyro;
        ss >> acc.x >> acc.y >> acc.z >> gyro.z >> gyro.y >> gyro.x;
        vAcc.push_back(acc);
        vGyro.push_back(gyro);
    }
}
