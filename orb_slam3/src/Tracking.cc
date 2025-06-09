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


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Initializer.h"
#include "G2oTypes.h"
#include "Optimizer.h"

#include <iostream>

#include <mutex>
#include <chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>

// COVINS - REMOVED: Old communication include that causes linker errors
// #include <comm/communicator.hpp> // for NO_LOOP_FINDER

// COVINS - Add include for utility functions, if Converter or other files in ORB-SLAM3 call them
// This is crucial for resolving 'covins::Utils' undefined references.
#include <covins/covins_base/utils_base.hpp>

using namespace std;

namespace ORB_SLAM3
{


Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0), time_recently_lost_visual(2.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false;

    mnNumDataset = 0;

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();

    vdUpdatedLM_ms.clear();
    vdSearchLP_ms.clear();
    vdPoseOpt_ms.clear();
#endif

    vnKeyFramesLM.clear();
    vnMapPointsLM.clear();
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBA_ms[i] << ","
          << mpLocalMapper->vdKFCulling_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#KF insert[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdStereoMatch_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();

    f.open("TrackLocalMapStats.txt");
    f << fixed << setprecision(6);

    f << "# number of KF, number of MP, UpdateLM[ms], SearchLP[ms], PoseOpt[ms]" << endl;

    for(int i=0; i<vnKeyFramesLM.size(); ++i)
    {

        f << vnKeyFramesLM[i] << "," << vnMapPointsLM[i] <<  "," << vdUpdatedLM_ms[i] << "," << vdSearchLP_ms[i] << "," << vdPoseOpt_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecTimeMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Map Tracking complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Local Map Tracking complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Local Map Tracking complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(vnKeyFramesLM);
    deviation = calcDeviation(vnKeyFramesLM, average);
    std::cout << "Local KFs: " << average << "$\\pm$" << deviation << std::endl;
    f << "Local KFs: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vnMapPointsLM);
    deviation = calcDeviation(vnMapPointsLM, average);
    std::cout << "Local MPs: " << average << "$\\pm$" << deviation << std::endl;
    f << "Local MPs: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBASync_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBASync_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCullingSync_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCullingSync_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllMaps()[0]->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMaps()[0]->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    f << "KFs in map: " << mpAtlas->GetAllMaps()[0]->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << mpAtlas->GetAllMaps()[0]->GetAllMapPoints().size() << std::endl;


    // Place recognition time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Place Recognition (mean$\\pm$std)" << std::endl << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl << std::endl;

    average = calcAverage(mpLoopClosing->vTimeBoW_ms);
    deviation = calcDeviation(mpLoopClosing->vTimeBoW_ms, average);
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vTimeSE3_ms);
    deviation = calcDeviation(mpLoopClosing->vTimeSE3_ms, average);
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vTimePRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vTimePRTotal_ms, average);
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl;

    // Loop Closing time stats
    if(mpLoopClosing->vTimeLoopTotal_ms.size() > 0)
    {
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Loop Closing (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeLoopTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeLoopTotal_ms, average);
        std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(mpLoopClosing->vTimeMergeTotal_ms.size() > 0)
    {
        // Map Merging time stats
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Map Merging (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeMergeTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeMergeTotal_ms, average);
        std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl;
    }


    if(mpLoopClosing->vTimeGBATotal_ms.size() > 0)
    {
        // Full GBA time stats
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Full GBA (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeFullGBA_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeFullGBA_ms, average);
        std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vTimeMapUpdate_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeMapUpdate_ms, average);
        std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vTimeGBATotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeGBATotal_ms, average);
        std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl;
    }

    f.close();

}

#endif

Tracking::~Tracking()
{

}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];

    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        // Camera calibration parameters

        cv::FileNode node = fSettings["Camera.fx"];

        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpAtlas->AddCamera(mpCamera);


        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;
                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;
                mpAtlas->AddCamera(mpCamera2); // Add the second camera
            }
        }
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(b_miss_params)
    {
        return false;
    }

    // Params for Imu_Cam Transform
    cv::Mat Tbc;
    node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    std::cout << std::endl;
    std::cout << "Left camera to Imu Transform (Tbc): " << std::endl << Tbc << std::endl;

    cv::cv2eigen(Tbc, mImuCalib.Tbc);

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_g = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_a = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_gw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_aw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroBiasInit"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_bg = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroBiasInit parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccBiasInit"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.sigma_ba = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccBiasInit parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.minFrames = node.real();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.InitTime"];
    if(!node.empty() && node.isReal())
    {
        mImuCalib.initTime = node.real();
    }
    else
    {
        std::cerr << "*IMU.InitTime parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.MaxFrames"];
    if(!node.empty() && node.isInt())
    {
        mMaxFrames = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.MaxFrames parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    std::cout << std::endl << "IMU Parameters: " << std::endl;
    std::cout << "- Accel Noise: " << mImuCalib.sigma_a << std::endl;
    std::cout << "- Gyro Noise: " << mImuCalib.sigma_g << std::endl;
    std::cout << "- Accel Walk: " << mImuCalib.sigma_aw << std::endl;
    std::cout << "- Gyro Walk: " << mImuCalib.sigma_gw << std::endl;
    std::cout << "- Accel Bias Init: " << mImuCalib.sigma_ba << std::endl;
    std::cout << "- Gyro Bias Init: " << mImuCalib.sigma_bg << std::endl;
    std::cout << "- IMU Frequency: " << mImuCalib.minFrames << std::endl;
    std::cout << "- IMU Init Time: " << mImuCalib.initTime << std::endl;
    std::cout << "- Max Frames for Initialization: " << mMaxFrames << std::endl;

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;
    bool b_miss_params = false;

    // Common ORB parameters
    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor == System::STEREO && !fSettings["ORBextractor.nFeatures_left"].empty() && fSettings["ORBextractor.nFeatures_left"].isInt()){
        int nFeaturesLeft = fSettings["ORBextractor.nFeatures_left"].operator int();
        mpORBextractorLeft = new ORBextractor(nFeaturesLeft,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    } else if (mSensor == System::STEREO) {
        mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }
    if(mSensor == System::STEREO && !fSettings["ORBextractor.nFeatures_right"].empty() && fSettings["ORBextractor.nFeatures_right"].isInt()){
        int nFeaturesRight = fSettings["ORBextractor.nFeatures_right"].operator int();
        mpORBextractorRight = new ORBextractor(nFeaturesRight,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    } else if (mSensor == System::STEREO){
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }

    std::cout << std::endl << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    // Stereo only parameters (for rectified cameras)
    if(mSensor==System::STEREO && !fSettings["Stereo.ThDepth"].empty())
    {
        mThDepth = fSettings["Stereo.ThDepth"];
    }
    else
    {
        mThDepth = 35.0; //Default value
        std::cout << "Stereo.ThDepth parameter doesn't exist. Setting to default: " << mThDepth << std::endl;
    }

    if(mSensor==System::STEREO)
    {
        if(!fSettings["Stereo.fx"].empty())
        {
            float fx = fSettings["Stereo.fx"];
            mbf = fx*0.06; //Baseline multiplied by fx
            std::cout << "- fx: " << fx << std::endl;
        }
        else
        {
            mbf = 0.0;
            std::cout << "*Stereo.fx parameter doesn't exist. Assuming 0.0*" << std::endl;
        }
    }


    if(mSensor==System::STEREO && !fSettings["ThDepth"].empty())
    {
        std::cerr << "WARNING: ThDepth parameter name changed to Stereo.ThDepth. Please update settings file" << std::endl;
    }

    // Min and Max frames between KeyFrames
    mMinFrames = 0;
    mMaxFrames = fSettings["KeyFrame.MaxFrames"];
    if(mMaxFrames==0)
    {
        mMaxFrames = 1000;
        std::cout << "KeyFrame.MaxFrames parameter doesn't exist. Setting to default: " << mMaxFrames << std::endl;
    }

    // For monocular-inertial, keyframes must be declared more quickly, since IMU accumulates bias quickly.
    if(mSensor == System::IMU_MONOCULAR)
    {
        mMaxFrames = 100;
        std::cout << "IMU-Monocular: KeyFrame.MaxFrames set to 100" << std::endl;
    }
    else if(mSensor == System::IMU_STEREO)
    {
        mMaxFrames = 100;
        std::cout << "IMU-Stereo: KeyFrame.MaxFrames set to 100" << std::endl;
    }

    std::cout << "- Max Frames between KeyFrames: " << mMaxFrames << std::endl;

    std::cout << std::endl;
    return true;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;

    if (mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }

    mCurrentFrame = Frame(mImGray,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mImuCalib,mSensor,filename);

    // TODO: The `GrabImageMonocular` (and Stereo/RGBD) methods are the entry points where `FrontendWrapper`
    // should potentially be used to send the image/odometry/keyframe.
    // The current implementation directly constructs `Frame` objects.
    // You would need to pass `mpFrontendWrapper` down to where `Frame` is created or where
    // keyframes are decided, and then use `mpFrontendWrapper->sendKeyframe(...)`
    // or `mpFrontendWrapper->sendImage(...)` etc.
    // This requires refactoring the internals of ORB-SLAM3 (e.g., in LocalMapping)
    // to use the new communication structure.

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if (mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }

    if (imGrayRight.channels()==3)
    {
        if(mbRGB)
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        else
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mImuCalib,mSensor,filename);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename)
{
    mImGray = imRGB;
    mImDepth = imD;

    if (mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }

    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mImuCalib,mSensor,filename);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::GrabImuData(const IMU::Point &imu_meas)
{
    mCurrentFrame.AddImu(imu_meas);
}


void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastFrame = Fr