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


#include "MapPoint.h"
#include "KeyFrame.h" // Needed for KeyFrame class definition used by MapPoint
#include "Map.h"      // Needed for Map class definition used by MapPoint
#include "ORBextractor.h"
#include "Converter.h"

#include <mutex>
#include <algorithm> // For std::min, std::max
#include <iostream>  // For COUTERROR

// COVINS includes for messages and utilities
#include <covins/comm_messages/MsgLandmark.hpp> // For MsgLandmark
#include <covins/covins_base/utils_base.hpp> // For covins::Utils::ToEigenVec3d


namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId = 0;
std::mutex MapPoint::mGlobalMutex; // Ensure this is defined once

MapPoint::MapPoint():
    mnId(nNextId++), mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mnOriginMapId(0), mpHostKF(static_cast<KeyFrame*>(NULL)),
    mInvDepth(-1)
{
    // Default initialize the Eigen members to zero or identity
    mWorldPosx = cv::Matx31f::zeros();
    mNormalVectorx = cv::Matx31f::zeros();
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap):
    mnId(nNextId++), mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mpRefKF(pRefKF), mpMap(pMap), mnOriginMapId(pRefKF->GetMap()->GetId()),
    mpHostKF(static_cast<KeyFrame*>(NULL)), mInvDepth(-1)
{
    mWorldPos = Pos.clone();
    cv::cv2eigen(mWorldPos, mWorldPosx);

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
    mNormalVectorx = cv::Matx31f::zeros();

    // Reference KeyFrame
    // mpRefKF = pRefKF;

    // Map
    // mpMap = pMap;

    // Add to Map
    mpMap->AddMapPoint(this);

    // Compute Distinctive Descriptor
    ComputeDistinctiveDescriptors();

    // Update Normal and Depth
    UpdateNormalAndDepth();
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap):
    mnId(nNextId++), mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mpRefKF(pRefKF), mpMap(pMap), mnOriginMapId(pRefKF->GetMap()->GetId()),
    mpHostKF(pHostKF), mInvDepth(invDepth)
{
    // Note: pHostKF->UnprojectStereo(uv_init.x, uv_init.y, invDepth) is not a standard KeyFrame method.
    // Assuming a helper function or direct calculation is intended here to get 3D point.
    // For now, setting mWorldPos to zero and relying on subsequent updates or a different creation path.
    // If you have a specific implementation for KeyFrame::UnprojectStereo that takes (u,v,invDepth),
    // you'll need to ensure that's correctly defined in KeyFrame.h/cc.
    mWorldPos = cv::Mat::zeros(3,1,CV_32F); // Placeholder, assuming it's set later or by other means
    // cv::Mat x3D = pHostKF->UnprojectStereo(uv_init.x, uv_init.y, invDepth);
    // mWorldPos = x3D.clone();
    cv::cv2eigen(mWorldPos, mWorldPosx);

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
    mNormalVectorx = cv::Matx31f::zeros();

    // Add to Map
    mpMap->AddMapPoint(this);

    // Compute Distinctive Descriptor
    ComputeDistinctiveDescriptors();

    // Update Normal and Depth
    UpdateNormalAndDepth();
}

MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF):
    mnId(nNextId++), mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(NULL), mpRefKF(static_cast<KeyFrame*>(NULL)), mpMap(pMap), mnOriginMapId(pMap->GetId()),
    mpHostKF(static_cast<KeyFrame*>(NULL)), mInvDepth(-1)
{
    mWorldPos = Pos.clone();
    cv::cv2eigen(mWorldPos, mWorldPosx);

    cv::Mat Ow;
    // Assuming pFrame->NLeft is correctly defined and accessible
    if(pFrame->NLeft == -1 || idxF < pFrame->NLeft){
        Ow = pFrame->GetCameraCenter();
    }
    else{
        cv::Mat Rwl = pFrame->mRwc; // Assuming mRwc is world to camera rotation
        cv::Mat tlr = pFrame->mTlr.col(3); // Assuming mTlr is transform from Left to Right camera
        cv::Mat twl = pFrame->mOw; // Assuming mOw is world origin to left camera

        // Calculate right camera center in world coordinates
        Ow = Rwl * tlr + twl;
    }
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);
    mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));


    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    int level = 0; // Default or calculate based on the frame type
    if (pFrame->NLeft == -1) { // Monocular
        level = pFrame->mvKeysUn[idxF].octave;
    } else if (idxF < pFrame->NLeft) { // Left stereo
        level = pFrame->mvKeys[idxF].octave;
    } else { // Right stereo
        level = pFrame->mvKeysRight[idxF - pFrame->NLeft].octave;
    }

    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(pMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex); // Using MapPoint::mGlobalMutex
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
    mWorldPosx = cv::Matx31f(Pos.at<float>(0), Pos.at<float>(1), Pos.at<float>(2));
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

cv::Matx31f MapPoint::GetWorldPos2()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosx;
}

cv::Matx31f MapPoint::GetNormal2()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVectorx;
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    std::tuple<int,int> indexes; // Use std::tuple explicitly

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = std::make_tuple(-1,-1); // Use std::make_tuple
    }

    if(pKF->NLeft != -1 && idx >= pKF->NLeft){
        std::get<1>(indexes) = idx; // Use std::get
    }
    else{
        std::get<0>(indexes) = idx; // Use std::get
    }

    mObservations[pKF]=indexes;

    if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0) // Check if it's a stereo observation in original ORB-SLAM3
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            std::tuple<int,int> indexes = mObservations[pKF]; // Use std::tuple
            int leftIndex = std::get<0>(indexes); // Use std::get
            int rightIndex = std::get<1>(indexes); // Use std::get

            if(leftIndex != -1){
                if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
            {
                // Select new reference keyframe
                if(!mObservations.empty())
                    mpRefKF=mObservations.begin()->first;
                else
                    mpRefKF=static_cast<KeyFrame*>(NULL); // No more observations
            }

            // If only 2 observations or less, discard point
            if(nObs<=2) // Heuristic for culling, can be adjusted
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag(); // This calls Map::EraseMapPoint
}


std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*, std::tuple<int,int>> obs; // Use std::tuple
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*, std::tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++) // Use std::tuple
    {
        KeyFrame* pKF = mit->first;

        std::tuple<int,int> indexes = mit->second; // Use std::tuple
        int leftIndex = std::get<0>(indexes); // Use std::get
        int rightIndex = std::get<1>(indexes); // Use std::get

        if(leftIndex != -1){
            pKF->EraseMapPointMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2); // Lock both mutexes

    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,std::tuple<int,int>> obs; // Use std::tuple
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,std::tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++) // Use std::tuple
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        std::tuple<int,int> indexes = mit->second; // Use std::tuple
        int leftIndex = std::get<0>(indexes); // Use std::get
        int rightIndex = std::get<1>(indexes); // Use std::get

        if(!pMP->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            // If the replacing MP already has an observation in this KF,
            // then the original MP's observation in this KF should be erased.
            if(leftIndex != -1){
                pKF->EraseMapPointMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2); // Lock both mutexes

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    std::vector<cv::Mat> vDescriptors; // Use std::vector

    std::map<KeyFrame*,std::tuple<int,int>> observations; // Use std::map, std::tuple

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++) // Use std::map, std::tuple
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad()){
            std::tuple<int,int> indexes = mit->second; // Use std::tuple
            int leftIndex = std::get<0>(indexes); // Use std::get
            int rightIndex = std::get<1>(indexes); // Use std::get

            if(leftIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N]; // C-style array, ok for stack if N is small, otherwise use std::vector<std::vector<float>> or Eigen
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        std::vector<int> vDists(Distances[i],Distances[i]+N); // Use std::vector
        std::sort(vDists.begin(),vDists.end()); // Use std::sort
        int median = vDists[ (int)(0.5*(N-1)) ]; // Correct median index calculation

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

std::tuple<int,int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) // Use std::tuple
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return std::make_tuple(-1,-1); // Use std::make_tuple
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    std::map<KeyFrame*,std::tuple<int,int>> observations; // Use std::map, std::tuple
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(std::map<KeyFrame*,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++) // Use std::map, std::tuple
    {
        KeyFrame* pKF = mit->first;

        std::tuple<int,int> indexes = mit->second; // Use std::tuple
        int leftIndex = std::get<0>(indexes); // Use std::get
        int rightIndex = std::get<1>(indexes); // Use std::get

        if(leftIndex != -1){
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
        if(rightIndex != -1){
            cv::Mat Owi = pKF->GetRightCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
            n++;
        }
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);

    std::tuple<int ,int> indexes = observations[pRefKF]; // Use std::tuple
    int leftIndex = std::get<0>(indexes); // Use std::get
    int rightIndex = std::get<1>(indexes); // Use std::get
    int level;
    if(pRefKF->NLeft == -1){ // Monocular
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if(leftIndex != -1){ // Left stereo
        level = pRefKF->mvKeys[leftIndex].octave;
    }
    else{ // Right stereo
        level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
    }


    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
        mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));
    }
}

void MapPoint::SetNormalVector(cv::Mat& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
    mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

Map* MapPoint::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

#ifdef COVINS_MOD
auto MapPoint::ConvertToMsg(covins::MsgLandmark &msg, KeyFrame *kf_ref, bool is_update, size_t client_id)->void {
    std::unique_lock<std::mutex> loc_obs(mMutexFeatures);
    std::unique_lock<std::mutex> lock_pos(mMutexPos);

    msg.is_new = !is_update; // If it's an update, it's not new (is_new should be false)

    msg.id.first = mnId;
    msg.id.second = client_id; // Use client_id provided

    if(!kf_ref) {
        std::cerr << "COUTERROR" << " MapPoint::ConvertToMsg: No kf_ref given for MapPoint " << mnId << std::endl;
        // Optionally handle this error more gracefully, e.g., by returning or setting a flag
        // For now, it will exit, but in a real-time system, you might want to log and skip.
        // exit(-1); // Removed for robustness, prefer returning if possible
        return; // Return if no reference KF, to avoid crash.
    }

    msg.id_reference.first = kf_ref->mnId;
    msg.id_reference.second = client_id; // Reference KF also gets the same client ID

    // Position in world coordinates
    msg.position = covins::Utils::ToEigenVec3d(mWorldPos);

    // Descriptor
    msg.descriptor = mDescriptor.clone();

    // MapPoint observations
    if (!is_update) { // Only send observations if it's a new landmark (or full update)
        for(const auto& obs_pair : mObservations) { // Iterate through the map
            KeyFrame* pKF = obs_pair.first;
            std::tuple<int,int> indexes = obs_pair.second; // Use std::tuple

            if (pKF != nullptr && !pKF->isBad()) { // Ensure KeyFrame is valid
                // Add observations to the message (KeyFrame ID, Client ID, Feature Index)
                msg.observations.insert(std::make_pair(std::make_pair(pKF->mnId, client_id), std::get<0>(indexes)));
                // If there's a right feature observation, add it too
                if (std::get<1>(indexes) != -1) {
                    msg.observations.insert(std::make_pair(std::make_pair(pKF->mnId, client_id), std::get<1>(indexes)));
                }
            }
        }
    }
    // Note: pos_ref field was removed from MsgLandmark (or was not expected here based on your MsgLandmark.h).
    // If you need to send position relative to reference frame, you should calculate it here.
    // Based on the provided MsgLandmark.h, it only has 'position' as world coordinates.
}
#endif

} //namespace ORB_SLAM3
