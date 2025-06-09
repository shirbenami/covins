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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "Config.h" // Assuming this contains covins_params::comm::send_updates etc.

#include <mutex>
#include <chrono>
#include <iostream> // For debug output like COUTERROR

// COVINS includes for messages and utilities
#include <covins/covins_base/config_comm.hpp> // For covins_params::comm
#include <covins/comm_messages/MsgKeyframe.hpp> // For MsgKeyframe
#include <src/covins/covins_frontend/src/frontend_wrapper.hpp> //
namespace ORB_SLAM3
{

const char* COUTERROR = "[ERROR]"; // Simple error tag for cout

LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mbNewInit(false), mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), infoInertial(Eigen::MatrixXd::Zero(9,9)),
    mClientId(-1) // Initialize mClientId
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

// Implement the new SetFrontendWrapper method
void LocalMapping::SetFrontendWrapper(std::shared_ptr<covins::FrontendWrapper> pFrontendWrapper) {
    mpFrontendWrapper = pFrontendWrapper;
    if (mpFrontendWrapper) {
        mClientId = mpFrontendWrapper->getClientId(); // Get the client ID from FrontendWrapper
        std::cout << "LocalMapping: Set FrontendWrapper, client ID: " << mClientId << std::endl;
    }
}


void LocalMapping::Run()
{
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)
        {

#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            MapPointCulling();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif
            // Triangulate new MapPoints
            CreateNewMapPoints();

            mbAbortBA = false;

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {

                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +
                                cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter());

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            // Add debug prints for motion and time
                            std::cout << "[DEBUG] mTinit: " << mTinit
                                    << ", dist: " << dist
                                    << ", mTinit<10.f: " << (mTinit<10.f)
                                    << ", dist<0.02: " << (dist<0.02)
                                    << std::endl;

                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
#ifdef COVINS_MOD
                        vector<KeyFrame*> local_kfs;
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),local_kfs,num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        if(covins_params::comm::send_updates && mpFrontendWrapper && mpFrontendWrapper->getCommunicator() && mpFrontendWrapper->getCommunicator()->isConnected()) {
                            if(local_kfs.size() <= covins_params::comm::update_window_size) kf_out_buffer_.insert(local_kfs.begin(),local_kfs.end());
                            else kf_out_buffer_.insert(local_kfs.begin(),local_kfs.begin() + covins_params::comm::update_window_size);
                        }
#else
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
#endif
                        b_doneLBA = true;
                    }
                    else
                    {
#ifdef COVINS_MOD
                        list<KeyFrame*> local_kfs;
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA,local_kfs);
                        if(covins_params::comm::send_updates && mpFrontendWrapper && mpFrontendWrapper->getCommunicator() && mpFrontendWrapper->getCommunicator()->isConnected()) {
                            if(local_kfs.size() <= covins_params::comm::update_window_size) kf_out_buffer_.insert(local_kfs.begin(),local_kfs.end());
                            else {
                                int cntx=0;
                                for(const auto& kfx : local_kfs) {
                                    kf_out_buffer_.insert(kfx);
                                    cntx++;
                                    if(cntx>=covins_params::comm::update_window_size) break;
                                }
                            }
                        };
#else
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA);
#endif
                        b_doneLBA = true;
                    }

                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBASync_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // Initialize IMU here
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    std::cout << "[LocalMapping] IMU not initialized, mbInertial=" << mbInertial << std::endl;
                    if (mbMonocular)
                    {
                        std::cout << "[LocalMapping] Calling InitializeIMU for monocular" << std::endl;
                        InitializeIMU(1e2, 1e10, true);
                    }
                    else
                    {
                        std::cout << "[LocalMapping] Calling InitializeIMU for non-monocular" << std::endl;
                        InitializeIMU(1e2, 1e5, true);
                    }
                }


                // Check redundant local Keyframes
                #ifndef COVINS_MOD
                KeyFrameCulling();
                #endif

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<100.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK)
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true);
                                else
                                    InitializeIMU(1.f, 1e5, true);

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true);
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=100) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            cout << "start scale ref" << endl;
                            if (mbMonocular)
                                ScaleRefinement();
                            cout << "end scale ref" << endl;
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBA_ms.push_back(timeLBA_ms);
            vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

            #ifdef COVINS_MOD
            #ifndef NO_LOOP_FINDER
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            #endif
            #else
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            #endif

#ifdef COVINS_MOD
            //-----------------------------
            // This section takes care of sending data to the COVINS backend
            // Check if frontend wrapper and communicator are initialized and connected
            if(covins_params::comm::send_updates && mpFrontendWrapper && mpFrontendWrapper->getCommunicator() && mpFrontendWrapper->getCommunicator()->isConnected())
            {
                // Delay sending a bit to achieve more consistency in data association
                while(kf_out_buffer_.size() > covins_params::comm::kf_buffer_withold) {
                    KeyFrame* kfi = *(kf_out_buffer_.begin());
                    kf_out_buffer_.erase(kf_out_buffer_.begin());

                    covins::MsgKeyframe msg_kf;
                    // Determine the reference KeyFrame for ConvertToMsg.
                    // This is typically the predecessor KF in the local mapping sequence.
                    KeyFrame* kf_predecessor_for_msg = nullptr;
                    if (kfi->mPrevKF && kfi->mPrevKF->mnId != kfi->mnId) { // Ensure it's a valid predecessor
                        kf_predecessor_for_msg = kfi->mPrevKF;
                    }

                    // For the 'is_update' flag: A KF in kf_out_buffer_ can be new or updated by LBA.
                    // If the KF was part of a local bundle adjustment (LBA), it's an update.
                    // For now, assuming any KF being processed here might be an update.
                    bool is_update_kf = true; // Conservative: assume it might be an update for now.

                    // Populate MsgKeyframe from KeyFrame data
                    kfi->ConvertToMsg(msg_kf, kf_predecessor_for_msg, is_update_kf, mClientId);

                    // Send the populated message via FrontendWrapper
                    mpFrontendWrapper->sendKeyframeMessage(msg_kf);
                }
            }
            //-----------------------------
#endif

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

#ifdef COVINS_MOD
    // empty comm buffer on shutdown
    std::cout << ">>> LocalMapping: Emptying comm buffer on shutdown..." << std::endl;
    if(mpFrontendWrapper && mpFrontendWrapper->getCommunicator() && mpFrontendWrapper->getCommunicator()->isConnected()) {
        while(!kf_out_buffer_.empty()) {
            KeyFrame* kfi = *(kf_out_buffer_.begin());
            kf_out_buffer_.erase(kf_out_buffer_.begin());

            covins::MsgKeyframe msg_kf;
            KeyFrame* kf_predecessor_for_msg = nullptr;
            if (kfi->mPrevKF && kfi->mPrevKF->mnId != kfi->mnId) {
                kf_predecessor_for_msg = kfi->mPrevKF;
            }
            kfi->ConvertToMsg(msg_kf, kf_predecessor_for_msg, true, mClientId); // Treat as update on shutdown

            mpFrontendWrapper->sendKeyframeMessage(msg_kf);
        }
    } else {
        std::cout << ">>> LocalMapping: Communicator not connected, cannot empty buffer on shutdown." << std::endl;
    }
    std::cout << ">>> LocalMapping: Done emptying buffer." << std::endl;
#endif

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;

#ifdef COVINS_MOD
    kf_out_buffer_.insert(pKF);
#endif
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if(pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
    //cout << "erase MP: " << borrar << endl;
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    // For stereo inertial case
    if(mbMonocular)
        nn=20;
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    if (mbInertial)
    {
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count=0;
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th,false);

    auto Rcw1 = mpCurrentKeyFrame->GetRotation_();
    auto Rwc1 = Rcw1.t();
    auto tcw1 = mpCurrentKeyFrame->GetTranslation_();
    cv::Matx44f Tcw1{Rcw1(0,0),Rcw1(0,1),Rcw1(0,2),tcw1(0),
                     Rcw1(1,0),Rcw1(1,1),Rcw1(1,2),tcw1(1),
                     Rcw1(2,0),Rcw1(2,1),Rcw1(2,2),tcw1(2),
                     0.f,0.f,0.f,1.f};

    auto Ow1 = mpCurrentKeyFrame->GetCameraCenter_();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        auto Ow2 = pKF2->GetCameraCenter_();
        auto vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        auto F12 = ComputeF12_(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        bool bCoarse = mbInertial &&
                ((!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && mpCurrentKeyFrame->GetMap()->GetIniertialBA1())||
                 mpTracker->mState==Tracking::RECENTLY_LOST);

        matcher.SearchForTriangulation_(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false,bCoarse);

        auto Rcw2 = pKF2->GetRotation_();
        auto Rwc2 = Rcw2.t();
        auto tcw2 = pKF2->GetTranslation_();
        cv::Matx44f Tcw2{Rcw2(0,0),Rcw2(0,1),Rcw2(0,2),tcw2(0),
                         Rcw2(1,0),Rcw2(1,1),Rcw2(1,2),tcw2(1),
                         Rcw2(2,0),Rcw2(2,1),Rcw2(2,2),tcw2(2),
                         0.f,0.f,0.f,1.f};

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                               : true;

            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose_();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter_();

                    Rcw2 = pKF2->GetRightRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation_();
                    Tcw2 = pKF2->GetRightPose_();
                    Ow2 = pKF2->GetRightCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose_();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter_();

                    Rcw2 = pKF2->GetRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation_();
                    Tcw2 = pKF2->GetPose_();
                    Ow2 = pKF2->GetCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetPose_();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter_();

                    Rcw2 = pKF2->GetRightRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation_();
                    Tcw2 = pKF2->GetRightPose_();
                    Ow2 = pKF2->GetRightCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    Rcw1 = mpCurrentKeyFrame->GetRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetPose_();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter_();

                    Rcw2 = pKF2->GetRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation_();
                    Tcw2 = pKF2->GetPose_();
                    Ow2 = pKF2->GetCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
            }

            // Check parallax between rays
            auto xn1 = pCamera1->unprojectMat_(kp1.pt);
            auto xn2 = pCamera2->unprojectMat_(kp2.pt);

            auto ray1 = Rwc1*xn1;
            auto ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Matx31f x3D;
            bool bEstimated = false;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
               (cosParallaxRays<0.9998 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                // Linear Triangulation Method
                cv::Matx14f A_r0 = xn1(0) * Tcw1.row(2) - Tcw1.row(0);
                cv::Matx14f A_r1 = xn1(1) * Tcw1.row(2) - Tcw1.row(1);
                cv::Matx14f A_r2 = xn2(0) * Tcw2.row(2) - Tcw2.row(0);
                cv::Matx14f A_r3 = xn2(1) * Tcw2.row(2) - Tcw2.row(1);
                cv::Matx44f A{A_r0(0), A_r0(1), A_r0(2), A_r0(3),
                              A_r1(0), A_r1(1), A_r1(2), A_r1(3),
                              A_r2(0), A_r2(1), A_r2(2), A_r2(3),
                              A_r3(0), A_r3(1), A_r3(2), A_r3(3)};

                cv::Matx44f u,vt;
                cv::Matx41f w;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                cv::Matx41f x3D_h = vt.row(3).t();

                if(x3D_h(3)==0)
                    continue;

                // Euclidean coordinates
//                x3D = x3D_h.get_minor<3,1>(0,0) / x3D_h(3);
                x3D = cv::Matx31f(x3D_h.get_minor<3,1>(0,0)(0) / x3D_h(3), x3D_h.get_minor<3,1>(0,0)(1) / x3D_h(3), x3D_h.get_minor<3,1>(0,0)(2) / x3D_h(3));
                bEstimated = true;

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo_(idx1);
                bEstimated = true;
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo_(idx2);
                bEstimated = true;
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            cv::Matx13f x3Dt = x3D.t();

            if(!bEstimated) continue;
            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            auto normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            auto normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints))
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            cv::Mat x3D_(x3D);
