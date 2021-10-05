/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock(mMutexFeatures);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mObservations;
}

int MapPoint::Observations()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
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

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock1(mMutexFeatures);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
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
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock1(mMutexFeatures);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexFeatures);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        EndTimerAndPrint(timer_start, "waiting on mutex");

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
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock3(mMutexPos);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexPos);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        auto timer_start = StartTimer();
        unique_lock<mutex> lock(mMutexPos);
        EndTimerAndPrint(timer_start, "waiting on mutex");

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
        auto timer_start = StartTimer();
        unique_lock<mutex> lock(mMutexPos);
        EndTimerAndPrint(timer_start, "waiting on mutex");

        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

std::chrono::time_point<std::chrono::high_resolution_clock> MapPoint::StartTimer()
{
    auto timer_start = std::chrono::high_resolution_clock::now();
    return timer_start;
}

void MapPoint::EndTimerAndPrint(std::chrono::time_point<std::chrono::high_resolution_clock> timer_start, std::string print) 
{
    auto timer_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start);
    auto tid = std::this_thread::get_id();
    if (duration.count() > 0) {
        cout << "Sofiya-LMTest," << tid << "," << print << "," << duration.count() << endl;
    }
}


} //namespace ORB_SLAM
