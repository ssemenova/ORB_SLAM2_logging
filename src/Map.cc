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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");
    
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");
    
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    auto timer_start = StartTimer();
    unique_lock<mutex> lock(mMutexMap);
    EndTimerAndPrint(timer_start, "waiting on mutex");

    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

std::chrono::time_point<std::chrono::high_resolution_clock> Map::StartTimer()
{
    auto timer_start = std::chrono::high_resolution_clock::now();
    return timer_start;
}

void Map::EndTimerAndPrint(std::chrono::time_point<std::chrono::high_resolution_clock> timer_start, std::string print) 
{
    auto timer_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start);
    auto tid = std::this_thread::get_id();
    if (duration.count() > 0) {
        cout << "Sofiya-LMTest," << tid << "," << print << "," << duration.count() << endl;
    }
}


} //namespace ORB_SLAM
