// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


// TODO:  
// + obstacle avoidance:
//    https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#How_the_obstacle_avoidance_works

#include "map.h"
#include <Arduino.h>
#include "config.h"


void Map::begin(){
  targetWaypointIdx = 0;
  waypointsCount = 0;
  for (int i=0; i < MAX_POINTS; i++){
    waypoints[i].x=0;
    waypoints[i].y=0;
  }  
}

bool Map::setWaypoint(int idx, int count, float x, float y){
  if ((idx >= MAX_POINTS) || (count > MAX_POINTS)) return false;  
  targetWaypointIdx = 0;  
  waypoints[idx].x = x;
  waypoints[idx].y = y;
  waypointsCount = count;  
  return true;
}

// 1.0 = 100%
void Map::setTargetWaypointPercent(float perc){
  targetWaypointIdx = (int)( ((float)waypointsCount) * perc);
  if (targetWaypointIdx >= waypointsCount) {
    targetWaypointIdx = waypointsCount-1;
  }
}

void Map::run(){
  targetPoint = waypoints[targetWaypointIdx];  
}

float Map::distanceToTargetPoint(float stateX, float stateY){  
  float dX = targetPoint.x - stateX;
  float dY = targetPoint.y - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

bool Map::nextWaypoint(){
  if (targetWaypointIdx+1 < waypointsCount){
    // next waypoint
    lastTargetPoint = targetPoint;
    targetWaypointIdx++;
    return true;
  } else {
    // finish        
    targetWaypointIdx=0;    
    return false;
  }       
}

void Map::setLastTargetPoint(float stateX, float stateY){
  lastTargetPoint.x = stateX; 
  lastTargetPoint.y = stateY;
}

