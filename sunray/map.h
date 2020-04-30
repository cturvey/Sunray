// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
/*
  mapping
*/

#ifndef MAP_H
#define MAP_H

#include <Arduino.h>

#define MAX_POINTS 5000


struct pt_t {
  float x; 
  float y;  
};

typedef struct pt_t pt_t;


class Map
{
  public:    
    // the line defined by (lastTargetPoint, targetPoint) is the current line to mow
    pt_t targetPoint; // target point
    pt_t lastTargetPoint; // last target point
    // all lines to mow are stored as a sequence of waypoints
    int targetWaypointIdx; // next waypoint in waypoint list
    int waypointsCount;
    pt_t waypoints[MAX_POINTS];
    void begin();    
    void run();    
    // set waypoint coordinate
    bool setWaypoint(int idx, int count, float x, float y);    
    // choose target point (0..100%) from waypoint list
    void setTargetWaypointPercent(float perc);
    // set last target point
    void setLastTargetPoint(float stateX, float stateY);
    // distance to target waypoint
    float distanceToTargetPoint(float stateX, float stateY);    
    // go to next waypoint
    bool nextWaypoint();
  private:
    
};



#endif
