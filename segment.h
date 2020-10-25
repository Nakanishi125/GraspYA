#ifndef SEGMENT_H
#define SEGMENT_H

#include<vector>
#include"dhMath.h"

class segment
{
public:
    vector<dhVec3> ObjectPoints;
    dhVec3 ObjectCoG;
    dhVec3 ObjectNormal;

    segment();
    void getCoGs(segment* segm,vector<dhVec3>& cogs,int index,vector<int>& key);

};

#endif // SEGMENT_H
