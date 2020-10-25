#include "segment.h"
#include <vector>
#include "dhMath.h"
#include "dhApplication.h"

segment::segment()
{

}

void segment::getCoGs(segment* segm,vector<dhVec3>& cogs,int index,vector<int>& key){
    if(segm[index].ObjectPoints.empty() == true){;
//        DH_LOG("path1",0);
    }
    else{
//        DH_LOG("path2",0);
        cogs.push_back(ObjectCoG);
        key.push_back(index);
    }
}
