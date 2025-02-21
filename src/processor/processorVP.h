

#ifndef SLAM_LYJ_PROCESSORVP_H
#define SLAM_LYJ_PROCESSORVP_H

#include "processor/processorAbr.h"

NSP_SLAM_LYJ_BEGIN

class processorVP : public processorAbr
{
private:
    /* data */
public:
    processorVP(/* args */);
    ~processorVP();


    //@override
    bool extractFeature(std::vector<Frame*> _frames){ return true;}
};






NSP_SLAM_LYJ_END


#endif //SLAM_LYJ_PROCESSORVP_H
