#ifndef SLAM_LYJ_PROCESSORABR_H
#define SLAM_LYJ_PROCESSORABR_H

#include "base/Frame.h"

NSP_SLAM_LYJ_BEGIN



class processorAbr
{
private:
    /* data */
public:
    processorAbr(/* args */){};
    ~processorAbr(){};


    virtual bool extractFeature(std::vector<Frame*> _frames)=0;
};



NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_PROCESSORABR_H