#pragma once

#include <cnoid/SimpleController>
#include <cnoid/EigenTypes>
#include <cnoid/RangeCamera>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

// define only when simulate on windows
//#define _WIN64
#ifdef _WIN64
#include "debug.h"
#endif

/*#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>*/

using namespace std;

namespace cnoid {
namespace vnoid {

class MyCamera : RangeCamera
{
public: 
    DeviceList<RangeCamera> cameras;
    double timeCounter;
    double timeStep;

public: 
	virtual void Init(SimpleControllerIO* io);
    virtual void GroundScan();

	MyCamera();

};

}  // namespace vnoid
}  // namespace cnoid