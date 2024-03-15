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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/harris_3d.h>

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
    virtual void GroundScan(vector<Vector3>& points_convex);

	MyCamera();

};

}  // namespace vnoid
}  // namespace cnoid