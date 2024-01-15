#include "mycamera.h"

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    #ifdef _WIN64
    cnoid::vnoid::Debug::Out("viewerOneOff\n");
    #else
    printf("viewerOneOff\n");
    #endif
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    #ifdef _WIN64
    cnoid::vnoid::Debug::Out("viewerPsyco\n");
    #else
    printf("viewerPsyco\n");
    #endif
}

namespace cnoid {
namespace vnoid {

MyCamera::MyCamera() {
    timeStep = 1.0;
}

void MyCamera::Init(SimpleControllerIO* io) {
    // enable camera
    cameras << io->body()->devices();
    for (size_t i = 0; i < cameras.size(); ++i) {
        Device* camera = cameras[i];
        io->enableInput(camera);
        
        #ifdef _WIN64
        OPD("Device type: %s, ", camera->typeName());
        OPD("id: %s, ", camera->id());
        OPD("name: %s.\n", camera->name());
        #else
        printf("Device type: %s, ", camera->typeName());
        printf("id: %d, ", camera->id());
        printf("name: %s.\n", camera->name());
        #endif

    }

    timeCounter = 0.0;
    timeStep = io->timeStep();
}

void MyCamera::TerrainAnalysis() {
    // get cameras
    // when there are several cameras
    for (size_t i = 0; i < cameras.size(); i++) {
        RangeCamera* camera = cameras[i];
        // describe here
    }
    // only one camera
    RangeCamera* camera = cameras[0];

    // Get an image of the current scene
    const Image& RangeImage = camera->constImage();
    // Save an image of current scene
    RangeImage.save("pointcloud.png");
    #ifdef _WIN64
    OPD("save image.\n");
    #else
    printf("save image.\n");
    #endif
    
    // width and height of this image
    const int width = RangeImage.width();
    const int height = RangeImage.height();
    // get color data of this image
    const unsigned char* pixels = RangeImage.pixels();

    // point cloud variable declaration
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // initialize point cloud
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // Stores values (coordinates, color) for each point in a point cloud
    std::size_t i = 0;
    for (const auto& e : camera->constPoints()) {
        //pcl::PointXYZ& point = cloud->points[i];
        pcl::PointXYZRGB& point = cloud->points[i];

        // X, Y, Z
        point.x = e(0);
        point.y = e(1);
        point.z = e(2);

        // color(R, G, B)
        //point.r = pixels[3 * i + 0];
        //point.g = pixels[3 * i + 1];
        //point.b = pixels[3 * i + 2];

        point.r = 255;
        point.g = 255;
        point.b = 255;

        ++i;
    }

    // create the model coeeficients object
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);

    // create the inliers object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // create the SACSegmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // optional setting
    seg.setOptimizeCoefficients(true);

    // madatory settings
    seg.setModelType(pcl::SACMODEL_PLANE);  // detect plane
    seg.setMethodType(pcl::SAC_RANSAC); // use RANSAC algorithm
    seg.setDistanceThreshold(0.01); // how close a point must be to the ,model in order to be consider an inlier

    // input cloud to segmentation
    seg.setInputCloud(cloud);

    // plane segmentation
    seg.segment(*inliers, *coeffs);

    // coloring plane segments
    for (size_t i = 0; i < inliers->indices.size(); i++){
        cloud->points[inliers->indices[i]].r = 255;
        cloud->points[inliers->indices[i]].g = 0;
        cloud->points[inliers->indices[i]].b = 0;
    }

    // make a viewer of point cloud
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(cloud);

    
    // set the thread that called at once in visualization
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // set the thread running while visualization
    viewer.runOnVisualizationThread(viewerPsycho);

    // loop for viewing visualized point cloud
    /*while (!viewer.wasStopped()) {  
        viewer.wasStopped() = 0;
        // i dont know how should i stop this loop...
    }*/
    
    // save the point cloud data
    /*pcl::io::savePCDFileBinaryCompressed("pointcloud.pcd", *cloud);
    #ifdef _WIN64
    OPD("save pointcloud\n");
    #else
    printf("save pointcloud\n");
    #endif*/
    
}

}  // namespace vnoid
}  // namespace cnoid