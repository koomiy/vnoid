#include "mycamera.h"

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    printf("viewerOneOff\n");
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    printf("viewerPsyco\n");
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
        
        printf("Device type: %s, ", camera->typeName());
        printf("id: %d, ", camera->id());
        printf("name: %s.\n", camera->name());
    }

    timeCounter = 0.0;
    timeStep = io->timeStep();
}

void MyCamera::GroundScan(vector<Vector3>& points_convex) {
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
    printf("save image.\n");
    
    // width and height of this image
    const int width = RangeImage.width();
    const int height = RangeImage.height();

    // get color data of this image
    const unsigned char* pixels = RangeImage.pixels();

    // point cloud variable declaration
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                           cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                           cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

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

        //printf("point data : %lf, %lf, %lf\n", e(0), e(1), e(2));

        // color(R, G, B)
        point.r = pixels[3 * i + 0];
        point.g = pixels[3 * i + 1];
        point.b = pixels[3 * i + 2];

        //point.r = 255;
        //point.g = 255;
        //point.b = 255;

        ++i;
    }

    // build a filter to remove spurious NaNs and scene background  カメラの座標系がどう実装されているかを把握し、足元の階段近辺の点群のみ抽出できるように調整する
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.5, -1.0);
    pass.filter (*cloud_filtered);

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
    seg.setDistanceThreshold(0.005); // how close a point must be to the ,model in order to be consider an inlier

    // input cloud to segmentation
    seg.setInputCloud(cloud_filtered);
    
    // plane segmentation
    seg.segment(*inliers, *coeffs);

    // coloring plane segments
    for (size_t i = 0; i < inliers->indices.size(); i++){
        cloud_filtered->points[inliers->indices[i]].r = 255;
        cloud_filtered->points[inliers->indices[i]].g = 0;
        cloud_filtered->points[inliers->indices[i]].b = 0;
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
    // Extract the plane inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
  
    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_convex (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> cvexhull;
    cvexhull.setInputCloud (cloud_plane);
    //ccavehull.setAlpha (0.1);
    cvexhull.reconstruct (*cloud_convex);
    
    // output point on the plane's convex hull
    size_t num = cloud_convex->size();
    printf("size of cloud_convex: %ld\n", num);
    for (size_t i = 0; i < num; i++){
        pcl::PointXYZRGB& p = cloud_convex->points[i];
        points_convex.push_back(Vector3(p.x, p.y, p.z));
        //printf("%lf, %lf, %lf\n", cloud_convex->points[i].x, cloud_convex->points[i].y, cloud_convex->points[i].z);
    }

    //// get corner points of the landable area using Harris Corner Detector
    //pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZI> detector;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corner (new pcl::PointCloud<pcl::PointXYZI>);
    //detector.setNonMaxSupression (true);
    //detector.setInputCloud (cloud_convex);
    //detector.setRadius (1e-2);
    //detector.setRefine(true);
    //detector.setSearchSurface(cloud_plane);
    //detector.setThreshold (1e-6);
    //detector.compute (*cloud_corner);

    // make a viewer of point cloud
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(cloud_convex);
    
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
    printf("save pointcloud\n");*/
    
}

}  // namespace vnoid
}  // namespace cnoid