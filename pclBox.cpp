#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>


// openNI/PCL get box dimension class

class _3dBoxScan {
private:
public:

    pcl::visualization::CloudViewer viewer;

    _3dBoxScan() : viewer("Box dimension Viewer") {
    }

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &raw_cloud) {

        if (!viewer.wasStopped()) {
            viewer.showCloud(raw_cloud);
        }

        return;
    }

    void run() {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&) > f =
                boost::bind(&_3dBoxScan::cloud_cb_, this, _1);

        interface->registerCallback(f);

        interface->start();

        while (!viewer.wasStopped()) {
            boost::this_thread::sleep(boost::posix_time::seconds(1));
        }

        interface->stop();
    }
};


// run

int main() {
    _3dBoxScan v;
    v.run();
    return 0;
}
