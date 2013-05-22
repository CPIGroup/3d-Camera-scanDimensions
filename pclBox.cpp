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


// constants

#define MAXDISTANCE       1.2f      // set workign distance

#define MAXITERATIONS     100
#define DISTANCETHRESHOLD 0.007
#define SEGMENTLARGEST    1.0       
#define CLUSTERTOLETANCE  0.01      
#define MINClUSTERSIZE    100
#define MAXCLUSTERSIZE    25000
#define MINOBJECTSIZE     1000


// 3d box

struct xyzBox {
    pcl::PointXYZ a[8]; // 8 corners
};


// 3D Box scan class

class _3dBoxScan {
private:

    // background
    pcl::PointCloud<pcl::PointXYZ>::Ptr *back_cloud_ptr;

    // action flag
    int action = 0;

    /**
     * Remove points not in working area
     * @param cloud
     * @param closer_cloud
     */
    void getCloserPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &closer_cloud) {

        for (size_t i = 0; i < cloud->size(); i++) {
            if (cloud->points[i].z < MAXDISTANCE) {
                closer_cloud->points.push_back(cloud->points[i]);
            }
        }

        return;
    }

    /**
     * Convert point cloud to RBG
     * @param cloud
     * @param rgb_cloud
     * @param rgb
     */
    void convertToRGB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgb_cloud, float rgb) {

        rgb_cloud->points.resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); i++) {
            rgb_cloud->points[i].x = cloud->points[i].x;
            rgb_cloud->points[i].y = cloud->points[i].y;
            rgb_cloud->points[i].z = cloud->points[i].z;
            rgb_cloud->points[i].rgb = rgb;
        }

        return;
    }

    /**
     * Find object
     * @param cloud
     * @param object_cloud
     */
    void getCloserObject(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &object_cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        object_cloud->points.resize(cloud->size());

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.005f, 0.005f, 0.005f);
        vg.filter(*cloud_filtered);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(MAXITERATIONS);
        seg.setDistanceThreshold(DISTANCETHRESHOLD);

        int i = 0, maxsize = 0, nr_points = (int) cloud_filtered->points.size();
        while (cloud_filtered->points.size() > SEGMENTLARGEST * nr_points) {

            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(CLUSTERTOLETANCE); // 1cm
        ec.setMinClusterSize(MINClUSTERSIZE);
        ec.setMaxClusterSize(MAXCLUSTERSIZE);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

            if (maxsize < cloud_cluster->points.size())
                maxsize = cloud_cluster->points.size();
        }

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

            if (maxsize == cloud_cluster->points.size()) {
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                for (size_t i = 0; i < cloud_cluster->size(); i++) {
                    object_cloud->points[i].x = cloud_cluster->points[i].x;
                    object_cloud->points[i].y = cloud_cluster->points[i].y;
                    object_cloud->points[i].z = cloud_cluster->points[i].z;
                }

            }

        }

        return;
    }

    /**
     * Get distance between two points
     * @param p1
     * @param p2
     * @return float
     */
    inline float getdistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
        return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2));
    }

    /**
     * Draw line from p1 to p2 on cloud
     * @param cloud
     * @param p1
     * @param p2
     */
    void drawLine(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ p1, pcl::PointXYZ p2) {

        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float dz = p2.z - p1.z;

        if (p2.x > p1.x) {
            for (float x = p1.x; x < p2.x; x += 0.0001) {
                float y = p1.y + (x - p1.x) * dy / dx;
                float z = p1.z + (x - p1.x) * dz / dx;
                cloud->push_back(pcl::PointXYZ(x, y, z));
            }
        } else {
            for (float x = p1.x; x > p2.x; x -= 0.0001) {
                float y = p1.y + (x - p1.x) * dy / dx;
                float z = p1.z + (x - p1.x) * dz / dx;
                cloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }

    }

    /**
     * Draw box on cloud
     * @param cloud
     * @param box
     */
    void drawBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, xyzBox box) {
        for (int i = 0; i <= 7 - 1; i++) {
            for (int l = i + 1; l <= 7; l++) {
                if (i != l && box.a[i].x != 0 && box.a[l].x != 0 && box.a[i].y != 0 && box.a[l].y != 0 && box.a[i].z != 0 && box.a[l].z != 0) {
                    drawLine(cloud, box.a[i], box.a[l]);
                }

            }
        }

    }

    /**
     * Get box corners
     * @param cloud
     * @return 
     */
    xyzBox get_corners(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        float d = 0;

        // get two far points
        xyzBox box;
        for (size_t i = 0; i < cloud->size() - 1; i++) {
            for (size_t l = i + 1; l < cloud->size(); l++) {
                if (getdistance(cloud->points[i], cloud->points[l]) > d) {
                    d = getdistance(cloud->points[i], cloud->points[l]);
                    box.a[0] = cloud->points[i];
                    box.a[1] = cloud->points[l];
                }
            }
        }

        // get middle point
        pcl::PointXYZ m;
        m.x = (box.a[0].x + box.a[1].x) / 2;
        m.y = (box.a[0].y + box.a[1].y) / 2;
        m.z = (box.a[0].z + box.a[1].z) / 2;

        // get other corners
        for (int i = 2; i <= 7; i = i + 2) {
            for (size_t t = 0; t < cloud->size(); t++) {
                float dd = getdistance(cloud->points[t], m);
                for (int l = 0; l < i; l++) {
                    dd += getdistance(cloud->points[t], box.a[l]);
                }
                if (dd > d) {
                    d = dd;
                    box.a[i] = cloud->points[t];
                    box.a[i + 1].x = m.x - box.a[i].x + m.x;
                    box.a[i + 1].y = m.y - box.a[i].y + m.y;
                    box.a[i + 1].z = m.z - box.a[i].z + m.z;
                }
            }
        }

        // debug
        int c = 0;
        for (int i = 0; i <= 7; i++) {
            if (box.a[i].x != 0 && box.a[i].y != 0 && box.a[i].z != 0) {
                c++;
            }
        }
        std::cout << "Found: " << c << " corners." << std::endl;

        return box;
    }

    xyzBox buildBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_cloud) {

        xyzBox box;
        float tmp = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < raw_cloud->size(); i++) {
            if (raw_cloud->points[i].x != 0 && raw_cloud->points[i].y != 0 && raw_cloud->points[i].z != 0) {
                cloud->points.push_back(raw_cloud->points[i]);
            }
        }

        if (cloud->points.size() > MINOBJECTSIZE) {
            std::cout << "Object size: " << cloud->points.size() << " data points." << std::endl;

            box = get_corners(cloud);
        }

        drawBox(raw_cloud, box);
        return box;
    }

public:

    // Cloud viewer
    pcl::visualization::CloudViewer viewer;

    _3dBoxScan() : viewer("PCL OpenNI Viewer") {
        srand(time(NULL));
        viewer.registerKeyboardCallback(&_3dBoxScan::keyboardEventOccurred, *this, 0);
        back_cloud_ptr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *) {

        if (event.getKeySym() == "a" && event.keyDown()) {
            action++;
            if (action > 2)
                action = 0;
        }

    }

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &raw_cloud) {

        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.007f);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr back_cloud = *back_cloud_ptr;

        boost::shared_ptr<std::vector<int> > newPointIdxVector(new std::vector<int>);

        getCloserPoints(raw_cloud, cloud);

        if (action != 0) {

            octree.setInputCloud(back_cloud);
            octree.addPointsFromInputCloud();

            octree.switchBuffers();

            octree.setInputCloud(cloud);
            octree.addPointsFromInputCloud();

            octree.getPointIndicesFromNewVoxels(*newPointIdxVector, 1);

            filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            filtered_cloud->points.reserve(newPointIdxVector->size());

            for (std::vector<int>::iterator it = newPointIdxVector->begin(); it != newPointIdxVector->end(); it++)
                filtered_cloud->points.push_back(cloud->points[*it]);
        }

        if (!viewer.wasStopped()) {


            switch (action) {

                case 0: // step 1 - show cloud

                    back_cloud->points.resize(cloud->size());
                    viewer.showCloud(cloud);
                    break;

                case 1: // step 2 - generate background cloud

                    for (size_t i = 0; i < filtered_cloud->size(); i++)
                        back_cloud->points.push_back(filtered_cloud->points[i]);

                    viewer.showCloud(filtered_cloud);
                    break;

                case 2: // step 3 - working

                    xyzBox box;

                    convertToRGB(filtered_cloud, rgb_cloud, static_cast<float> ((70 << 16) | (70 << 8) | 70));

                    getCloserObject(filtered_cloud, object_cloud);
                    box = buildBox(object_cloud);

                    convertToRGB(object_cloud, rgb_object_cloud, static_cast<float> ((0 << 16) | (255 << 8) | 255));
                    *rgb_cloud += *rgb_object_cloud;


                    viewer.showCloud(rgb_cloud);
                    break;
            }

        }

        return;
    }

    // run
    
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

// main

int main() {
    _3dBoxScan v;
    v.run();
    return 0;
}
