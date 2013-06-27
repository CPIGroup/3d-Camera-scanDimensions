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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <math.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// constants

#define LINEPOINTSTEP       0.0001
#define MAXDISTANCE         1.4f        // set workign distance

#define BACKGROUNDCAPTURE   100

//#define MAXITERATIONS       100
//#define DISTANCETHRESHOLD   0.007


//#define SEGMENTLARGEST    1.0       

#define CLUSTERTOLETANCE    0.01      
#define MINClUSTERSIZE      100
#define MAXCLUSTERSIZE      25000

//#define MINOBJECTSIZE     1000



struct BoxType {    // 3d box
    pcl::PointXYZ a[8]; // 8 corners (0..7)
};


// 3D Box scan class

class _3dBoxScan {
    
private:
    
    // background
    int background_capture;
    pcl::PointCloud<pcl::PointXYZ>::Ptr *background_cloud_ptr;

    // tools
    
    /**
     * Convert color from simple red, green, blue to rgb
     * @param int r - red (0..255)
     * @param int g - green (0..255)
     * @param int b - blue (0..255)
     * @return color
     */
    float inline rgb(int r, int g, int b) {
        //#if defined(_M_X64) || defined(_WIN64) || defined(__LP64__)
        //return static_cast<float> ((r << 32) | (g << 16) | b);
        //#else
        return static_cast<float> ((((int)r) << 16) | (((int)g) << 8) | ((int)b));
        //#endif
    }
    
    /**
     * Get distance between two points
     * @param pcl::PointXYZ p1 - first point
     * @param pcl::PointXYZ p2 - second point
     * @return float
     */
    inline float getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
        return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2));
    }    
    
    /**
     * Copy Cloud by pixels
     * @param const pcl::PointCloud<pcl::PointXYZ> cloud1 - from
     * @param pcl::PointCloud<pcl::PointXYZ>      cloud2 - to
     */
    void inline copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
        cloud2->points.resize(cloud1->size());
        for (size_t i = 0; i < cloud1->size(); i++) {
            cloud2->points[i].x = cloud1->points[i].x;
            cloud2->points[i].y = cloud1->points[i].y;
            cloud2->points[i].z = cloud1->points[i].z;
        }
        return;
    }
    
    /**
     * Copy Cloud by pixels
     * @param const pcl::PointCloud<pcl::PointXYZ> cloud1   - from
     * @param pcl::PointCloud<pcl::PointXYZ>      cloud2   - to
     * @param float          distance - distance filter
     */
    void inline copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance) {
        
        long f1 = 0, f2 = 0;
        cloud2->points.resize(0);
        for (size_t i = 0; i < cloud1->size(); i++) {
        if (getDistance(pcl::PointXYZ(0, 0, 0), cloud1->points[i]) < distance) {
            cloud2->points.push_back(cloud1->points[i]);
            f1++;
            } else {
            //std::cout <<  getDistance(pcl::PointXYZ(0, 0, 0), cloud1->points[i]) << std::endl;
            f2++;
            }
        }
                 
        std::cout << f1  << ", " << f2 << std::endl;
        return;
    }
    
    /**
     * Copy Cloud by pixels
     * @param pcl::PointCloud<pcl::PointXYZ> cloud1 - from
     * @param pcl::PointCloud<pcl::PointXYZ> cloud2 - to
     */
    void inline copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
        cloud2->points.resize(cloud1->size());
        for (size_t i = 0; i < cloud1->size(); i++) {
            cloud2->points[i].x = cloud1->points[i].x;
            cloud2->points[i].y = cloud1->points[i].y;
            cloud2->points[i].z = cloud1->points[i].z;
        }
        return;
    }
    
    /**
     * Copy Cloud by pixels
     * @param pcl::PointCloud<pcl::PointXYZ> cloud1   - from
     * @param pcl::PointCloud<pcl::PointXYZ> cloud2   - to
     * @param float     distance - distance filter
     */
    void inline copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance) {
        cloud2->points.resize(0);
        for (size_t i = 0; i < cloud1->size(); i++) {
        if (getDistance(pcl::PointXYZ(0, 0, 0), cloud1->points[i]) < distance) {
            cloud2->points.push_back(cloud1->points[i]);
            }
        }
        return;
    }    
    
    /**
     * Copy Cloud by pixels
     * @param pcl::PointCloud<pcl::PointXYZ>    cloud1 - from
     * @param pcl::PointCloud<pcl::PointXYZRGB> cloud2 - to
     * @param float        rgb    - color
     */
    void inline copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2, float rgb) {
        cloud2->points.resize(cloud1->size());
        for (size_t i = 0; i < cloud1->size(); i++) {
            cloud2->points[i].x = cloud1->points[i].x;
            cloud2->points[i].y = cloud1->points[i].y;
            cloud2->points[i].z = cloud1->points[i].z;
            cloud2->points[i].rgb = rgb;
        }
        return;
    }
    
    /**
     * Draw line in Cloud from p1 to p2
     * @param pcl::PointCloud<pcl::PointXYZ> cloud - destination cloud 
     * @param pcl::PointXYZ p1    - start point
     * @param pcl::PointXYZ p2    - end point
     */
    void drawLine(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ p1, pcl::PointXYZ p2) {
        float dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
        if (p2.x > p1.x) {
            for (float x = p1.x; x < p2.x; x += LINEPOINTSTEP) {
                cloud->push_back(pcl::PointXYZ(x, (p1.y + (x - p1.x) * dy / dx), (p1.z + (x - p1.x) * dz / dx)));
            }
        } else {
            for (float x = p1.x; x > p2.x; x -= LINEPOINTSTEP) {
                cloud->push_back(pcl::PointXYZ(x, (p1.y + (x - p1.x) * dy / dx), (p1.z + (x - p1.x) * dz / dx)));
            }
        }
        return;
    }    
    
    /**
     * Draw box in cloud
     * @param pcl::PointCloud<pcl::PointXYZ> cloud - destination cloud
     * @param BoxType   box   - box to draw
     */
    void drawBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, BoxType box) {
        for (int i = 0; i <= 7 - 1; i++) {
            for (int l = i + 1; l <= 7; l++) {
                if (i != l && box.a[i].x != 0 && box.a[l].x != 0 && box.a[i].y != 0 && box.a[l].y != 0 && box.a[i].z != 0 && box.a[l].z != 0) {
                    drawLine(cloud, box.a[i], box.a[l]);
                }
            }
        }
        return;
    }    
    
    // filters
    
    /**
     * Get difference between two point clouds
     * @param pcl::PointCloud<pcl::PointXYZ> cloud1 - first cloud
     * @param pcl::PointCloud<pcl::PointXYZ> cloud2 - second cloud
     * @param pcl::PointCloud<pcl::PointXYZ> diff   - different cloud
     */
    void diffCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr &diff) {
        
        boost::shared_ptr<std::vector<int> > newPointIdxVector(new std::vector<int>);
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.007f);

        octree.setInputCloud(cloud1);
        octree.addPointsFromInputCloud();

        octree.switchBuffers();

        octree.setInputCloud(cloud2);
        octree.addPointsFromInputCloud();

        octree.getPointIndicesFromNewVoxels(*newPointIdxVector, 1);

        diff.reset(new pcl::PointCloud<pcl::PointXYZ>);
        diff->points.reserve(newPointIdxVector->size());

        for (std::vector<int>::iterator it = newPointIdxVector->begin(); it != newPointIdxVector->end(); it++) {
            diff->points.push_back(cloud2->points[*it]);        
        }
        
        return;
    }
    
    
    
    
//    // background
//    pcl::PointCloud<pcl::PointXYZ>::Ptr *back_cloud_ptr;
//
//    // action flag
//    int action = 0;
//    
//    float lastd[3];
//
//    /**
//     * Remove points not in working area
//     * @param cloud
//     * @param closer_cloud
//     */
//    void getCloserPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
//            pcl::PointCloud<pcl::PointXYZ>::Ptr &closer_cloud) {
//
//        for (size_t i = 0; i < cloud->size(); i++) {
//            if (cloud->points[i].z < MAXDISTANCE) {
//                closer_cloud->points.push_back(cloud->points[i]);
//            }
//        }
//
//        return;
//    }
//
//
//    /**
//     * Find object
//     * @param cloud
//     * @param object_cloud
//     */
//    void getCloserObject(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
//            pcl::PointCloud<pcl::PointXYZ>::Ptr &object_cloud) {
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//
//        object_cloud->points.resize(cloud->size());
//
//        pcl::VoxelGrid<pcl::PointXYZ> vg;
//        vg.setInputCloud(cloud);
//        vg.setLeafSize(0.005f, 0.005f, 0.005f);
//        vg.filter(*cloud_filtered);
//
//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ> ());
//        seg.setOptimizeCoefficients(true);
//        seg.setModelType(pcl::SACMODEL_PLANE);
//        seg.setMethodType(pcl::SAC_RANSAC);
//        seg.setMaxIterations(MAXITERATIONS);
//        seg.setDistanceThreshold(DISTANCETHRESHOLD);
//
//        int i = 0, maxsize = 0, nr_points = (int) cloud_filtered->points.size();
//        while (cloud_filtered->points.size() > SEGMENTLARGEST * nr_points) {
//
//            seg.setInputCloud(cloud_filtered);
//            seg.segment(*inliers, *coefficients);
//            if (inliers->indices.size() == 0) {
//                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//                break;
//            }
//            pcl::ExtractIndices<pcl::PointXYZ> extract;
//            extract.setInputCloud(cloud_filtered);
//            extract.setIndices(inliers);
//            extract.setNegative(false);
//            extract.filter(*cloud_plane);
//            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
//
//            // Remove the planar inliers, extract the rest
//            extract.setNegative(true);
//            extract.filter(*cloud_f);
//            *cloud_filtered = *cloud_f;
//        }
//
//        // Creating the KdTree object for the search method of the extraction
//        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//        tree->setInputCloud(cloud_filtered);
//
//        std::vector<pcl::PointIndices> cluster_indices;
//        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//        ec.setClusterTolerance(CLUSTERTOLETANCE); // 1cm
//        ec.setMinClusterSize(MINClUSTERSIZE);
//        ec.setMaxClusterSize(MAXCLUSTERSIZE);
//        ec.setSearchMethod(tree);
//        ec.setInputCloud(cloud_filtered);
//        ec.extract(cluster_indices);
//
//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//
//            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
//                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
//
//            if (maxsize < cloud_cluster->points.size())
//                maxsize = cloud_cluster->points.size();
//        }
//
//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//
//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//
//            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
//                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
//
//            if (maxsize == cloud_cluster->points.size()) {
//                cloud_cluster->width = cloud_cluster->points.size();
//                cloud_cluster->height = 1;
//                cloud_cluster->is_dense = true;
//
//                for (size_t i = 0; i < cloud_cluster->size(); i++) {
//                    object_cloud->points[i].x = cloud_cluster->points[i].x;
//                    object_cloud->points[i].y = cloud_cluster->points[i].y;
//                    object_cloud->points[i].z = cloud_cluster->points[i].z;
//                }
//
//            }
//
//        }
//
//        return;
//    }
//

//

//
//    /**
//     * Draw box on cloud
//     * @param cloud
//     * @param box
//     */
//    void drawBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, xyzBox box) {
//        for (int i = 0; i <= 7 - 1; i++) {
//            for (int l = i + 1; l <= 7; l++) {
//                if (i != l && box.a[i].x != 0 && box.a[l].x != 0 && box.a[i].y != 0 && box.a[l].y != 0 && box.a[i].z != 0 && box.a[l].z != 0) {
//                    drawLine(cloud, box.a[i], box.a[l]);
//                }
//
//            }
//        }
//
//    }
//
//    /**
//     * Get box corners
//     * @param cloud
//     * @return 
//     */
//    xyzBox get_corners(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
//        float d = 0;
//
//        // get two far points
//        xyzBox box;
//        for (size_t i = 0; i < cloud->size() - 1; i++) {
//            for (size_t l = i + 1; l < cloud->size(); l++) {
//                if (getdistance(cloud->points[i], cloud->points[l]) > d) {
//                    d = getdistance(cloud->points[i], cloud->points[l]);
//                    box.a[0] = cloud->points[i];
//                    box.a[1] = cloud->points[l];
//                }
//            }
//        }
//
//        // get middle point
//        pcl::PointXYZ m;
//        m.x = (box.a[0].x + box.a[1].x) / 2;
//        m.y = (box.a[0].y + box.a[1].y) / 2;
//        m.z = (box.a[0].z + box.a[1].z) / 2;
//
//        // get other corners
//        for (int i = 2; i <= 7; i = i + 2) {
//            for (size_t t = 0; t < cloud->size(); t++) {
//                float dd = getdistance(cloud->points[t], m);
//                for (int l = 0; l < i; l++) {
//                    dd += getdistance(cloud->points[t], box.a[l]);
//                }
//                if (dd > d) {
//                    d = dd;
//                    box.a[i] = cloud->points[t];
//                    box.a[i + 1].x = m.x - box.a[i].x + m.x;
//                    box.a[i + 1].y = m.y - box.a[i].y + m.y;
//                    box.a[i + 1].z = m.z - box.a[i].z + m.z;
//                }
//            }
//        }
//
//        // debug
//        int c = 0;
//        for (int i = 0; i <= 7; i++) {
//            if (box.a[i].x != 0 && box.a[i].y != 0 && box.a[i].z != 0) {
//                c++;
//            }
//        }
//        //std::cout << "Found: " << c << " corners." << std::endl;
//
//        return box;
//    }
//
//    xyzBox buildBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_cloud) {
//
//        xyzBox box;
//        float tmp = 0;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//        for (size_t i = 0; i < raw_cloud->size(); i++) {
//            if (raw_cloud->points[i].x != 0 && raw_cloud->points[i].y != 0 && raw_cloud->points[i].z != 0) {
//                cloud->points.push_back(raw_cloud->points[i]);
//            }
//        }
//
//        if (cloud->points.size() > MINOBJECTSIZE) {
//            //std::cout << "Object size: " << cloud->points.size() << " data points." << std::endl;
//
//            box = get_corners(cloud);
//        }
//
//        drawBox(raw_cloud, box);
//        return box;
//    }
//    
//    void showBoxSize(xyzBox box) {
//        
//        float d[8];
//        
//        for (int i=1; i<8; i++) {
//            d[i-1] = getdistance(box.a[0], box.a[i]);
//            d[i-1] = round(d[i-1] * 39.37);
//            //std::cout << d[i-1] << std::endl;
//        }
//        
//        std::sort(d, d + 7, std::less<float>());
//        
//        if (lastd[0] == d[0] && lastd[1] == d[1] && lastd[2] == d[2] && d[0] != 0 && d[1] != 0 && d[2] != 0 ) {
//            std::cout << d[0] << " x " << d[1] << " x " << d[2] << std::endl;
//        }
//        
//        lastd[0] = d[0];
//        lastd[1] = d[1];
//        lastd[2] = d[2];
//    }
    
    
    void subtracting (pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out){

        pcl::PointCloud<pcl::PointXYZ>::Ptr workCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr workCloudDiff (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_planeCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr workCloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        
        do {
            
            // get plane
            std::vector<int> ransac_inliers;
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (in));

            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
            ransac.setDistanceThreshold (0.0045);
            ransac.computeModel();
            ransac.getInliers(ransac_inliers); 

            pcl::copyPointCloud<pcl::PointXYZ>(*in, ransac_inliers, *ransac_planeCloud);

            if (ransac_planeCloud->size() == 0) {
                break;
            }
            
            // get segment
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(ransac_planeCloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(CLUSTERTOLETANCE); // 1cm
            ec.setMinClusterSize(MINClUSTERSIZE);
            ec.setMaxClusterSize(MAXCLUSTERSIZE);
            ec.setSearchMethod(tree);
            ec.setInputCloud(ransac_planeCloud);
            ec.extract(cluster_indices);

            workCloud->resize(1);
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                    workCloud->points.push_back(ransac_planeCloud->points[*pit]);
            }        
        
            std::cout << "PointCloud representing the planar component: " << workCloud->size() << " data points." << std::endl;

            if (workCloud->size() < 1000) {
                break;
            }

            copyCloud(workCloud, planeCloudRGB, rgb(rand() % 150 + 100, rand() % 150 + 100, rand() % 150 + 100));
            *out += *planeCloudRGB;

            diffCloud(workCloud, in, workCloudDiff);
            
            copyCloud(workCloudDiff, in);
            //copyCloud(workCloudDiff, planeCloudRGB, rgb(rand() % 150 + 100, rand() % 150 + 100, rand() % 150 + 100));
            //*out += *planeCloudRGB;
                    
            //workCloudRGB;
//            //
//            boost::shared_ptr<std::vector<int> > newPointIdxVector(new std::vector<int>);
//            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.001f);
//
//            octree.setInputCloud(cloud_plane);
//            octree.addPointsFromInputCloud();
//
//            octree.switchBuffers();
//
//            octree.setInputCloud(in);
//            octree.addPointsFromInputCloud();
//
//            octree.getPointIndicesFromNewVoxels(*newPointIdxVector, 1);
//
//            workCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);      //!!!!
//            workCloud->points.reserve(newPointIdxVector->size());
//
//            for (std::vector<int>::iterator it = newPointIdxVector->begin(); it != newPointIdxVector->end(); it++) {
//                workCloud->points.push_back(in->points[*it]);        
//            }
//
//            std::cout << "2: " << workCloud->size() << std::endl;
//
//            copyCloud(workCloud, in);

            
        } while(true);

        std::cout << std::endl;
        
        //copyCloud(workCloud, workCloudRGB, rgb(150, 150, 150));
        //*out += *workCloudRGB;

        return;        
    }
    
//    void mls (pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointNormal> &out) {
//          // Create a KD-Tree
//          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//
//          // Init object (second point type is for the normals, even if unused)
//          pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//
//          mls.setComputeNormals (true);
//
//          // Set parameters
//          mls.setInputCloud (in);
//          mls.setPolynomialFit (true);
//          mls.setSearchMethod (tree);
//          mls.setSearchRadius (0.03);
//
//          // Reconstruct
//          mls.process (out);
//    }

    // filters

//    void project_inliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out) {
//        
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//        coefficients->values.resize(4);
//        coefficients->values[0] = 0;
//        coefficients->values[1] = 0;
//        coefficients->values[2] = 1.0;
//        coefficients->values[3] = 0;
//
//        // Create the filtering object
//        pcl::ProjectInliers<pcl::PointXYZ> proj;
//        proj.setModelType(pcl::SACMODEL_PLANE);
//        proj.setInputCloud(in);
//        proj.setModelCoefficients(coefficients);
//        proj.filter(*out);
//        
//        return;
//    }

//    void statistical_rutlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out) {
//        
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud (in);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (1.0);
//
//        //sor.setNegative (true);
//        sor.filter (*out);
//        
//        return;
//    }
      
//    void sample_consensus_model_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out) {
//        
//        std::vector<int> inliers;
//        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (in));
//        
//        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
//        ransac.setDistanceThreshold (.01);
//        ransac.computeModel();
//        ransac.getInliers(inliers);        
//        
//        pcl::copyPointCloud<pcl::PointXYZ>(*in, inliers, *out);
//        
//        return;        
//    }
    
//    void sample_consensus_model_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out) {
//        
//        std::vector<int> inliers;
//        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (in));
//        
//        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
//        ransac.setDistanceThreshold (1.0);
//        ransac.computeModel();
//        ransac.getInliers(inliers);        
//        
//         pcl::copyPointCloud<pcl::PointXYZ>(*in, inliers, *out);
//        
//        return;        
//    }
    
    
public:

    // Cloud viewer
    pcl::visualization::CloudViewer viewer;

    _3dBoxScan() : viewer("PCL OpenNI Viewer") {
        srand(time(NULL));
        background_capture = 0;
        viewer.registerKeyboardCallback(&_3dBoxScan::keyboardEventOccurred, *this, 0);
        background_cloud_ptr = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *) {

        if (event.getKeySym() == "b" && event.keyDown()) {
            background_capture = BACKGROUNDCAPTURE;
        }

    }

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &raw_cloud) {
        
        pcl::PointXYZRGB p;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud = *background_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_near (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        //copyCloud(raw_cloud, cloud_near);
        copyCloud(raw_cloud, cloud_near, MAXDISTANCE);

        if (background_capture > 0) {
            
            // capture background
            
            if (background_capture == BACKGROUNDCAPTURE) {
                copyCloud(cloud_near, background_cloud);
            } else {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);                
                diffCloud(background_cloud, cloud_near, cloud_diff);
                *background_cloud += *cloud_diff;
            }
            
            copyCloud(background_cloud, cloud_visual, rgb(150, 150, 255));
            
            background_capture--;
            
        } else if (background_cloud->size() > 0) {
            
            // working
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);   
            
            diffCloud(background_cloud, cloud_near, cloud_diff);            
            subtracting(cloud_diff, cloud_visual);
            
        } else {
            
            // splash
            
            copyCloud(cloud_near, cloud_visual, rgb(255, 255, 255));
            
        }

        if (!viewer.wasStopped()) {
            viewer.showCloud(cloud_visual);
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
int main() { _3dBoxScan v; v.run(); return 0; }
