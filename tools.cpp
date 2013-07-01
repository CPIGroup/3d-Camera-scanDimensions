#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>

#include "config.h"
#include "tools.h"

float rgb(int r, int g, int b) {
    uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
    return *reinterpret_cast<float*> (&rgb);
}

float getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2) + pow((p2.z - p1.z), 2));
}

void copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
    cloud2->points.resize(cloud1->size());
    for (size_t i = 0; i < cloud1->size(); i++) {
        cloud2->points[i].x = cloud1->points[i].x;
        cloud2->points[i].y = cloud1->points[i].y;
        cloud2->points[i].z = cloud1->points[i].z;
    }
    return;
}

void copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance) {
    cloud2->points.resize(0);
    for (size_t i = 0; i < cloud1->size(); i++) {
        if (getDistance(pcl::PointXYZ(0, 0, 0), cloud1->points[i]) < distance) {
            cloud2->points.push_back(cloud1->points[i]);
        }
    }
    return;
}

void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2) {
    cloud2->points.resize(cloud1->size());
    for (size_t i = 0; i < cloud1->size(); i++) {
        cloud2->points[i].x = cloud1->points[i].x;
        cloud2->points[i].y = cloud1->points[i].y;
        cloud2->points[i].z = cloud1->points[i].z;
    }
    return;
}

void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance) {
    cloud2->points.resize(0);
    for (size_t i = 0; i < cloud1->size(); i++) {
        if (getDistance(pcl::PointXYZ(0, 0, 0), cloud1->points[i]) < distance) {
            cloud2->points.push_back(cloud1->points[i]);
        }
    }
    return;
}

void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2, float rgb) {
    cloud2->points.resize(cloud1->size());

    for (size_t i = 0; i < cloud1->size(); i++) {
        cloud2->points[i].x = cloud1->points[i].x;
        cloud2->points[i].y = cloud1->points[i].y;
        cloud2->points[i].z = cloud1->points[i].z;
        cloud2->points[i].rgb = rgb;
    }

    return;
}

void drawLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointXYZ p1, pcl::PointXYZ p2) {

    float dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_tmp->points.resize(0);

    if (p2.x > p1.x) {
        for (float x = p1.x; x < p2.x; x += LINEPOINTSTEP) {
            cloud_tmp->push_back(pcl::PointXYZ(x, (p1.y + (x - p1.x) * dy / dx), (p1.z + (x - p1.x) * dz / dx)));
        }
    } else {
        for (float x = p1.x; x > p2.x; x -= LINEPOINTSTEP) {
            cloud_tmp->push_back(pcl::PointXYZ(x, (p1.y + (x - p1.x) * dy / dx), (p1.z + (x - p1.x) * dz / dx)));
        }
    }

    cloud->points.resize(cloud_tmp->size());
    for (size_t i = 0; i < cloud_tmp->size(); i++) {
        cloud->points[i].x = cloud_tmp->points[i].x;
        cloud->points[i].y = cloud_tmp->points[i].y;
        cloud->points[i].z = cloud_tmp->points[i].z;
        cloud->points[i].r = cloud->points[i].g = cloud->points[i].b = 0xff;
    }

    return;
}

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