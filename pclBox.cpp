#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <iostream>

#include <pcl/filters/voxel_grid.h>                     // VoxelGrid
#include <pcl/segmentation/extract_clusters.h>          // EuclideanClusterExtraction
#include <pcl/sample_consensus/ransac.h>                // RandomSampleConsensus
#include <pcl/sample_consensus/sac_model_plane.h>       // SampleConsensusModelPlane

#include "config.h"
#include "tools.h"

#ifndef OUTPUTJSONCOUNT
#define OUTPUTJSONCOUNT 1
#endif /* OUTPUTJSONCOUNT */

#ifdef OUTPUTJSONFILE
#include <fstream>
char JSONBUFF[OUTPUTJSONCOUNT][1024];
#endif /* OUTPUTJSONFILE */

class box {
private:

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr *workCloudRGB_ptr;

    int p;
    pcl::PointXYZ m[3];
    pcl::PointCloud<pcl::PointXYZ>::Ptr *a_ptr[3];
    pcl::PointXYZ c;
    float side[3][2];

public:

    /**
     * constructor
     */
    box() {
        workCloudRGB_ptr = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        a_ptr[0] = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        a_ptr[1] = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        a_ptr[2] = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        p = 0;
        for (int i = 0; i < 3; i++) {
            for (int g = 0; g < 2; g++) {
                side[i][g] = 0.0f;
            }
        }
    }

    /**
     * Add plane to box class
     * @param PointCloud _cloud - plane
     */
    void addPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud) {

        if (p > 2) {
            return;
        }

        // create mesh
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(_cloud);
        vg.setLeafSize(LEAFSIZE, LEAFSIZE, LEAFSIZE);
        vg.filter(*cloud);


        pcl::PointCloud<pcl::PointXYZ>::Ptr a = *a_ptr[p];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr workCloudRGB = *workCloudRGB_ptr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

        double x = 0, y = 0, z = 0;

        // add and 
        cloudRGB->points.resize(cloud->size());

        for (size_t i = 0; i < cloud->size(); i++) {
            x += cloudRGB->points[i].x = cloud->points[i].x;
            y += cloudRGB->points[i].y = cloud->points[i].y;
            z += cloudRGB->points[i].z = cloud->points[i].z;
            switch (p) {
                case 0:
                    cloudRGB->points[i].rgb = RGBCOLOR_RED;
                    break;
                case 1:
                    cloudRGB->points[i].rgb = RGBCOLOR_GREEN;
                    break;
                case 2:
                    cloudRGB->points[i].rgb = RGBCOLOR_BLUE;
                    break;
            }
        }

        // get middle point
        m[p].x = x / cloud->size();
        m[p].y = y / cloud->size();
        m[p].z = z / cloud->size();

        getCorner(cloudRGB, a);

        *workCloudRGB += *cloudRGB;

        p++;
    }

    /**
     * Get corner and return array of corner
     * @param PointCloud cloud - plane
     * @param PointCloud a - corner
     */
    void getCorner(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &a) {

        a->points.resize(4);

        for (size_t e = 0; e < 4; e++) {
            float d = 0.0f;

            for (size_t i = 1; i < cloud->size(); i++) {
                float f = 0.0f;
                if (e == 0) {
                    if (cloud->points[i].x != 0 && cloud->points[i].y != 0 && cloud->points[i].z != 0) {
                        f = sqrt(pow((m[e].x - cloud->points[i].x), 2) + pow((m[e].y - cloud->points[i].y), 2) + pow((m[e].z - cloud->points[i].z), 2));
                    }
                } else {
                    for (size_t g = 0; g < e; g++) {
                        if (cloud->points[i].x != 0 && cloud->points[i].y != 0 && cloud->points[i].z != 0) {
                            f = f + sqrt(pow((a->points[g].x - cloud->points[i].x), 2) + pow((a->points[g].y - cloud->points[i].y), 2) + pow((a->points[g].z - cloud->points[i].z), 2));
                        }
                    }
                }
                if (f > d) {
                    d = f;
                    a->points[e].x = cloud->points[i].x;
                    a->points[e].y = cloud->points[i].y;
                    a->points[e].z = cloud->points[i].z;
                }
            }

        }
    }

    /**
     * get measurement of plane
     * @param PointCloud a - plane
     * @param float f1 - width or height
     * @param float f2 - width or height
     */
    void getSide(pcl::PointCloud<pcl::PointXYZ>::Ptr &a, float &f1, float &f2) {

        float s[3];

        for (int t = 0; t < 3; t++) {
            float dd = 0.0f;

            pcl::PointXYZ p1 = a->points[0];

            if (p1.x != 0 && p1.y != 0 && p1.z != 0) {

                for (size_t j = 1; j < 4; j++) {
                    pcl::PointXYZ p2 = a->points[j];

                    if (p2.x != 0 && p2.y != 0 && p2.z != 0) {

                        float d = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2)) * METERSTOINCH;

                        for (int tt = 0; tt < t; tt++) {
                            if (s[tt] == d) {
                                d = -1;
                            }
                        }

                        if (d > dd) {
                            s[t] = d;
                            dd = d;
                        }


                    }

                }
            }
        }

        f1 = s[1];
        f2 = s[2];
    }

    /**
     * Get could of box include technical lines and point
     * @todo normalize box
     * @param PointCloud cloud - output cloud
     */
    void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

        if (p < 2) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr workCloudRGB = *workCloudRGB_ptr;

        cloud->points.resize(0);

        for (size_t i = 0; i < workCloudRGB->size(); i++) {
            cloud->points.push_back(workCloudRGB->points[i]);
        }

        // normalize here
        for (size_t e = 0; e < 3; e++) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr a = *a_ptr[e];


            if (m[e].x != 0 && m[e].y != 0 && m[e].z != 0) {

                getSide(a, side[e][0], side[e][1]);

                for (size_t g = 0; g < 4; g++) {
                    if (a->points[g].x != 0 && a->points[g].y != 0 && a->points[g].z != 0) {
                        drawLine(cloud_line, m[e], a->points[g]);
                        *cloud += *cloud_line;
                    }
                }
            }

        }

    }

    /**
     * Output box dimensions 
     */
    void getBox() {

        float box[3];

        float s[3][2];
        for (int t = 0; t < 3; t++) {
            box[t] = 0.0f;
            s[t][0] = round(side[t][0]*100)/100;
            s[t][1] = round(side[t][1]*100)/100;
        }

        if (s[0][0] == 0 || s[0][1] == 0 || s[1][0] == 0 || s[1][1] == 0) {
            return;
        }

        if (s[2][0] != 0 && s[2][1] != 0) {
            // 3 side
            for (int a = 0; a < 2; a++) {
                float aa = s[0][a];
                float ab = (a == 0 ? s[0][1] : s[0][0]);

                for (int b = 0; b < 2; b++) {
                    float ba = s[1][b];
                    float bb = (b == 0 ? s[1][1] : s[1][0]);

                    if (aa > ba - SIDECOMPAREINCH && aa < ba + SIDECOMPAREINCH) {
                        for (int c = 0; c < 2; c++) {
                            float ca = s[2][c];
                            float cb = (c == 0 ? s[2][1] : s[2][0]);

                            if ((bb > ca - SIDECOMPAREINCH && bb < ca + SIDECOMPAREINCH) 
                                    && (cb > ab - SIDECOMPAREINCH && cb < ab + SIDECOMPAREINCH)) {
                                box[0] = (aa + ba) / 2;
                                box[1] = (bb + ca) / 2;
                                box[2] = (cb + ab) / 2;
                            }
                        }
                    }
                }
            }
        } else {
            // 2 side
            for (int a = 0; a < 2; a++) {
                float aa = s[0][a];
                float ab = (a == 0 ? s[0][1] : s[0][0]);

                for (int b = 0; b < 2; b++) {
                    float ba = s[1][b];
                    float bb = (b == 0 ? s[1][1] : s[1][0]);

                    if (aa > ba - SIDECOMPAREINCH && aa < ba + SIDECOMPAREINCH) {
                        box[0] = (aa + ba) / 2;
                        box[1] = ab;
                        box[2] = bb;
                    }
                }
            }

        }

#ifdef OUTPUTSCREEN
        
        std::cout << "BOX: " << round(box[0]) << "x" << round(box[1]) << "x" << round(box[2]) << " "
                << " RAWBOX: " << box[0] << "x" << box[1] << "x" << box[2] << " "
                << " Side1: " << s[0][0] << "x" << s[0][1] << ","
                << " Side2: " << s[1][0] << "x" << s[1][1] << ","
                << " Side3: " << s[2][0] << "x" << s[2][1] << std::endl;
        
#endif	/* OUTPUTSCREEN */

#ifdef OUTPUTJSONFILE

        struct timeval tv;
        gettimeofday(&tv, NULL);

        for (int i=0; i<OUTPUTJSONCOUNT-1; i++) {
            std::strcpy(JSONBUFF[i], JSONBUFF[i+1]);
        }
        
        sprintf (JSONBUFF[OUTPUTJSONCOUNT-1], 
            "{\"timestamp\":%Lf,\"box\":[%d,%d,%d],\"str\":\"%dx%dx%d\",\"raw\":[%.2f,%.2f,%.2f],\"sides\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]]}",
            ((long double)tv.tv_sec + ( (long double)tv.tv_usec / 1000000 )),
            (int)round(box[0]), (int)round(box[1]), (int)round(box[2]),
            (int)round(box[0]), (int)round(box[1]), (int)round(box[2]),
            box[0], box[1], box[2],
            s[0][0], s[0][1],
            s[1][0], s[1][1],
            s[2][0], s[2][1]);
        
        std::ofstream jout(OUTPUTJSONFILE);
        
        for (int i=0; i<OUTPUTJSONCOUNT; i++) {
            jout << JSONBUFF[i] << std::endl;
        }
        
        jout.close();
        
#endif	/* OUTPUTJSONFILE */

    }

};


// 3D Box scan class

class _3dBoxScan {
private:

    // background
    int background_capture;
    pcl::PointCloud<pcl::PointXYZ>::Ptr *background_cloud_ptr;

    /**
     * Get plane from PointCloud and save into box class
     * @param PointCloud in
     * @param box b
     */
    void subtracting(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, box &b) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr workCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr workCloudDiff(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_planeCloud(new pcl::PointCloud<pcl::PointXYZ>);

        int c = 0;

        do {

            // get plane
            std::vector<int> ransac_inliers;
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (in));

            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
            ransac.setDistanceThreshold(0.0045);
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

            if (workCloud->size() < 1000) {
                break;
            }

            b.addPlane(workCloud);

            diffCloud(workCloud, in, workCloudDiff);
            copyCloud(workCloudDiff, in);

            c++;

        } while (c < 3);

        return;
    }


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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);

        //copyCloud(raw_cloud, cloud_near);
        copyCloud(raw_cloud, cloud_near, MAXDISTANCE);

        if (background_capture > 0) {

            // capture background

            if (background_capture == BACKGROUNDCAPTURE) {
                copyCloud(cloud_near, background_cloud);
            } else {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZ>);
                diffCloud(background_cloud, cloud_near, cloud_diff);
                *background_cloud += *cloud_diff;
            }

            copyCloud(background_cloud, cloud_visual, RGBCOLOR_LIGHTGRAY);

            background_capture--;

        } else if (background_cloud->size() > 0) {

            // working

            box b;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZ>);

            diffCloud(background_cloud, cloud_near, cloud_diff);
            subtracting(cloud_diff, b);

            b.getCloud(cloud_visual);

            b.getBox();

        } else {

            // splash

            copyCloud(cloud_near, cloud_visual, RGBCOLOR_GRAY);

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

int main() {
    _3dBoxScan v;
    v.run();
    return 0;
}
