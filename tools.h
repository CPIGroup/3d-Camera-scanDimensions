/**
 * Copyright 2013 CPI Group, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TOOLS_H
#define	TOOLS_H

/**
 * Convert color from simple red, green, blue to rgb
 * @param int r - red (0..255)
 * @param int g - green (0..255)
 * @param int b - blue (0..255)
 * @return color
 */
float rgb(int r, int g, int b);

/**
 * Get distance between two points
 * @param pcl::PointXYZ p1 - first point
 * @param pcl::PointXYZ p2 - second point
 * @return float
 */
float getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2);

/**
 * Copy Cloud by pixels
 * @param const pcl::PointCloud<pcl::PointXYZ> cloud1 - from
 * @param pcl::PointCloud<pcl::PointXYZ>      cloud2 - to
 */
void copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);

/**
 * Copy Cloud by pixels
 * @param const pcl::PointCloud<pcl::PointXYZ> cloud1   - from
 * @param pcl::PointCloud<pcl::PointXYZ>      cloud2   - to
 * @param float          distance - distance filter
 */
void copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance);

/**
 * Copy Cloud by pixels
 * @param pcl::PointCloud<pcl::PointXYZ> cloud1 - from
 * @param pcl::PointCloud<pcl::PointXYZ> cloud2 - to
 */
void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);

/**
 * Copy Cloud by pixels
 * @param pcl::PointCloud<pcl::PointXYZ> cloud1   - from
 * @param pcl::PointCloud<pcl::PointXYZ> cloud2   - to
 * @param float     distance - distance filter
 */
void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float distance);

/**
 * Copy Cloud by pixels
 * @param pcl::PointCloud<pcl::PointXYZ>    cloud1 - from
 * @param pcl::PointCloud<pcl::PointXYZRGB> cloud2 - to
 * @param float        rgb    - color
 */
void copyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2, float rgb);

/**
 * Draw line in Cloud from p1 to p2
 * @param pcl::PointCloud<pcl::PointXYZ> cloud - destination cloud 
 * @param pcl::PointXYZ p1    - start point
 * @param pcl::PointXYZ p2    - end point
 */
void drawLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointXYZ p1, pcl::PointXYZ p2);

/**
 * Get difference between two point clouds
 * @param pcl::PointCloud<pcl::PointXYZ> cloud1 - first cloud
 * @param pcl::PointCloud<pcl::PointXYZ> cloud2 - second cloud
 * @param pcl::PointCloud<pcl::PointXYZ> diff   - different cloud
 */
void diffCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &diff);

#endif	/* TOOLS_H */

