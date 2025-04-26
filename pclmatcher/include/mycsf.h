#ifndef MYCSF_H
#define MYCSF_H

#include<ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "../csf/CSF.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <locale.h>
#include <time.h>
#include <cstdlib>
#include <cstring>
#include "../csf/Cfg.h"
#include <chrono>

//需要改成用户的具体路径
#define CONFIG_FILE_PATH "/home/hlc/code/ground_segment/src/csf/include/params.cfg"

class ClothSimulationFilter {
public:

    ClothSimulationFilter();
    ~ClothSimulationFilter();

    void filterGroundFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud);

private:
    Cfg cfg;
    CSF csf;
    
    void initializeConfig();

};


#endif