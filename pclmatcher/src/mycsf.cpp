#include"../include/mycsf.h"


ClothSimulationFilter::ClothSimulationFilter() {

    initializeConfig();
}

ClothSimulationFilter::~ClothSimulationFilter() {}

 void ClothSimulationFilter::initializeConfig() 
{
    std::string slop_smooth, class_threshold, cloth_resolution, iterations, rigidness, time_step, terr_pointClouds_filepath;

    cfg.readConfigFile(CONFIG_FILE_PATH, "slop_smooth", slop_smooth);
    bool ss = slop_smooth == "true" || slop_smooth == "True";

    cfg.readConfigFile(CONFIG_FILE_PATH, "class_threshold", class_threshold);
    cfg.readConfigFile(CONFIG_FILE_PATH, "cloth_resolution", cloth_resolution);
    cfg.readConfigFile(CONFIG_FILE_PATH, "iterations", iterations);
    cfg.readConfigFile(CONFIG_FILE_PATH, "rigidness", rigidness);
    cfg.readConfigFile(CONFIG_FILE_PATH, "time_step", time_step);
    cfg.readConfigFile(CONFIG_FILE_PATH, "terr_pointClouds_filepath", terr_pointClouds_filepath);

    // 使用读取的参数更新csf实例
    csf.params.bSloopSmooth = ss;
    csf.params.class_threshold = atof(class_threshold.c_str());
    csf.params.cloth_resolution = atof(cloth_resolution.c_str());
    csf.params.interations = atoi(iterations.c_str());
    csf.params.rigidness = atoi(rigidness.c_str());
    csf.params.time_step = atof(time_step.c_str());
}

void ClothSimulationFilter::filterGroundFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud)
{
    // 转换ROS点云到PCL点云可以在外部完成，这里直接处理PCL点云
    this->csf.setLivoxPointcloud(input_cloud);

    std::vector<int> groundIndexes, offGroundIndexes;
    this->csf.do_filtering(groundIndexes, offGroundIndexes, false);

    // 创建地面点云和障碍物点云的子集
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
    pcl::PointIndices::Ptr off_ground_indices(new pcl::PointIndices());

    // 填充地面和非地面点索引
    for (int index : groundIndexes) {
        ground_indices->indices.push_back(index);
    }
    for (int index : offGroundIndexes) {
        off_ground_indices->indices.push_back(index);
    }

    // 提取地面点云
    extract.setInputCloud(input_cloud);
    extract.setIndices(ground_indices);
    extract.filter(*ground_cloud);
}