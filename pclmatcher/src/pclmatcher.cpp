#include "pclmatcher.h"

PCLMatcher::PCLMatcher(ros::NodeHandle& nh) : nh_(nh) {
    if_auto_align = true;
    field_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_sub = nh.subscribe("/livox/lidar", 10, &PCLMatcher::cloudCallback, this);
    field_pub = nh.advertise<sensor_msgs::PointCloud2>("field_cloud", 1);
    field_pub_thread = std::thread(&PCLMatcher::fieldCloudPublisher, this);
    initial_pose_sub = nh.subscribe("/initialpose", 1, &PCLMatcher::initialPoseCallback, this);
    clickpoint_sub = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &PCLMatcher::clickedPointCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    centroid_publisher = nh.advertise<sensor_msgs::PointCloud2>("centroid_points", 10);

    icp_transformation_pub = nh.advertise<geometry_msgs::TransformStamped>("icp_transform", 10);


    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义旋转角度和旋转轴
    level_transform = Eigen::Affine3f::Identity();
    float theta = - M_PI / 12; // 假设传感器倾斜的角度
    level_transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));  // Y轴旋转

    // 添加Z轴的平移
    level_transform.translation() << 0.0, 0.0, -4.1;  // Z轴向上平移4单位


    adjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 10);
    icpadjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("icpadjusted_cloud", 10);
    obstaclecloud_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud",10);

    cumulative_transform = Eigen::Matrix4f::Identity();
    icp_thread = std::thread(&PCLMatcher::icp_run,this);

    ground_readyICP.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_field.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

PCLMatcher::~PCLMatcher() {
    if (field_pub_thread.joinable() || icp_thread.joinable()) {
        field_pub_thread.join();
        icp_thread.join();
    }
}

void PCLMatcher::auto_align(){
    if (if_auto_align){
        initial_alignment_transform << 0.991867, 0.100943, 0.0775333, -2.58843,
                                    -0.110685, 0.984806, 0.133812, -5.05125,
                                    -0.0628479, -0.141306, 0.98796,0.386377,
                                    0,        0,       0,         1;
        ROS_INFO_STREAM("Initial alignment transform estimated:\n" << initial_alignment_transform);
        isINITFinish = true;
    }

}


void PCLMatcher::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if(if_auto_align==false){ 
        std::cout<<"x:"<<msg->point.x<<"y:"<<msg->point.y<<"z:"<<msg->point.z<<std::endl;
        Eigen::Vector3f point(msg->point.x, msg->point.y, msg->point.z);
        if(map_feature_points.size() < 4) {
            map_feature_points.push_back(point);
            ROS_INFO("Map feature point %ld added: [%f, %f, %f]", 
                    map_feature_points.size(), point.x(), point.y(), point.z());
        } else if(sensor_feature_points.size() < 4) {
            sensor_feature_points.push_back(point);
            ROS_INFO("Sensor feature point %ld added: [%f, %f, %f]", 
                    sensor_feature_points.size(), point.x(), point.y(), point.z());
        }

        if (map_feature_points.size() == 4 && sensor_feature_points.size() == 4) {
            computeInitialAlignment();
        }
    }
    else{
        computeInitialAlignment();
    }
}



void PCLMatcher::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{
    if(if_auto_align == false){
        Eigen::Vector3f point(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        if(map_feature_points.size() < 4) {
            map_feature_points.push_back(point);
            ROS_INFO("Map feature point %ld added: [%f, %f, %f]", 
                    map_feature_points.size(), point.x(), point.y(), point.z());
        } else if(sensor_feature_points.size() < 4) {
            sensor_feature_points.push_back(point);
            ROS_INFO("Sensor feature point %ld added: [%f, %f, %f]", 
                    sensor_feature_points.size(), point.x(), point.y(), point.z());
        }

        if (map_feature_points.size() == 4 && sensor_feature_points.size() == 4) {
            computeInitialAlignment();
        }
    }
    else{
        computeInitialAlignment();
    }
}


void PCLMatcher::computeInitialAlignment() {
    if(if_auto_align == false){
        pcl::PointCloud<pcl::PointXYZ> map_points;
        pcl::PointCloud<pcl::PointXYZ> sensor_points;

        for (size_t i = 0; i < map_feature_points.size(); ++i) {
            map_points.push_back(pcl::PointXYZ(map_feature_points[i].x(), map_feature_points[i].y(), map_feature_points[i].z()));
            sensor_points.push_back(pcl::PointXYZ(sensor_feature_points[i].x(), sensor_feature_points[i].y(), sensor_feature_points[i].z()));
        }

        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
        svd.estimateRigidTransformation(sensor_points, map_points, initial_alignment_transform);

        ROS_INFO_STREAM("Initial alignment transform estimated:\n" << initial_alignment_transform);
    
        isINITFinish = true;
    }

    else{
        auto_align();
    }
}


void PCLMatcher::loadPCD(const std::string& file_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *field_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", file_path.c_str());
        return;
    }
    ROS_INFO("Loaded %zu data points from %s", field_cloud->points.size(), file_path.c_str());
    // 应用体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(field_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*temp_cloud);

    field_cloud.swap(temp_cloud);
    
    pcl::toROSMsg(*field_cloud, ros_field_cloud);
}

void PCLMatcher::fieldCloudPublisher() {
    ros::Rate rate(10); // 设置发布频率，例如每秒10次
    while (ros::ok()) 
    {
        ros_field_cloud.header.frame_id = "livox_frame";
        ros_field_cloud.header.stamp = ros::Time::now();
        field_pub.publish(ros_field_cloud);
        rate.sleep();
    }
}

void PCLMatcher::icpFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcecloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& targetcloud, float transformationEpsilon, float distance, float euclideanFitnessEpsilon, int maximumIterations, bool useReciprocalCorrespondences, pcl::PointCloud<pcl::PointXYZ>::Ptr& aligncloud, Eigen::Matrix4f& final_transform)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(sourcecloud);
    icp.setInputTarget(targetcloud);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setMaxCorrespondenceDistance(distance);
    icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
    icp.setMaximumIterations(maximumIterations);
    icp.setUseReciprocalCorrespondences(useReciprocalCorrespondences);
    icp.align(*aligncloud);
    pcl::transformPointCloud(*sourcecloud, *sourcecloud, icp.getFinalTransformation());

    // 累加变换矩阵
    final_transform = icp.getFinalTransformation();

    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算并打印 ICP 算法耗时（微秒）
    auto duration_milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    std::cout << "ICP Duration in milliseconds: " << duration_milliseconds << " ms" << std::endl;
}

void PCLMatcher::trans_publisher(Eigen::Matrix4f final_transform){
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "base_frame";  // 源坐标系
    transform_msg.child_frame_id = "target_frame";  // 目标坐标系

    // 将Eigen矩阵转换为geometry_msgs::Transform
    transform_msg.transform.translation.x = final_transform(0, 3);
    transform_msg.transform.translation.y = final_transform(1, 3);
    transform_msg.transform.translation.z = final_transform(2, 3);
    Eigen::Quaternionf quat(final_transform.block<3, 3>(0, 0));
    transform_msg.transform.rotation.x = quat.x();
    transform_msg.transform.rotation.y = quat.y();
    transform_msg.transform.rotation.z = quat.z();
    transform_msg.transform.rotation.w = quat.w();
    ros::Rate rate(10);
    icp_transformation_pub.publish(transform_msg);
    //std::cout<<"Publish final transformation Matrix"<<std::endl;
    
}

void PCLMatcher::icp_run()
{
    ros::Rate rate(10); // 设置发布频率，例如每秒10次
    while(ros::ok())
    {
        if(isreadyICP)
        {
            // 累加变换矩阵
            icpFunction(readyICP_cloud, field_cloud, 1e-10, 1, 0.001, 50, true, icp_cloud, final_transform);
            initial_alignment_transform = final_transform * initial_alignment_transform;
            final_transform = initial_alignment_transform;
            trans_publisher(final_transform);
            std::cout << "icp_run Initial alignment transform matrix:" << std::endl;
            std::cout << initial_alignment_transform.matrix() << std::endl;
        
            isICPFinish = true;            
        }
        if(isICPFinish)
        {
            //isreadyICP = false;ros::Rate rate(10);

            sensor_msgs::PointCloud2 ros_adjusted_cloud;
            pcl::toROSMsg(*readyICP_cloud, ros_adjusted_cloud);
            ros_adjusted_cloud.header.frame_id = "livox_frame";
            ros_adjusted_cloud.header.stamp = ros::Time::now();
            icpadjusted_pub.publish(ros_adjusted_cloud);
            std::cout<<"icpadjusted_pub had pub!!!!!!!!!"<<std::endl;
        } 
        rate.sleep();
    }
}
void publishCentroids(const std::vector<Eigen::Vector3f>& centroids, const ros::Publisher& publisher)
{
    // 创建 PCL 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& centroid : centroids)
    {
        pcl::PointXYZ point;
        point.x = centroid.x();
        point.y = centroid.y();
        point.z = centroid.z();
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // 转换为 ROS 消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "rm_frame";  // 设定坐标系
    cloud_msg.header.stamp = ros::Time::now();
    std::cerr<<cloud_msg.header.stamp<<std::endl;

    // 发布点云
    publisher.publish(cloud_msg);
    std::cout << "Published centroids array." << std::endl;
}

void PCLMatcher::publishCentroidMarkers(const ros::Publisher& marker_pub, const std::vector<Eigen::Vector3f>& centroids)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "rm_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "centroids";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置点标记的尺寸
    points.scale.x = 0.2; // 点的宽度
    points.scale.y = 0.2; // 点的高度

    // 设置点的颜色为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0; // 不透明度

    // 循环添加点
    for (const Eigen::Vector3f& centroid : centroids) {
        geometry_msgs::Point p;
        p.x = centroid[0];
        p.y = centroid[1];
        p.z = centroid[2];
        points.points.push_back(p);
    }

    // 发布Marker
    marker_pub.publish(points);
}

void PCLMatcher::upsampleVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size, int points_per_voxel) 
{
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*downsampled);

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-leaf_size / 2, leaf_size / 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled(new pcl::PointCloud<pcl::PointXYZ>());

    // 对每个体素的中心点进行多次复制和随机偏移，但只有当点的 x 坐标大于 15m 时
    for (const auto& point : downsampled->points) {
        if (point.x > 15.0) { // 只有当 x 坐标大于 15 米时，才进行上采样
            for (int i = 0; i < points_per_voxel*2; ++i) {
                pcl::PointXYZ new_point;
                new_point.x = point.x + distribution(generator);
                new_point.y = point.y + distribution(generator);
                new_point.z = point.z + distribution(generator);
                upsampled->push_back(new_point);
            }
        } else {
            for (int i = 0; i < points_per_voxel / 2; ++i) {
                pcl::PointXYZ new_point;
                new_point.x = point.x + distribution(generator);
                new_point.y = point.y + distribution(generator);
                new_point.z = point.z + distribution(generator);
                upsampled->push_back(new_point);
            }
        }
    }

    cloud.swap(upsampled); // 更新原始点云
    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算并打印上采样耗时（毫秒）
    auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    std::cout << "upsampleVoxelGrid: " << duration_ms << " ms" << std::endl;
}


void PCLMatcher::mergePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& plb, pcl::PointCloud<pcl::PointXYZ>::Ptr& pl) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &cloud : plb) {
        *merged_cloud += *cloud; // 合并所有点云
    }
    // 更新current_source为合并后的点云
    pl = merged_cloud;
}


void PCLMatcher::voxel_grid_upsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr resampled_bg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(field_cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*resampled_bg);
    field_cloud.swap(resampled_bg);
}

void PCLMatcher::removeOverlappingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud)
{
    // 计算两个点云之间的差异
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::SegmentDifferences<pcl::PointXYZ> segment;
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //segment.setSearchMethod(tree);
    voxel_grid_upsample(field_cloud);
    segment.setInputCloud(live_cloud);
    segment.setTargetCloud(field_cloud);
    segment.setDistanceThreshold(0.1);  // 可根据实际情况调整

    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
    segment.segment(*dynamic_obstacles);

    // 在y轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(dynamic_obstacles);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-15, -0.6);
    pass_y.filter(*dynamic_obstacles);

    // 在x轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(dynamic_obstacles);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, 27.5);
    pass_x.filter(*dynamic_obstacles);

    // 在z轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(dynamic_obstacles);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, 1.4);
    pass_z.filter(*dynamic_obstacles);

    // statisticalOutlierRemoval(dynamic_obstacles, 10, 1);
    // meanshiftclustering(dynamic_obstacles, 12);
    //快速欧氏聚类
    // euclideancluster(dynamic_obstacles, 0.5, 10, 50000);
    radiusoutlierremoval(dynamic_obstacles, 0.5, 2, 0.1, 10);
    fastEuclideanCluster(dynamic_obstacles, 0.55, 50, 20, 50000, m_centroids);

    // 累加点云
    upsampleVoxelGrid(dynamic_obstacles, 0.01, 500); // 每个体素中生成10个点
    std::cout<<"length" << dynamic_obstacles->points.size()<<std::endl;
    
    sensor_msgs::PointCloud2 ros_dynamic_obstacles;
    pcl::toROSMsg(*dynamic_obstacles, ros_dynamic_obstacles);
    ros_dynamic_obstacles.header.frame_id = "livox_frame";
    ros_dynamic_obstacles.header.stamp = ros::Time::now();
    obstaclecloud_pub.publish(ros_dynamic_obstacles);
    publishCentroids(m_centroids,centroid_publisher);
    printCentroids(m_centroids);
    publishCentroidMarkers(marker_pub, m_centroids);
    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算并打印算法耗时（微秒）
    auto duration_milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    std::cout << "Removing Duration in milliseconds: " << duration_milliseconds << " ms" << std::endl;
}


void PCLMatcher::printCentroids(const std::vector<Eigen::Vector3f>& centroids) {
    if (centroids.empty()) {  // 检查向量是否为空
        std::cout << "No centroids available to display." << std::endl;
        return;  // 如果为空，打印消息并退出函数
    }
    int clusterIndex = 1;
    for (const Eigen::Vector3f& centroid : centroids) {
        std::cout << "Cluster " << clusterIndex << " centroid: X=" << centroid[0]
                  << ", Y=" << centroid[1] << ", Z=" << centroid[2] <<'\n\n\n'<< std::endl;
        clusterIndex++;
    }
}

void PCLMatcher::radiusoutlierremoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double near_radius, int near_neighbor, double far_radius, int far_neighbor)
{
    // 检查输入点云是否为空
    if (!cloud || cloud->empty()) {
        std::cerr << "radiusoutlierremoval Input cloud is empty or null!" << std::endl;
        return; // 如果点云为空，则提前返回
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);

     // 分割点云为近处和远处两部分
    pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_far_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : *cloud) {
        if (point.x >= 13.0) {
            far_cloud->push_back(point);
        } else {
            near_cloud->push_back(point);
        }
    }

    // if (near_cloud->empty()) 
    // {
    //     std::cout<<"near_cloud->empty()"<<std::endl;
    // }
    // if (far_cloud->empty()) 
    // {
    //     std::cout<<"near_cloud->empty()"<<std::endl;
    // }

    // 应用半径滤波 - 近处点云
    if (!near_cloud->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> near_ror;
        near_ror.setInputCloud(near_cloud);
        near_ror.setRadiusSearch(near_radius);  // 例如0.05m
        near_ror.setMinNeighborsInRadius(near_neighbor); // 例如2
        near_ror.filter(*filtered_near_cloud);
    }

    // 应用半径滤波 - 远处点云
    if (!far_cloud->empty()) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> far_ror;
        far_ror.setInputCloud(far_cloud);
        far_ror.setRadiusSearch(far_radius);  // 例如0.2m
        far_ror.setMinNeighborsInRadius(far_neighbor); // 例如5
        far_ror.filter(*filtered_far_cloud);
    }
    

    *cloud_radius = *filtered_near_cloud + *filtered_far_cloud;
    cloud.swap(cloud_radius);
}

void PCLMatcher::fastEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double radius, int searchmaxsize, int minclustersize, int maxclustersize, std::vector<Eigen::Vector3f>& centroids)
{
    pcl::console::TicToc time;
    time.tic();
	// ---------------------------改进快速欧式聚类-------------------------------
	std::vector<std::vector<int>>label;
	FastEuclideanCluster fec;
	fec.setInputCloud(cloud);      // 输入点云
	fec.setClusterTolerance(radius);  // 搜索半径
	fec.setSearchMaxSize(searchmaxsize);      // 邻域最大点数
	fec.setMinClusterSize(minclustersize);   // 聚类最小点数
	fec.setMaxClusterSize(maxclustersize);   // 聚类最大点数
	fec.extract(label);
	// ---------------------------聚类结果分类保存------------------------------
    centroids.clear(); // 清空centroids向量以防含有旧数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterResult(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < label.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterTemp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud, label[i], *clusterTemp);
		*clusterResult += *clusterTemp;

        // 计算每个聚类的质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusterTemp, centroid);

        // 只将X, Y, Z坐标存入centroids向量
        centroids.push_back(Eigen::Vector3f(centroid[0] * 1, 15 + centroid[1]*(1), centroid[2]*1));

	}
	std::cout << "快速欧式聚类用时：" << time.toc() << " ms" << std::endl;
	cloud.swap(clusterResult);
}

void PCLMatcher::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num, float thresh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);    //输入点云
    sor.setMeanK(num);            //设置领域点的个数
    sor.setStddevMulThresh(thresh); //设置离群点的阈值
    sor.filter(*cloud_filtered); //滤波后的结果
    cloud.swap(cloud_filtered);
}

void PCLMatcher::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *source_cloud);
    isLevel = true;
    if (!isLevel) {
        isLevel = true;
    } else {
        // 应用旋转到原始点云
        pcl::transformPointCloud(*source_cloud, *source_cloud, level_transform.inverse());
    }

    // 在x轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(source_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(5.0, 32.0);
    pass_x.filter(*source_cloud);

    // 在y轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(source_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-8.0, 8.0);
    pass_y.filter(*source_cloud);

    //在z轴上设置过滤范围
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(source_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-10, 3);
    pass_z.filter(*source_cloud);


    // 累加点云，仅对x坐标大于特定值（例如15米）的点进行
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& point : *source_cloud) {
        if (point.x > 15.0) { // 仅对远处的点进行累加
            for (int i = 0; i < 10; ++i) { // 已有一次，所以循环19次
                accumulated_cloud->push_back(point);
            }
        } else {
            accumulated_cloud->push_back(point); // 近处的点保持不变
        }
    }

    // 使用累加后的点云进行后续处理
    source_cloud.swap(accumulated_cloud);
  

    if(!isINITFinish)
    {
        // 发布调整后的点云
        sensor_msgs::PointCloud2 ros_adjusted_cloud;
        pcl::toROSMsg(*source_cloud, ros_adjusted_cloud);
        ros_adjusted_cloud.header.frame_id = "livox_frame";
        ros_adjusted_cloud.header.stamp = ros::Time::now();
        adjusted_pub.publish(ros_adjusted_cloud);
    }else{
        pcl::transformPointCloud(*source_cloud, *source_cloud, initial_alignment_transform);
        sensor_msgs::PointCloud2 ros_adjusted_cloud;
        pcl::toROSMsg(*source_cloud, ros_adjusted_cloud);
        ros_adjusted_cloud.header.frame_id = "livox_frame";
        ros_adjusted_cloud.header.stamp = ros::Time::now();
        adjusted_pub.publish(ros_adjusted_cloud);

        // std::cout << "cloudCallback Initial alignment transform matrix:" << std::endl;
        // std::cout << initial_alignment_transform.matrix() << std::endl;

        if (cloud_buffer.size() < 20) 
        {
            std::cout<<"cloud_buffer.size()"<<cloud_buffer.size()<<std::endl;
            cloud_buffer.push_back(source_cloud);
            if(cloud_buffer.size() == 19)
            {
                mergePointClouds(cloud_buffer, readyICP_cloud); // 合并点云
                isreadyICP = true;
            }
        }
    }

    if(isICPFinish)
    {
        //单帧
        removeOverlappingPoints(source_cloud, field_cloud);
    }
}

// 主函数在这里设置ROS环境和节点处理
int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_matcher_node");
    ros::NodeHandle nh;
    PCLMatcher matcher(nh);
    std::string pcd_file_path = "/home/nvidia/RadarWorkspace/data/rm2024v4.pcd"; // RMUC2025.pcd
    matcher.loadPCD(pcd_file_path);
    ros::spin();
    return 0;
}
