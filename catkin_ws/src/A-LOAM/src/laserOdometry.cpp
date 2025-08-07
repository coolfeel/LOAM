

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


#include <eigen3/Eigen/Dense>

#include <mutex>
#include <queue>



#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"


using namespace std;


#define DISTORTION 0


int corner_correspondence = 0;
int plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;


int skipFrameNum = 5;
bool systemInited = false;


double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;


pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());



//pcl格式, from ros to pcl
// 线点
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
// 面点
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

// 整体点云
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());



pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());


int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;


// Lidar Odometry线程估计的frame在world坐标系的位姿P, Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);



// 点云特征匹配时的优化变量
// q_curr_last(x, y, z, w)
double para_q[4] = {0, 0, 0, 1};
// t_curr_last
double para_t[3] = {0, 0, 0};


// 2个分别是优化变量para_q和para_t的映射：表示的是两个局部坐标系下的位姿P之间的增量，例如△P = P0.inverse() * P1, 待求解的新帧的
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);


// 传感器消息的队列，存储指针
// 线点
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;

// 面点
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;

// 总云
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

// 互斥锁
std::mutex mBuf;


// 转到帧的开始 
// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)
{
    // interpolation ratio
    double s;

    if (DISTORTION)
    {
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    }
    else
    {
        s = 1.0;
    }

    // s = 1;

    // 单位四元数的球面线性插值，last_curr当前到上一时刻，point_last任意点i到上一时刻
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    // 平移的线性插值
    Eigen::Vector3d t_point_last = s * t_last_curr;

    //当前点的坐标
    Eigen::Vector3d point(pi->x, pi->y, pi->z);

    // 利用该点和上一时刻帧的插值后的变换，将当前点变换到上一时刻
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    // 变换后的结果
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();

    po->intensity = pi->intensity;


}


// transform all lidar points to the start of the next frame
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}




// 处理锐角点云消息回调函数
// 线点
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    // 上锁以确保对缓冲区的安全访问
    mBuf.lock();
    // 将接收到的线特征点云消息存储到相应的缓冲区中
    cornerSharpBuf.push(cornerPointsSharp2);
    // 解锁
    mBuf.unlock();
}


void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}



// 面点
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}




// 总点，all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}




// odo main
int main(int argc, char **argv)
{
    printf("coolodometry\n");


    

    // 节点
    ros::init(argc, argv, "laserOdometry");
    // 句柄
    ros::NodeHandle nh;

    // 从参数服务器中读取参数，每处理多少帧数据执行一次建图
    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);
    
    // skipFrameNum: 在代码中定义的变量，存储从参数服务器获取到的值。找到了参数，存到skipFrameNum 中
    // 在参数服务器中找不到参数，赋默认值2

    // 找到值1
    // printf("%d\n", skipFrameNum); 
    
    // 10
    printf("Mapping %d Hz \n", 10 / skipFrameNum);


    // 订阅从scanReg.cpp中发布的点云和特征点数据

    // 订阅锐利角点的点云，并执行回调函数
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    // 订阅较不锐利角点的点云
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    // 订阅平坦面点的点云
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    // 订阅较不平坦面点的点云
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    // 订阅完整分辨率的激光点云,总点
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);



    // 创建发布者对象

    // 线点
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    // 平面点
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    // 总点
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);


    // 里程计
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    // 路径
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;



    // 帧计数器
    int frameCount = 0;

    // 循环频率
    ros::Rate rate(100);

    // ROS主循环
    while (ros::ok())
    {
        
        // 处理回调函数, 所有当前等待的消息回调
        ros::spinOnce();

        
        // 检查所有5个点云缓冲区队列都不空
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() && !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty())
        {
            // 处理队列第一个元素
            // 获取每个点云消息的时间戳，点云格式ros的PointCloud2
            // 线点
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            // 1422133388.503344
            // printf("%f\n", timeCornerPointsSharp);

            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            
            // 面点
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();

            // 总点
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

            
            // 检查所有点云数据的时间戳是否一致, 一个时刻sweep，
            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }

            // 上锁以同步访问
            mBuf.lock();

            // 线点
            // 清空上次存储的点云指针，并从ROS消息转换为PCL点云，作为当前
            cornerPointsSharp->clear();
            // 线点
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            // 剔除队列第一个
            cornerSharpBuf.pop();

            
            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();


            // 面点
            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();



            // 总点，清空上次存储的
            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();

            // 解锁
            mBuf.unlock();

            


            TicToc t_whole;

            //作用是跳过第一帧
            // initializing
            // 第一帧不进行匹配，仅仅将 cornerPointsLessSharp 保存至 laserCloudCornerLast
            // 将 surfPointsLessFlat 保存至 laserCloudSurfLast，为下次匹配提供target
            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else
            {
                
                // 线，面特征点的数量
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                // 192
                // printf("%d\n", cornerPointsSharp->points.size());

                // 1675
                // printf("%d\n", cornerPointsLessSharp->points.size());

                // 364
                // printf("%d\n", surfPointsFlat->points.size());

                // 5936
                // printf("%d\n", surfPointsLessFlat->points.size());

                // 线+面特征点大概8000
                // 所有点云14221 = 16 * 900
                // printf("%d\n", laserCloudFullRes->points.size());

                TicToc t_opt;

                // 进行两次迭代
                for (size_t opti_counter = 0; opti_counter < 2; opti_counter++)
                {
                    
                    corner_correspondence = 0; 
                    plane_correspondence = 0;

                    // 定义HUBER核函数减小外点权重，阈值0.1
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    
                    // 定义处理四元数的流形接口
                    ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();

                    // 定义ceres问题的配置和问题
                    ceres::Problem::Options problem_options;
                    ceres::Problem problem(problem_options);

                    // 定义参数
                    // 待优化变量四元数和平移加到问题中，使用自定义四元数处理。double para_q[4] = {0, 0, 0, 1},para_t[3] = {0, 0, 0};
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    
                    problem.AddParameterBlock(para_t, 3);

                    // 变换到帧开始的点
                    pcl::PointXYZI pointSel;
                    // 索引
                    std::vector<int> pointSearchInd;
                    // 距离
                    std::vector<float> pointSearchSqDis;


                    TicToc t_data;

                    //寻找角点线的约束，遍历每一个线点，并计算残差代价
                    // 基于最近邻原理建立corner特征点之间关联，find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; i++)
                    {

                        // 当前点变换到帧开始时刻k+1
                        // 将当前帧的corner_sharp特征点O_cur，从当前帧的Lidar坐标系下变换到上一帧的Lidar坐标系下（记为点O，注意与前面的点O_cur不同），
                        // 以利于寻找corner特征点的correspondence
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel); 

                        // 变换到帧的开始，之后，最近的K个搜索（共同坐标下点了， 共用索引）
                        // kdtree中的点云是上一帧的corner_less_sharp，所以这是在上一帧的corner_less_sharp中寻找当前帧corner_sharp特征点O的最近邻点（记为A）
                        // 调用KDTREE接口寻找这帧角点转换到上一帧后距离最近的1个角点，论文中的j
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        // j
                        int closestPointInd = -1;
                        // l
                        int minPointInd2 = -1;

                        // 如果最近邻的corner特征点之间距离平方小于阈值， 最近点j的距离合理
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // 最近点j的索引
                            closestPointInd = pointSearchInd[0];
                            // 最近点j所在scan的ID
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            // 另一个附近scan上，距离i最近的点l，记录当前最小距离
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;

                            // 对于找到的每个近邻角点，搜索其在扫描线上相邻的其他角点，以形成角点对。搜索方向包括扫描线的增加方向和减少方向，以确保找到合适的匹配点
                            // laserCloudCornerLast 来自上一帧的corner_less_sharp特征点,由于提取特征时是按照scan的顺序提取的，所以laserCloudCornerLast中的点也是按照scanID递增的顺序存放的
                            // 寻找附近线上的角点
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); j++)
                            {
                                // 要在不同的scan上
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                {
                                    continue;
                                }

                                // 线离得太远的也pass掉
                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                {
                                    break;
                                }

                                // 和当前角点的距离
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                //遍历完记录寻找到的最近的，第二个最近邻点有效,，更新论文中点l
                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }

                            }

                            //寻找附近线上的角点，往下找，可能在scanID减少方向，计算同理
                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; j--)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }


                        }

                        //如果找到论文中有效点l
                        if (minPointInd2 >= 0)
                        {
                            //取出当前角点i (还没有变换到帧的开始时刻) 和上一帧的两个角点j,l (用变换后的点pointSel找到的索引，在开始时刻)
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);
                            
                            double s;
                            // 根据是否存在畸变调整点的时间戳，用于运动补偿
                            if (DISTORTION)
                            {
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            }
                            else
                            {
                                s = 1.0;
                            }
                                

                            
                            //使用ceres构建约束， 残差代价，调用lidarFactor.hpp里的类计算残差, curr点是还没有变换的
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            
                            // 将代价函数添加到优化问题中
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            
                            // 线关联数量
                            corner_correspondence++;

                        }

                    }


                    // 面关联
                    // find correspondence for plane features，论文中的j,l,m
                    for (int i = 0; i < surfPointsFlatNum; i++)
                    {
                        // 将平面点变换到扫描开始时的位置
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);

                        // 在上一帧点云中查找最近邻点j
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        // j, l, m
                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        // 距离有效的j
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // j的索引
                            closestPointInd = pointSearchInd[0];

                            //  // 获取最近点j的扫描scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // 向上搜索相邻扫描线上的点
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); j++)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 找到的第2个最近邻点有效，更新点l，注意如果scanID准确的话，一般点j和点l的scanID相同
                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // 找到的第3个最近邻点有效，更新点m，注意如果scanID准确的话，一般点j和点l的scanID相同,且与点m的scanID不同
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // 同理，向下搜索相邻扫描线上的点
                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; j--)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 根据点的扫描线ID和距离选择合适的点
                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // 确保找到了有效的近点j, l, m
                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {
                                // 将当前点和找到的三个最近点转换为Eigen的Vector3d格式
                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                // 根据是否存在畸变调整点的时间戳
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                
                                // 构建ceres优化问题
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                
                                // 将代价函数添加到优化问题中
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                
                                // 面关联数量
                                plane_correspondence++;
                            }
                        }
                    }

                    // 线和面，关联点的总数统计
                    printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());


                    // 关联点太少
                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }


                    TicToc t_solver;

                    // 定义求解器
                    // 选择哪种求解器即可,例如使用DENSE_QR
                    ceres::Solver::Options options;

                    // DENSE_QR: 用于小规模最小二乘问题的求解
                    options.linear_solver_type = ceres::DENSE_QR;

                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;

                    // 优化信息
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);

                    printf("solver time %f ms \n", t_solver.toc());

                   
                    
                }

                printf("optimization twice time %f \n", t_opt.toc());

                // 将帧间变换转成位姿
                // Transformation from current frame to world frame
                // 当前帧到上一帧的变换 结合 上一帧到世界坐标的，就是当前帧到世界坐标的里程计位姿
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;


            }


            // 以下处理需要注意是否为第一帧
            TicToc t_pub;

            // 当前帧的信息
            printf("----------------pub odo----------------");

            // nav_msgs导航消息中，当前时间的里程计信息
            nav_msgs::Odometry laserOdometry;

            laserOdometry.header.frame_id = "camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);

            // q， 当前局部坐标到世界坐标, 如果是第一帧，初始化是w1,x0,y0,z0
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();

            // t， 当前局部坐标到世界坐标，初始化是x0, y0, z0
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();

            // 其实还有线速度和角速度

            // publish odometry
            pubLaserOdometry.publish(laserOdometry);



            // geo msgs几何消息，当前时间的位姿戳
            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;



            // 导航消息中的路径信息，nav_msgs::Path
            // 路径，多个位姿
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);

            laserPath.header.frame_id = "camera_init";

            // pub path
            pubLaserPath.publish(laserPath);



            // transform corner features and plane features to the scan end point
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            // 如果只是第一帧，就不匹配，仅仅存储为last点云，作为目标点云
            // 更新当前帧次线点 作为上一帧
            // 次线
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;

            // laserCloudCornerLast:0
            // ...
            // laserCloudCornerLast:1760
            // laserCloudCornerLast:1757
            // cout << "laserCloudCornerLast:" << laserCloudCornerLast->points.size() << endl;
            laserCloudCornerLast = laserCloudTemp; 



            // 次面 
            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;


            // 上一帧的点数
            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            // 更新kdtree的点云，来构建上一帧作为目标点云
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);



             // 进行一定的降频后发给建图， % 5，每5帧
            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                // 上一帧，线点
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                // 上一帧，面点
                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                // 总点 
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }


            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            
            // 超过100ms,输出提示
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");


            frameCount++;
        }



        // 等待，以保持设定的循环频率
        rate.sleep();

    }


    // 执行回调函数，后续代码不执行了
    //ros::spin();


    return 0;
}