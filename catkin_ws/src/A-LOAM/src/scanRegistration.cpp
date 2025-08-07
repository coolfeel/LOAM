



#include <iostream>
#include <vector>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include "aloam_velodyne/tic_toc.h"
#include "aloam_velodyne/common.h"

using std::atan2;
using std::sin;
using std::cos;


const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;


const double scanPeriod = 0.1;

int N_SCANS = 0;

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];


bool comp (int i, int j)
{
    return (cloudCurvature[i] < cloudCurvature[j]);
}


// 定义话题发布者对象
// 总点
ros::Publisher pubLaserCloud;

// 特征点，线点
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
// 特征点，面点
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
// 删除的点
ros::Publisher pubRemovePoints;

std::vector<ros::Publisher> pubEachScan;



bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1;



// 剔除近距离的点
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // 检查输入和输出点云是否为不同的对象pcl::PointCloud<pcl::PointXYZ>
    if (&cloud_in != &cloud_out)
    {
        // 复制点云的头信息
        cloud_out.header = cloud_in.header;
        // 重设输出点云的大小，个数
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); i++)
    {
        // 计算点到原点的距离的平方
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
        {
            // 如果距离小于阈值，忽略该点
            continue;
        }
        // 将有效的点复制到输出点云
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }

    // 调整输出点云的大小，如果有些点被移除了
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    // 设置输出点云的其他属性
    // 高度为1，表示这是一个无序点云，宽度与点云大小相同
    cloud_out.height = 1;

    // 宽度等于有效点的数量
    cloud_out.width = static_cast<uint32_t>(j);

    // 有限的数值大小，不包括包含inf/NaN
    cloud_out.is_dense = true;
}




// 订阅数据的回调函数，处理传感器消息中的总点云
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // 判断系统是否初始化完成   
    if (!systemInited)
    {
        // 如果系统初始化计数达到预设的延迟值，则标记系统为初始化完成 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            // 如果系统还未初始化完成，则返回，不处理这个点云
            return;
    }

    // 创建两个时间记录对象，用于测量处理时间
    TicToc t_whole;
    TicToc t_prepare;

    // 初始化两个vector，记录每个扫描线scan的起始和结束索引
    // 16线16个元素, 初始0
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    
    // PCL 点云对象,点云类型，用于存储转换后的点云数据
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;

    // ROS 消息中的点云数据sensor_msgs::PointCloud2转换为 PCL 点云格式
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    // 28453
    // std::cout << laserCloudIn.points.size() << std::endl;

    // 创建一个整数向量，用于存储去除 NaN 点后的索引
    std::vector<int> indices;

    // 移除点云中的 NaN 点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    // 28419
    // std::cout << laserCloudIn.points.size() << std::endl;

    // 移除距离过近的点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    // 点云中点的总数
    int cloudSize = laserCloudIn.points.size();

    // 14230
    // printf("%d", cloudSize);

    // 计算起始和结束方向的角度, 方位角度
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    // 调整结束角度，以确保它在合理范围内
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    // 0.254120
    // 6.587222
    // printf("%f", startOri);
    // printf("%f", endOri);

    // 标志位，用于判断是否处理了一半的点云
    bool halfPassed = false;

    // 有效点计数
    int count = cloudSize;


    // 当前处理的点 pcl::PointXYZI PointType;
    PointType point;

    // 存储每条scan扫描线的点云, 16个点云对象分别存储
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    // 遍历总点云中的每个点， 为点分配scan线，16线总共点数cloudSize，并计算强度intensity
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // 俯仰角度
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;

        int scanID = 0;

        // 计算每个点，所在线的索引scanID
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);


        // 计算方位角度
        float ori = -atan2(point.y, point.x);

        // 判断并调整点的方位角，以保持连续性
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        // 计算相对时间
        float relTime = (ori - startOri) / (endOri - startOri);

        // 设置点的强度为扫描线ID加上相对时间
        point.intensity = scanID + scanPeriod * relTime;

        // 将点添加到对应扫描线的点云中
        laserCloudScans[scanID].push_back(point);

    }


    // 16线的点的总数
    cloudSize = count;

    // point size: 14243 
    // printf("point size: %d \n", cloudSize);

    // 一个scan线
    // 901 
    // 900
    // 903
    // 16线是14000点
    // printf("%d\n", laserCloudScans[0].points.size());


    
    // 创建一个新的总点云来存储总的扫描数据，带强度
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

    // 遍历每一个线，把点云拼起来，已经有强度intensity
    for (int i = 0; i < N_SCANS; i++)
    {
        // 0 901 1808 2710 ... 13355
        // printf("%d\n", laserCloud->size());

        // 为当前扫描设置开始索引，考虑到边界，记录每个scan的开始index，忽略前5个点
        scanStartInd[i] = laserCloud->size() + 5;

        // 将当前扫描的点云数据加入到总的点云中
        *laserCloud += laserCloudScans[i];

        // 设置结束索引，考虑到边界，记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的接缝，对提取特征不利
        scanEndInd[i] = laserCloud->size() - 6;

    }

    // 输出准备阶段耗费的时间
    // prepare time: 1.099979 
    printf("prepare time: %f \n", t_prepare.toc());


    // 判断总点中每个点，为提取特征点做准备
    for (int i = 5; i < cloudSize - 5; i++)
    {
        // 计算每个点相对于其邻居点的曲率, 使用差分方法, 10个邻域点和点i作差, 加权差分方法来增强中心点的影响
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        
        // 计算曲率并存储
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;

        // 每个点的索引,用于排序
        cloudSortInd[i] = i;

        // 点有没有被选选择为feature点
        cloudNeighborPicked[i] = 0;

        // Label 2: corner_sharp
        // Label 1: corner_less_sharp, 包含Label 2
        // Label -1: surf_flat
        // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
        cloudLabel[i] = 0;

    
    }

    // 开始提取特征点
    TicToc t_pts;

    // 存储各种特征点的点云
    // 角点
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;

    // 面点
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    // 用于记录排序时间
    float t_q_sort = 0;

    // 遍历每个扫描线上点
    for (int i = 0; i < N_SCANS; i++)
    {
        // 第i线scan
        // 如果扫描的点太少，就跳过这个扫描
        if (scanEndInd[i] - scanStartInd[i] < 6)
        {
            continue;
        }

        // 为较平的表面点创建一个新的点云,用于后续滤波，lessflat点太多
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>());

        // 将每个扫描线上的点，分成6部分进行处理，分段处理
        for (int j = 0; j < 6; j++)
        {
            // 在每个scan开始和结束索引的基础上，确定一个scan内，每部分subscan的起始和结束索引
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;

            // 对当前部分的点按曲率进行排序，5～150，根据曲率有小到大对subscan的点进行sort
            std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            t_q_sort += t_tmp.toc();

            //在一个subscan内确定线，面特征点，用label标记

            // subscan内，曲率最大的被选点数，如果目标点位于一个棱角或尖角位置，它与周围点的坐标差异会比较大，导致计算出的曲率值较高
            int largestPickedNum = 0;

            // 从后往前，即从曲率大的点开始提取corner feature
            for (int k = ep; k >= sp; k--)
            {
                // 每个点的索引
                int ind = cloudSortInd[k]; 

                // 如果该点没有被选择过，并且曲率大于0.1
                // 角点
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    // 该subscan中曲率最大的前2个点认为是corner_sharp特征点
                    if (largestPickedNum <= 2)
                    {                        
                        // Label 2: corner_sharp
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    // 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                    else if (largestPickedNum <= 20)
                    {                        
                        // Label 1: corner_less_sharp, 包含Label 2
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    // 标记该点被选择过了
                    cloudNeighborPicked[ind] = 1; 

                    // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }


                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            
            // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
            
            int smallestPickedNum = 0;

            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                // 面点
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                {
                    // Label -1: surf_flat
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    // 被选
                    cloudNeighborPicked[ind] = 1;

                    //避免密集分布
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
            for (int k = sp; k <= ep; k++)
            {
                // Label -1: surf_flat
                // Label 0: surf_less_flat， 包含Label -1
                if (cloudLabel[k] <= 0)
                {
                    // 点多，要降采样
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }

        }

        // 最后对该scan点云中提取的所有surf_less_flat特征点进行降采样，点太多
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        // 体素滤波
        pcl::VoxelGrid<PointType> downSizeFilter;
        // 降采样
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        // 结果
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        // less flat
        surfPointsLessFlat += surfPointsLessFlatScanDS;

    }
    // 这里提取了各种特征点，并且对次平面点进行了一次降采样

    // sort q time 0.332059 
    // seperate points time 0.927423 
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());




    
    // 接下来就是对当前总点云、特征点云发布，供给laserodometry 通信，
    // 将预处理的数据传给里程计节点

    // 总点，ros传感器消息结果
    sensor_msgs::PointCloud2 laserCloudOutMsg;

    // 总点云的指针laserCloud，to ros msg
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);

    // 输入的ROS MSG，laserCloudMsg
    // 时间戳、帧id（传感器消息）
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "camera_init";

    // MSG进行发布，总点的发布
    pubLaserCloud.publish(laserCloudOutMsg);


    // 14196
    // printf("%d\n", (*laserCloud).points.size());



    // 线点，格式转换+发布
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    // 192
    // printf("%d\n", cornerPointsSharp.points.size());


    // 次线点
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    // 1763
    // printf("%d\n", cornerPointsLessSharp.points.size());



    // 面点
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    // 370
    // printf("%d\n", surfPointsFlat.points.size());


    // 次面点
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // 5901
    // printf("%d\n", surfPointsLessFlat.points.size());



    // pub each scam
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());

    if(t_whole.toc() > 100)
    {
        ROS_WARN("scan registration process over 100ms");
    }


}



// main
int main(int argc, char **argv)
{
    printf("coolslam\n");

    // 初始化ros节点
    ros::init(argc, argv, "scanRegistration");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 从 ROS 参数服务器获取扫描线数量，如果未设置则默认为 16
    nh.param<int>("scan_line", N_SCANS, 16);

    // 获取最小测量范围，如果未设置则默认为 0.1
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    // 16
    // printf("scan line number: %d \n", N_SCANS);


    if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // 订阅激光雷达原始点云数据，话题/velodyne_points，回调处理函数laserCloudHandler
    // 接收 sensor_msgs::PointCloud2 类型的消息
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);



    // 创建发布消息的对象
    // 总点
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
    // 线点
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    // 面点
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    // 删除的点
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);


    if (PUB_EACH_LINE)
    {
        for (int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }

    // 执行回调函数，后续代码不执行了
    ros::spin();


    return 0;
}