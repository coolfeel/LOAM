
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <ceres/ceres.h>

#include <mutex>
#include <queue>
#include <thread>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

using namespace std;


int frameCount = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;


int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;

const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;

// cube的总数量，也就是上图中的小格子个总数量 21 * 21 * 11 = 4851
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851


//  下面两个变量是一模一样的，有点冗余，记录submap中的有效cube的index，submap中cube的最大数量为 5 * 5 * 5 = 125
int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());


// 点云特征匹配时的优化变量
double parameters[7] = {0, 0, 0, 1, 0, 0, 0};


// Mapping线程估计的frame在world坐标系的位姿P,因为Mapping的算法耗时很有可能会超过100ms,所以这个位姿P不是实时的
// LOAM最终输出的实时位姿P_realtime,需要Mapping线程计算的相对低频位姿 和 Odometry线程计算的相对高频位姿做整合，见laserOdometryHandler函数分析。
// 不同于Odometry线程，这里的位姿P，即q_w_curr和t_w_curr，本身就是匹配时的优化变量。
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);


// world坐标系下的Odometry计算的位姿和Mapping计算的位姿之间的增量（也即变换，transformation）
// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);


// Odometry线程(laserOdo.cpp)计算的frame在world坐标系的位姿
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);




// 线，面，总点的存储队列，包含传感器消息的点云指针
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;



// 导航消息的里程计信息
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;

std::mutex mBuf;




// 体素栅格滤波器
pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;



std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

PointType pointOri, pointSel;

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

// 导航消息的路径
nav_msgs::Path laserAfterMappedPath;


// set initial guess，上一帧的增量wmap_wodom * 本帧Odometry位姿wodom_curr，旨在为本帧Mapping位姿w_curr设置一个初始值
void transformAssociateToMap()
{
	// q_wmap_wodom世界坐标下里程计位姿——世界坐标下地图
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

// 由上推出
// 用在最后，当Mapping的位姿w_curr计算完毕后，更新增量wmap_wodom，旨在为下一次执行transformAssociateToMap函数时做准备
void transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}


// 当前帧点变换到世界坐标下
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}


// 世界坐标下点变换到当前帧
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}




// 线点
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
    
    mBuf.lock();
    cornerLastBuf.push(laserCloudCornerLast2);
    mBuf.unlock();

}

// 面点
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
    
    mBuf.lock();
    surfLastBuf.push(laserCloudSurfLast2);
    mBuf.unlock();
}

// 总点
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}


// 导航消息中里程计信息
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    mBuf.lock();
    odometryBuf.push(laserOdometry);
    mBuf.unlock();
    

    // 区分里程计位姿 和 建图位姿
    // 高频接收
    // 高频的里程计位姿(laserOdo.cpp发布，由此接收处理), 当前到世界坐标的里程计位姿
    Eigen::Quaterniond q_wodom_curr; 
    Eigen::Vector3d t_wodom_curr; 

    q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
    q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
    q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
    q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;

    t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
    t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
    t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

    // 为了保证LOAM整体的实时性，防止Mapping线程耗时>100ms导致丢帧
	// 用上一次的增量wmap_wodom(世界坐标下里程计到世界坐标下地图的变换)来更新Odometry的位姿，旨在用Mapping位姿的初始值（也可以理解为预测值）来实时输出
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 


    // 高频发布mapping后的里程计信息
    // high frequence publish，直到世界坐标下建图后的里程计信息
    nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";

	odomAftMapped.header.stamp = laserOdometry->header.stamp;

	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();

	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();

	pubOdomAftMappedHighFrec.publish(odomAftMapped);



}


// 多线程执行
void process()
{
	
    while(1)
    {
		
		
        // 保证LOAM算法整体的实时性，Mapping线程每次只处理cornerLastBuf.front()及其他与之时间同步的消息
		// Mapping线程耗时>100ms导致的历史缓存都会被clear
        while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty())
        {
			// 一帧
            mBuf.lock();

			// 1
			// 1
			// 1
			// 1
			// cout << cornerLastBuf.size() << endl;
			// cout << odometryBuf.size() << endl;
			// cout << surfLastBuf.size() << endl;
			// cout << fullResBuf.size() << endl;

			// 1.42213e+09
			// 1.42213e+09
			// 1.42213e+09
			// 1.42213e+09
			// cout << cornerLastBuf.front()->header.stamp.toSec() << endl;
			// cout << odometryBuf.front()->header.stamp.toSec() << endl;
			// cout << surfLastBuf.front()->header.stamp.toSec() << endl;
			// cout << fullResBuf.front()->header.stamp.toSec() << endl;


            // 选帧处理, 以cornerLastBuf为参考，删除之前不同步的
            // odometryBuf中的时间同步的最新消息 
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
			{
                odometryBuf.pop();
            }

			if (odometryBuf.empty())
			{
				mBuf.unlock();
				break;
			}

            // surfLastBuf也同理
			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
			{
                surfLastBuf.pop();
            }

			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

            // fullResBuf也同理 
			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
			{
                fullResBuf.pop();
            }

			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// 1
			// 1
			// 1
			// 1
			// cout << cornerLastBuf.size() << endl;
			// cout << odometryBuf.size() << endl;
			// cout << surfLastBuf.size() << endl;
			// cout << fullResBuf.size() << endl;


            // 时间戳
            timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

			
			

            // 如果时间不同步 
            if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
				printf("unsync messeage!");
				mBuf.unlock();
				break;
			}

			
			

            // 接收队列中存储的上一帧，线
            laserCloudCornerLast->clear();
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
			cornerLastBuf.pop(); 


            // 接收上一帧， 面
			laserCloudSurfLast->clear();
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
			surfLastBuf.pop();

            
            // 接收上一帧，总点
			laserCloudFullRes->clear();
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
			fullResBuf.pop();


            // 接收上一帧，里程计的位姿
            q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;

			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;

			odometryBuf.pop();



            // 清空cornerLastBuf的历史缓存，保持空队列
            while(!cornerLastBuf.empty())
			{
				cornerLastBuf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}

			mBuf.unlock();




            // 
            TicToc t_whole;

            // 上一帧的增量wmap_wodom * 本帧Odometry位姿wodom_curr，旨在为本帧Mapping位姿w_curr设置一个初始值
			transformAssociateToMap();



            TicToc t_shift;

			// 0.00111978
			// 0.00191922
			// -0.0256823
			// cout << t_w_curr.x() << endl;
			// cout << t_w_curr.y() << endl;
			// cout << t_w_curr.z() << endl;


			
            // 计算当前帧位置t_w_curr（在上图中用红色五角星表示的位置）IJK坐标（见上图中的坐标轴），
			// 参照LOAM_NOTED的注释，有关25和50的运算，等效于以50m为单位进行缩放，因为LOAM用1维数组
			// 进行cube的管理，而数组的index只用是正数，所以要保证IJK坐标都是正数，所以加了laserCloudCenWidth/Heigh/Depth的偏移
			// 来使得当前位置尽量位于submap的中心处，也就是使得上图中的五角星位置尽量处于所有格子的中心附近，
			// 偏移laserCloudCenWidth/Heigh/Depth会动态调整，来保证当前位置尽量位于submap的中心处。

			
			// laserCloudCenWidth = 10, laserCloudCenHeight = 10, laserCloudCenDepth = 5;
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;



            // 求余是向零取整，不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;


            
			// 以下注释部分参照LOAM_NOTED，结合我画的submap的示意图说明下面的6个while loop的作用：
			// 注意世界坐标系下的点云地图是固定的，但是IJK坐标系我们是可以移动的，调整IJK坐标系（也就是调整所有cube位置），使得五角星在IJK坐标系的坐标范围处于
		    // 3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18，目的是为了防止后续向
			// 四周拓展cube（图中的黄色cube就是拓展的cube）时，index（即IJK坐标）成为负数。
			// I
            while (centerCubeI < 3)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{ 
						int i = laserCloudWidth - 1;
						
                        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						
						// 在I方向上，将cube[I] = cube[I-1],最后一个空出来的cube清空点云，实现IJK坐标系向I轴负方向移动一个cube的效果
						// 从相对运动的角度看，就是图中的五角星在IJK坐标系下向I轴正方向移动了一个cube，如下面的动图所示，所
				        // 以centerCubeI最后++，laserCloudCenWidth也会++，为下一帧Mapping时计算五角星的IJK坐标做准备。
                        for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI++;
				laserCloudCenWidth++;
			}


            while (centerCubeI >= laserCloudWidth - 3)
			{ 
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						
                        
                        // 在I方向上，将cube[I] = cube[I-1],最后一个空出来的cube清空点云，实现IJK坐标系向I轴负方向移动一个cube的效果
						// 从相对运动的角度看，就是在IJK坐标系下向I轴正方向移动了一个cube
				        // 所以centerCubeI最后++，laserCloudCenWidth也会++，为下一帧Mapping时计算IJK坐标做准备。
                        for (; i < laserCloudWidth - 1; i++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			// J
            while (centerCubeJ < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			// K
			while (centerCubeK < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}



            // submap中有效cube数
            int laserCloudValidNum = 0;
			// submap中环境cube数
			int laserCloudSurroundNum = 0; 

			
			
			// 在submap中扩展cube 
            // 向IJ坐标轴的正负方向各拓展2个cube，K坐标轴的正负方向各拓展1个cube，上图中五角星所在的蓝色cube就是当前位置
			// 所处的cube，拓展的cube就是黄色的cube，这些cube就是submap的范围
            for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
                        // 如果坐标合法
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{ 
                            // 1维数组记录submap中的所有cube的index，记为有效index
							// submap中cube的最大数量为 5 * 5 * 5 = 125
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							// 2381,2822,1961,2402
							// cout << laserCloudValidInd[laserCloudValidNum] << endl;

							// 同理
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}

			// 75 cube
			// 75 cube
			// cout << laserCloudValidNum << endl;
			// cout << laserCloudSurroundNum << endl;
			

            // submap点
            laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();

			// 在每个cube内 
            for (int i = 0; i < laserCloudValidNum; i++)
			{
				// cube的总数量，也就是上图中的小格子总数量 21 * 21 * 11 = 4851
				// laserCloudCornerArray存储每个cube的指针
                // 将有效index的cube中的点云叠加到一起组成submap的特征点云
				
				// 线
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				// 面
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}

            // 
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

			
			// map prepare time 0.376622 ms
			printf("map prepare time %f ms\n", t_shift.toc());
			// map corner num 5571,  surf num 4308 
			printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
			
			

			// 体素滤波, 对局部坐标上一帧点
			// 线
            pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);

			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			// 面
			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);

			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
			

			

			// submap点合理
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
			{
				TicToc t_opt;
				TicToc t_tree;

				// submap点放入kdtree
				// 线
				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
				// 面
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

				printf("build tree time %f ms \n", t_tree.toc());


				// 迭代 mapping的优化
				for (int iterCount = 0; iterCount < 2; iterCount++)
				{
					//ceres::LossFunction *loss_function = NULL;

					// 损失函数
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
					
					// ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
					// 定义流形参数，四元数处理
					ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();

					// 定义问题配置
					ceres::Problem::Options problem_options;
					// 定义问题
					ceres::Problem problem(problem_options);

					// 定义参数块
					problem.AddParameterBlock(parameters, 4, q_parameterization);
					problem.AddParameterBlock(parameters + 4, 3);

					TicToc t_data;


					// 线点，关联
					int corner_num = 0;
					
					// 体素滤波后的点数量
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						// curr局部上一帧点
						pointOri = laserCloudCornerStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						
						// 变换到世界坐标下地图点, mapping位姿
						// 需要注意的是submap中的点云都是world坐标系，而当前帧的点云都是Lidar坐标系，所以
						// 在搜寻最近邻点时，先用预测的Mapping位姿w_curr，将Lidar坐标系下的特征点变换到world坐标系下
						pointAssociateToMap(&pointOri, &pointSel);

						// submap中找5个近邻点
						// 在submap的corner特征点（target）中，寻找距离当前帧corner特征点（source）最近的5个点
						kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

						// 5点中最大距离都合理
						if (pointSearchSqDis[4] < 1.0) 
						{ 
							// 存
							std::vector<Eigen::Vector3d> nearCorners;
							Eigen::Vector3d center(0, 0, 0);

							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
								nearCorners.push_back(tmp);
							}
							// 均值, 计算这个5个最近邻点的中心
							center = center / 5.0;
							
							// 协方差矩阵
							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

							for (int j = 0; j < 5; j++)
							{
								
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
							}// 不除n?


							// 计算协方差矩阵的特征值和特征向量，用于判断这5个点是不是呈线状分布，此为PCA的原理
							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							// 如果5个点呈线状分布，最大的特征值对应的特征向量就是该线的方向向量 
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

							// curr局部上一帧点
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

							// 如果最大的特征值 >> 其他特征值，则5个点确实呈线状分布，否则认为直线不够直
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
							{ 
								Eigen::Vector3d point_on_line = center;
								Eigen::Vector3d point_a, point_b;

								// 从中心点沿着方向向量向两端移动0.1m，构造线上的两个点
								point_a = 0.1 * unit_direction + point_on_line;
								point_b = -0.1 * unit_direction + point_on_line;

								//构造点到线的距离
								// 然后残差函数的形式就跟Odometry一样了，残差距离即点到线的距离，到介绍lidarFactor.cpp时再说明具体计算方法
								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
								
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								
								corner_num++;	
							}							
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}



					// 面点，关联
					int surf_num = 0;

					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						// 求面的法向量就不是用的PCA了（虽然论文中说还是PCA），使用的是最小二乘拟合，是为了提效？
						// 假设平面不通过原点，则平面的一般方程为Ax + By + Cz + 1 = 0，用这个假设可以少算一个参数
						Eigen::Matrix<double, 5, 3> matA0;
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

						// 用上面的2个矩阵表示平面方程就是 matA0 * norm（A, B, C） = matB0，这是个超定方程组，因为数据个数超过未知数的个数
						// 距离合理
						if (pointSearchSqDis[4] < 1.0)
						{
							
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}

							// 求解这个最小二乘问题，可得平面的法向量，find the norm of plane
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);

							// Ax + By + Cz + 1 = 0，全部除以法向量的模长，方程依旧成立，而且使得法向量归一化了
							double negative_OA_dot_norm = 1 / norm.norm();
							norm.normalize();


							// 判断平面是否拟合好，Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;

							for (int j = 0; j < 5; j++)
							{
								// 点(x0, y0, z0)到平面Ax + By + Cz + D = 0 的距离公式 = fabs(Ax0 + By0 + Cz0 + D) / sqrt(A^2 + B^2 + C^2)
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
								{
									// 平面没有拟合好，平面不够平
									planeValid = false;
									break;
								}
							}

							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

							// 拟合好，计算面残差
							if (planeValid)
							{
								// 构造点到面的距离残差项，同样的，具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
								
								// 残差块
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								
								surf_num++;
							}
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					//printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					//printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

					printf("mapping data assosiation time %f ms \n", t_data.toc());



					TicToc t_solver;

					// 定义求解器的配置
					ceres::Solver::Options options;

					// 配置参数
					options.linear_solver_type = ceres::DENSE_QR;
					options.max_num_iterations = 4;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;

					// 优化信息
					ceres::Solver::Summary summary;

					ceres::Solve(options, &problem, &summary);

					printf("mapping solver time %f ms \n", t_solver.toc());

					//printf("time %f \n", timeLaserOdometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}

				printf("mapping optimization time %f \n", t_opt.toc());

			}
			else
			{
				ROS_WARN("time Map corner and surf num are not enough");
			}



			// 完成ICP（迭代2次）的特征匹配后，用最后匹配计算出的优化变量w_curr，更新增量wmap_wodom，为下一次Mapping做准备
			transformUpdate();



			TicToc t_add; 
			// 位姿优化好后的转点
			// 下面两个for loop的作用就是将当前帧的特征点云，逐点进行操作：转换到world坐标系并添加到对应位置的cube中
			// 线点
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				// Lidar坐标系转到world坐标系
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

				// 计算本次的特征点的IJK坐标，进而确定添加到哪个cube中
				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					// points in every cube
					laserCloudCornerArray[cubeInd]->push_back(pointSel);
				}
			}

			// 同理，面点
			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudSurfArray[cubeInd]->push_back(pointSel);
				}
			}
			printf("add points time %f ms\n", t_add.toc());

			


			TicToc t_filter;
			// 因为新增加了点云，对之前已经存有点云的cube全部重新进行一次降采样
			// 这个地方可以简单优化一下：如果之前的cube没有新添加点就不需要再降采样
			// cube数
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				// cube索引
				int ind = laserCloudValidInd[i];

				// 线
				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
				// cube索引找到cube指针的cube内点云
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*tmpCorner);
				laserCloudCornerArray[ind] = tmpCorner;

				// 面
				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*tmpSurf);
				laserCloudSurfArray[ind] = tmpSurf;
			}
			printf("filter time %f ms \n", t_filter.toc());



			
			TicToc t_pub;

			// 每5帧，就发布世界坐标环境submap点
			//publish surround map for every 5 frame
			if (frameCount % 5 == 0)
			{
				// ouput: all visualble cube points
				// 清空之前的，重新发布当前已有的
				laserCloudSurround->clear();

				// cube数
				for (int i = 0; i < laserCloudSurroundNum; i++)
				{
					// cube索引
					int ind = laserCloudSurroundInd[i];

					// 环境点是由所有 线 和 面点组成
					*laserCloudSurround += *laserCloudCornerArray[ind];
					*laserCloudSurround += *laserCloudSurfArray[ind];
				}

				// PCL格式 转成sensor msg
				sensor_msgs::PointCloud2 laserCloudSurround3;
				pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);

				laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudSurround3.header.frame_id = "camera_init";
				// 发布
				pubLaserCloudSurround.publish(laserCloudSurround3);
			}


			// 每20帧，发布所有地图点
			if (frameCount % 20 == 0)
			{
				pcl::PointCloud<PointType> laserCloudMap;

				// cube的总数量，也就是上图中的小格子个总数量 21 * 21 * 11 = 4851
				for (int i = 0; i < 4851; i++)
				{
					laserCloudMap += *laserCloudCornerArray[i];
					laserCloudMap += *laserCloudSurfArray[i];
				}

				sensor_msgs::PointCloud2 laserCloudMsg;
				pcl::toROSMsg(laserCloudMap, laserCloudMsg);

				laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudMsg.header.frame_id = "camera_init";
				// 发布
				pubLaserCloudMap.publish(laserCloudMsg);
			}


			//input & output: points in one frame. local --> global
			int laserCloudFullResNum = laserCloudFullRes->points.size();
			// 一帧中所有点
			for (int i = 0; i < laserCloudFullResNum; i++)
			{
				// 从局部坐标帧变换到世界坐标下 
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);

			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			laserCloudFullRes3.header.frame_id = "camera_init";
			// 发布一帧的点
			pubLaserCloudFullRes.publish(laserCloudFullRes3);



			printf("mapping pub time %f ms \n", t_pub.toc());

			printf("whole mapping time %f ms +++++\n", t_whole.toc());



			// mapping后的里程计信息
			nav_msgs::Odometry odomAftMapped;
			odomAftMapped.header.frame_id = "camera_init";
			odomAftMapped.child_frame_id = "/aft_mapped";
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			// 当前局部帧到世界坐标下地图的位姿
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
			odomAftMapped.pose.pose.position.x = t_w_curr.x();
			odomAftMapped.pose.pose.position.y = t_w_curr.y();
			odomAftMapped.pose.pose.position.z = t_w_curr.z();

			pubOdomAftMapped.publish(odomAftMapped);


			// mapping后的位姿戳
			geometry_msgs::PoseStamped laserAfterMappedPose;
			laserAfterMappedPose.header = odomAftMapped.header;
			// 
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;


			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
			laserAfterMappedPath.header.frame_id = "camera_init";

			// 位姿戳构成导航消息的路径
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);

			// 发布导航消息的路径
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);



			// ROS的tf库用于处理坐标系间的转换，简化了广播和监听坐标变换的过程。广播tf涉及设置坐标原点和旋转角度
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;

			// 设置原点位置
			transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));

			// 姿态
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());

			transform.setRotation(q);

			// 该函数将当前坐标系（以父坐标系为参考）的变换信息广播到 TF 树中
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "/aft_mapped"));

			frameCount++;



        }


		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


// mapping main
int main(int argc, char **argv)
{
    cout << "mapping" << endl;
    ros::init(argc, argv, "laserMapping");
    
    ros::NodeHandle nh;

    // 体素栅格的分辨率
    float lineRes = 0;
	float planeRes = 0;

    // 从参数服务器中取线和面点的分辨率，没有就默认值 
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);

    // 0.2 0.4
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);

    // 体素滤波器，下采样，设置体素栅格大小
    downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
    downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);


    // 订阅laserOdo.cpp发布的线、面、总点信息
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);
    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

    // 订阅里程计信息
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);



    // 定义发布
	// submap环境点
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

	// 所有地图点
	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

	// 一帧的点云
	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

    // mapping后的里程计信息
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

	// 高频发布mapping后的里程计信息
	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

	// 导航消息的路径
	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

    // 每个cube 内的点云重置空
    for (int i = 0; i < laserCloudNum; i++)
	{
        // 线点
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        // 面点
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}

	

    // 多线程执行
	std::thread mapping_process{process};

	ros::spin();
    
    return 0;
}