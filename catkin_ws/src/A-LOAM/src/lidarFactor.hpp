#include <ceres/ceres.h>


// 线点
struct LidarEdgeFactor
{
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double s_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_)
    {

    }


    // 重载()实现自定义残差计算, 用输入的参数通过计算，算出残差用于求导
    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        //写成模板的形式，类似于vector，定义当前点和两个端点
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};


        //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		// q的插值，当前到上一帧的变换
        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        q_last_curr = q_identity.slerp(T(s), q_last_curr);

        // t的插值
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        Eigen::Matrix<T, 3, 1> lp;

        //四元数球面线性插值
        // cp点三当前点，此时还没有变换到开始时刻，通过当前变换进行插值后的结果来把当前点变换到开始时刻
        lp = q_last_curr * cp + t_last_curr;

        // 面积和底，求距离
        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        //向量求模即是点到直线距离
        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        return true;
    }


    // 代价函数
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_, const Eigen::Vector3d last_point_b_, const double s_)
    {
        // ceres::AutoDiffCostFunction是用自动求导的方式，是一个类模板，
        // 需要传入参数类型实例化为模板类（类名，输出维度（误差，维度3），输入维度（二个参数，维度4和3）），然后传入实际参数来实例化出一个类，也可以自己求雅克比传给ceres
        return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
    }


    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
};



// 面点
struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
        // 底面积, j和l同scan
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}


	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};

		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};

		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};

        // 变换插值求变换
		q_last_curr = q_identity.slerp(T(s), q_last_curr);

		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};


		Eigen::Matrix<T, 3, 1> lp;
        // 变换到上一帧
		lp = q_last_curr * cp + t_last_curr;

        // 论文中i为lp，i和j距离最近作为高
		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}


    // 代价函数
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
        // 自动求导
		return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}


	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};



// 面，法向
struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_) 
        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_) 
    {

    }

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
        // 当前局部帧下点，变换到世界坐标下地图点 
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};

		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;

		point_w = q_w_curr * cp + t_w_curr;

        // 法向量
		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		
        // 残差
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}


	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_, const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};