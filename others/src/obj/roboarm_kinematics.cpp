#include "roboarm_kinematics.hpp"
#include <cmath>


void RoboArmJointAngles::Update() const
{
	// 基座坐标系旋转矩阵 R 的三列，初始为单位阵（R_0^0）
	float rx[3] = {1.f, 0.f, 0.f};
	float ry[3] = {0.f, 1.f, 0.f};
	float rz[3] = {0.f, 0.f, 1.f};

	// 从底部向末端逐关节解算。joints_[i] 对应连杆 i+1，其局部系为 frame_{i+1}。
	// 先累乘当前关节的旋转 R = R * Rot_X(alpha_i) * Rot_Z(theta_{i+1}) 得到
	// frame_{i+1} 相对基座的旋转，再把连杆局部方向向量 mdh.axis 变换到基座系：
	// g = R * axis，最后与基座 z 轴 (0,0,1) 点积求倾角。
	for (uint8_t i = 0; i < RoboArm_Kine_N::dof; ++i)
	{
		const Joint& j = joints_[i].get();

		// 累乘旋转矩阵：R = R * Rot_X(alpha_i) * Rot_Z(theta_{i+1})
		const float theta = j.angle.theta;
		const float alpha = j.Mdh().alpha;
		const float ct = cosf(theta), st = sinf(theta);
		const float ca = cosf(alpha), sa = sinf(alpha);

		// Rot_X(alpha) * Rot_Z(theta) 的三列：
		// col0 = (ct, st*ca, st*sa), col1 = (-st, ct*ca, ct*sa), col2 = (0, -sa, ca)
		float nx[3], ny[3], nz[3];
		for (uint8_t k = 0; k < 3; ++k)
		{
			nx[k] = ct * rx[k] + st * ca * ry[k] + st * sa * rz[k];
			ny[k] = -st * rx[k] + ct * ca * ry[k] + ct * sa * rz[k];
			nz[k] = -sa * ry[k] + ca * rz[k];
		}
		for (uint8_t k = 0; k < 3; ++k)
		{
			rx[k] = nx[k];
			ry[k] = ny[k];
			rz[k] = nz[k];
		}

		// 连杆方向向量在基座系下的坐标 g = R * axis；
		// 与基座 z 轴 (0,0,1) 的点积只需 z 分量 gz，倾角 = arccos(gz)
		const float* axis = j.Mdh().axis;
		const float gz = rx[2] * axis[0] + ry[2] * axis[1] + rz[2] * axis[2];

		const float c = gz > 1.f ? 1.f : (gz < -1.f ? -1.f : gz);
		joints_[i].get().angle.base = acosf(c);
	}
}
