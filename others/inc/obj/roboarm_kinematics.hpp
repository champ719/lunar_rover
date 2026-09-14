#pragma once

#include <cstdint>
#include <array>
#include <functional>
#include "roboarm.hpp"


/* 机械臂连杆倾角解算（极简版）
 *
 * 由各关节电机的编码角度，从基座底部向末端逐关节累乘旋转矩阵，把每根
 * 连杆在其局部坐标系中的方向向量（mdh.axis）变换到基座坐标系，再与
 * 基座 z 轴（竖直方向）点积求倾角。
 *
 * offset / alpha / theta_offset / axis 统一存放在 Joint 类中，此处直接
 * 读取，不涉及平移 a/d、雅可比等传统动力学矩阵。
 */

namespace RoboArm_Kine_N
{
	inline constexpr uint8_t dof = 7;  // 关节数，按实际电机数调整
}

class RoboArmJointAngles
{
	public:
	explicit RoboArmJointAngles(const std::array<std::reference_wrapper<Joint>, RoboArm_Kine_N::dof>& joints)
		: joints_(joints)
	{}

	/* 从底部逐关节解算，把各连杆与基座 z 轴的倾角写入 Joint::angle.base（rad，0..pi） */
	void Update() const;

	private:
	const std::array<std::reference_wrapper<Joint>, RoboArm_Kine_N::dof> joints_;
};
