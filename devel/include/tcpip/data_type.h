#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_
namespace LEECHAO
{
/**
 * @brief 感知端与控制端通信数据，一个float4个字节
*/
struct  Com_Data
{
    // 行走方向，相对当前机器人状态，yaw方向
    char flag = 'A';
    float direct = 0.;
    float left_arm_joints[6] = {0.};
    float right_arm_joints[6] = {0.};
    float left_leg_joints[6] = {0.};
    float right_leg_joints[6] = {0.};
};

/**
 * @brief 落脚点数据格式
*/
struct FootStep
{
    bool is_left;
    float x, y, z, roll, yaw, pitch;
    FootStep():is_left(1),x(0.),y(0.),z(0.),roll(0.),pitch(0.),yaw(0.)
    {

    }
    FootStep(float x_, float y_, float z_, float roll_, float yaw_, float pitch_):x(x_), y(y_), z(z_), roll(roll_), yaw(yaw_), pitch(pitch_)
    {

    }
};



}

#endif