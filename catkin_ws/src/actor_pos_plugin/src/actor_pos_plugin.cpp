#include <string>
#include <vector>
#include <deque>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math.hh>
#include <Eigen/Core>
#include "gazebo_msgs/ModelStates.h"
#include <angles/angles.h>
#include <std_msgs/Empty.h>  // 添加Empty消息类型头文件
#include <ros/timer.h>       // 添加定时器头文件
#include <geometry_msgs/Twist.h>  // 新增：订阅机器狗速度

#define WALKING_ANIMATION "walking"

namespace gazebo
{

class GZ_PLUGIN_VISIBLE ActorPosPlugin : public ModelPlugin
  {     /// \brief Constructor
    public: ActorPosPlugin();

    // \brief Load the actor plugin.
    // \param[in] _model Pointer to the parent model.
    // \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: void modelStateCallback(const gazebo_msgs::ModelStates & msg);
    // 新增机器狗速度回调函数
    public: void dogCmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg);

    // Documentation Inherited.
    public: virtual void Reset();



    // \brief Function that is called every update cycle.
    // \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);
    private: void OnUpdate_1(const common::UpdateInfo &_info);
    // 新增：计算机器狗加速度
    private : ignition::math::Vector3d computeDogAcceleration();
    // 新增：发布人类位置
    private : void publishHumanPos(const ignition::math::Vector3d& pos);

    // \brief Helper function to choose a new target location
    private: void ChooseNewTarget();

    // \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    // \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    // \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    // \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    // \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    private: ros::NodeHandle* _node_handle;
    private: ros::Publisher  _man_pos_pub;
    private: ros::Subscriber  _dog_pos_sub;
    private: ros::Subscriber  _dog_cmd_vel_sub;  // 新增：订阅机器狗速度
    // \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    // \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    // \brief Current target location
    private: ignition::math::Vector3d target;

    // \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    // \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    // \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    // \brief Time of the last update.
    private: common::Time lastUpdate;

    // \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    // \brief Custom trajectory info.
     private: physics::TrajectoryInfoPtr trajectoryInfo;
    // 这里是机器狗状态
    private: Eigen::Vector3d _dog_pos;
    private: ignition::math::Vector3d rpy_dog;
    // 新增：机器狗线速度
    private: ignition::math::Vector3d dog_linear_vel;
    // 新增：机器狗角速度
    private: ignition::math::Vector3d dog_angular_vel;
    // 新增：线速度历史
    private: std::deque<ignition::math::Vector3d> dog_linear_vel_history;  // 线速度历史
    // 新增：速度时间戳
    private: std::deque<gazebo::common::Time> dog_vel_time;  // 时间戳（gazebo时间类型）
    // 保留最近五个速度数据
    private: const int vel_history_size = 5;
    //  private: ignition::math::Vector3d dog_velocity; // 机器狗速度
    // 这里是人类状态
    private:
      double delta = 0.0;  // 距离偏移量
      double delta_dot = 0.0;  // 距离偏移率
      double delta_ddot = 0.0;  // 距离偏移加速度
      double theta_h = 0.0;  // human朝向
      double theta_h_dot = 0.0;  // human转向角速度
      double theta_h_ddot = 0.0;  // human转向角加速度

      // 物理参数
      const double L0 = 1.0;  // 基础距离（初始1m）
      const double m = 5.0;  // 手臂等效质量(kg)
      const double c = 20.0;  // 伸缩阻尼系数(N·s/m)
      const double k = 100.0;  // 伸缩刚度系数(N/m)
      const double I = 1.0;  // 转动惯量(kg·m²)
      const double b = 15.0;  // 转向阻尼系数(N·m·s/rad)
      const double k_tau = 50.0;  // 转向力矩系数(N·m/rad)
      const double phi = M_PI * 0.25;  // 相对方位角（45°）
      const double human_mass = 60.0;  // 人类质量(kg)      

      // 交互力与力矩
      double F_t = 0.0;  // 牵引力（沿杆方向）
      double M_r = 0.0;  // 转向力矩
      double F_h = 0.0;  // 导盲杆合力


    private: 
      ros::Subscriber resetSub;
      ros::Timer resetDelayTimer;
      bool isResetRequested = false;
    private: void OnRLResetSignal(const std_msgs::Empty::ConstPtr& msg);
    private: void OnResetDelayTimeout(const ros::TimerEvent& event);


  };

/////////////////////////////////////////////////
ActorPosPlugin::ActorPosPlugin()
{
  
}

/////////////////////////////////////////////////
void ActorPosPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  
  _node_handle = new ros::NodeHandle();
  _man_pos_pub = _node_handle->advertise<geometry_msgs::PointStamped>("/actor_pos", 1);
  _dog_pos_sub = _node_handle->subscribe("/gazebo/model_states", 1, &ActorPosPlugin::modelStateCallback, this );
  // 新增 订阅机器狗速度
  _dog_cmd_vel_sub = _node_handle->subscribe("/go2_gazebo/cmd_vel", 10, &ActorPosPlugin::dogCmdVelCallback, this);


  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPosPlugin::OnUpdate, this, std::placeholders::_1)));


  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model"); 
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
////////////////////////////////////////////////////////
  // 初始化ROS节点
  // this->rosNode = new ros::NodeHandle();
  // 订阅强化学习发布的reset信号（/go2_gazebo/reset）
  this->resetSub = this->_node_handle->subscribe(
      "/go2_gazebo/reset", 1, &ActorPosPlugin::OnRLResetSignal, this);

  // 初始化延迟定时器（回调函数在0.5秒后执行）
  this->resetDelayTimer = this->_node_handle->createTimer(
      ros::Duration(0.5),  // 延迟0.5秒
      &ActorPosPlugin::OnResetDelayTimeout, this,
      false,  // 不自动启动
      false   // 初始为未启动状态
  );
}

// n订阅

// 接收强化学习的reset信号回调
void ActorPosPlugin::OnRLResetSignal(const std_msgs::Empty::ConstPtr& msg) {
    gzmsg << "收到强化学习reset信号，将在0.5秒后执行人类模型重置..." << std::endl;
    isResetRequested = true;
    // 启动定时器（0.5秒后触发OnResetDelayTimeout回调）
    resetDelayTimer.start();
}

// 定时器回调：1秒后执行人类模型重置
void ActorPosPlugin::OnResetDelayTimeout(const ros::TimerEvent& event) {
    if (!isResetRequested) return;

    // ====== 1秒后，获取机器狗重置后的坐标 ======
    std::string dogModelName = "go2";  // 机器狗模型名称
    physics::ModelPtr dogModel = this->world->ModelByName(dogModelName);
    if (!dogModel) {
        gzerr << "未找到机器狗模型 " << dogModelName << "！" << std::endl;
        isResetRequested = false;
        return;
    }

    // 获取机器狗重置后的位置和姿态
    ignition::math::Pose3d dogPose = dogModel->WorldPose();
    _dog_pos[0] = dogPose.Pos().X();  // 重置后的X坐标
    _dog_pos[1] = dogPose.Pos().Y();  // 重置后的Y坐标
    _dog_pos[2] = dogPose.Pos().Z();
    rpy_dog = dogPose.Rot().Euler();  // 重置后的姿态（尤其是yaw角）
    gzmsg << "1秒延迟后，获取机器狗重置后坐标：" 
          << _dog_pos[0] << ", " << _dog_pos[1] << std::endl;

    // ====== 执行人类模型的重置 ======
    ignition::math::Pose3d humanPose = this->actor->WorldPose();

    // 按原有逻辑计算人类位置（机器狗右后方45度，距离0.5米）
    double dog_yaw = rpy_dog.Z();  // 机器狗重置后的yaw角
    double offset_angle = dog_yaw - M_PI * 3 / 4;  // 偏移角度
    humanPose.Pos().X() = _dog_pos[0] + L0 * cos(offset_angle);
    humanPose.Pos().Y() = _dog_pos[1] + L0 * sin(offset_angle);
    humanPose.Pos().Z() = 1;  // 高度固定

    // 设置人类朝向（与机器狗匹配）
    humanPose.Rot() = ignition::math::Quaterniond(1.5707, 0, dog_yaw + M_PI / 2);

    // 同步动力学状态变量
    delta = 0;  // 距离偏移量0
    delta_dot = 0;  // 距离偏移率0
    theta_h = dog_yaw;  // 保存机器狗原始yaw角
    theta_h_dot = 0;  // human转向角速度0

    // 强制更新人类模型位置
    this->actor->SetWorldPose(humanPose);
    gzmsg << "人类模型已基于机器狗重置后坐标完成重置：" << humanPose.Pos() << std::endl;

    // 重置动画（保留原有逻辑）
    auto skelAnims = this->actor->SkeletonAnimations();
    if (skelAnims.find(WALKING_ANIMATION) != skelAnims.end()) {
        this->trajectoryInfo.reset(new physics::TrajectoryInfo());
        this->trajectoryInfo->type = WALKING_ANIMATION;
        this->trajectoryInfo->duration = 1.0;
        this->actor->SetCustomTrajectory(this->trajectoryInfo);
    }

    // 重置标志
    isResetRequested = false;
    resetDelayTimer.stop();
}

///////////////////////////////////////////////

void ActorPosPlugin::modelStateCallback(const gazebo_msgs::ModelStates & msg) {
    bool found = false;
    int index = 0;
    std::string _model_name="go2";

    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
      ignition::math::Quaterniond q(msg.pose[index].orientation.w, msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z);
      rpy_dog=q.Euler();

      _dog_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z;
      

      // 获取机器狗速度（新增）
      // ignition::math::Vector3d dog_velocity(
      //   msg.twist[index].linear.x,
      //   msg.twist[index].linear.y,
      //   msg.twist[index].linear.z
      // );
      // this->dog_velocity = dog_velocity;  // 保存到类成员变量
    }
}


// 新增订阅机器狗速度
void ActorPosPlugin::dogCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    dog_linear_vel = ignition::math::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
    dog_angular_vel = ignition::math::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z);

    // 缓存速度历史（用于计算加速度）
    gazebo::common::Time curr_time = gazebo::common::Time::GetWallTime();
    dog_linear_vel_history.push_back(dog_linear_vel);
    dog_vel_time.push_back(curr_time);
    if (dog_linear_vel_history.size() > vel_history_size) {
        dog_linear_vel_history.pop_front();
        dog_vel_time.pop_front();
    }
}

// 新增计算机器狗加速度
ignition::math::Vector3d ActorPosPlugin::computeDogAcceleration() {
    if (dog_linear_vel_history.size() < 2) return ignition::math::Vector3d::Zero;
    auto vel1 = dog_linear_vel_history[dog_linear_vel_history.size()-2];
    auto vel2 = dog_linear_vel_history[dog_linear_vel_history.size()-1];
    gazebo::common::Time t1 = dog_vel_time[dog_vel_time.size()-2];
    gazebo::common::Time t2 = dog_vel_time[dog_vel_time.size()-1];
    double dt = (t2 - t1).Double();
    return (dt < 1e-6) ? ignition::math::Vector3d::Zero : (vel2 - vel1)/dt;
}

// 新增 发布人类位置
void ActorPosPlugin::publishHumanPos(const ignition::math::Vector3d& pos) {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.point.x = pos.X();
    msg.point.y = pos.Y();
    msg.point.z = pos.Z();
    _man_pos_pub.publish(msg);
}


///////////////////////////////////////////////
void ActorPosPlugin::Reset()
{
  this->velocity = 0.05;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

///////////////////////////////////////////////
void ActorPosPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}
///////////////////////////////////////////////
void ActorPosPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPosPlugin::OnUpdate(const common::UpdateInfo &_info)
{

 common::Time currTime = _info.simTime ;

  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();
  // 固定速度
  this->velocity = 1;
  
  ignition::math::Pose3d pose = this->actor->WorldPose();

  // 人体位置为机器狗右后方四十五度距离为0.6米
  double dog_yaw = rpy_dog.Z(); // 机器狗的yaw角
  double offset_angle = dog_yaw - M_PI * 3 / 4; // 偏移角度

  ignition::math::Vector3d pos_target;
  pos_target.X() = _dog_pos[0] + 0.6 * cos(offset_angle);
  pos_target.Y() = _dog_pos[1] + 0.6 * sin(offset_angle);
  pos_target.Z() = 1;

  // 更新位置和朝向
  ignition::math::Vector3d pos = pos_target - pose.Pos();
  pos = pos.Normalize() * this->targetWeight;

  // 设置人类朝向
  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, dog_yaw + M_PI / 2);
  // 更新位置
  pose.Pos() += pos * this->velocity * dt;

  // 发布人类位置
  ros::Time now = ros::Time::now();
  geometry_msgs::PointStamped p;
  p.header.stamp.sec = now.sec;
  p.header.stamp.nsec = now.nsec;
  p.point.x = pose.Pos().X();
  p.point.y = pose.Pos().Y();
  p.point.z = pose.Pos().Z();
  _man_pos_pub.publish(p);

  // 动画
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();
  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

void ActorPosPlugin::OnUpdate_1(const common::UpdateInfo &_info)
{
  double dt = (_info.simTime - this->lastUpdate).Double();
  if (dt < 1e-6)
    return;
  // 1. 获取机器狗当前状态
  ignition::math::Vector3d dog_pos = ignition::math::Vector3d(_dog_pos[0], _dog_pos[1], _dog_pos[2]);
  double dog_yaw = rpy_dog.Z(); // 机器狗的yaw角

  // 2. 计算机器狗加速度与交互力
  ignition::math::Vector3d dog_acc = computeDogAcceleration();
  F_h = human_mass * dog_acc.Length();  // 导盲杆合力

  // 3. 计算朝向偏差与力矩
  double theta_diff = angles::shortest_angular_distance(dog_yaw, theta_h);  // 朝向偏差
  F_t = F_h * cos(theta_diff);  // 牵引力（沿杆方向）
  M_r = k_tau * theta_diff;  // 转向力矩

  // 4. 更新距离偏差（二阶动力学）
  delta_ddot = (F_t - c*delta_dot - k*delta) / m;  // 加速度
  delta_dot += delta_ddot * dt;  // 速度
  delta = ignition::math::clamp(delta, -0.2, 0.2);  // 限制偏差范围（±0.2m）
  delta += delta_dot * dt;  // 位置

  // 5. 更新人类朝向（二阶动力学）
  theta_h_ddot = (M_r - b*theta_h_dot) / I;  // 角加速度
  theta_h_dot += theta_h_ddot * dt;  // 角速度
  theta_h += theta_h_dot * dt;  // 角度
  theta_h = angles::normalize_angle(theta_h);  // 归一化到[-π, π]
  // 6. 计算新的人类位置
  double actual_distance = L0 + delta;
  double angle = theta_h + phi;  // 相对角度，不包含pi/2
  ignition::math::Vector3d new_pos;
  new_pos.X() = dog_pos.X() - actual_distance * cos(angle);
  new_pos.Y() = dog_pos.Y() - actual_distance * sin(angle);
  new_pos.Z() = 1;  // 高度保持不变
  // 7. 平滑更新人类姿态（应用+M_PI/2补偿）
  ignition::math::Pose3d current_pose = actor->WorldPose();
  ignition::math::Vector3d pos_diff = new_pos - current_pose.Pos();
  current_pose.Pos() += pos_diff * 0.2;
  current_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, theta_h + M_PI/2);  // 关键：应用+M_PI/2补偿
  // 8. 应用姿态并发布
  actor->SetWorldPose(current_pose);
  publishHumanPos(current_pose.Pos());  
  // 9. 更新动画（匹配移动距离）
  double move_dist = pos_diff.Length();
  actor->SetScriptTime(actor->ScriptTime() + move_dist * 4.5);  // 动画速率
  lastUpdate = _info.simTime;
}

    GZ_REGISTER_MODEL_PLUGIN(ActorPosPlugin)
}

