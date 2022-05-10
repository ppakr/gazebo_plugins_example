// #include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/common/Plugin.hh>

#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ignition/math/Vector3.hh>
// #include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

#include <math.h>
#include <map>
#include <memory>

namespace gazebo
{
    class GazeboExample : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model1`
            this->model = _parent;

            this->inputCommand = 0.0;

            this->joint = this->model->GetJoint("joint");

            this->ClampMin = -1.0;
            this->ClampMax = 1.0;
            this->gain = 1.0;
            this->tau = 9.5;

            // Create ROS Node
            myRosNode = gazebo_ros::Node::CreateWithArgs("gazebo_example_node");
            // Create Subscription
            // mySubVel = myRosNode->create_subscription<geometry_msgs::msg::TwistStamped>(
            //     "vel_sub", 10,
            //     std::bind(&GazeboExample::SetReference, this, std::placeholders::_1));
            
            mySubPose = myRosNode->create_subscription<geometry_msgs::msg::PoseStamped>(
                "poseSub", 10,
                std::bind(&GazeboExample::SetReference, this, std::placeholders::_1)
            );

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboExample::OnUpdate, this, std::placeholders::_1));
        }

            // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo &_info)
        {
            double dynamicsInput;
            // Test if the thruster has been turned off
            if (this->inputCommand)
            {
                double clamped = this->inputCommand;
                clamped = std::min(clamped, this->ClampMax);
                clamped = std::max(clamped, this->ClampMin);

                dynamicsInput = this->gain*clamped;
            }
            else
            {
                dynamicsInput = 0.0;
            }

            this->jointPosition = DynamicsFirstOrder(_info, dynamicsInput);
            this->joint->SetPosition(0, this->jointPosition);
        }

        public: double DynamicsFirstOrder(const common::UpdateInfo &_info, double _cmd)
        {
            _t = _info.simTime.Double();
            if (prevTime < 0)
            {
                prevTime = _t;
                return state;
            }

            double dt = _t - prevTime;

            double alpha = std::exp(-dt/this->tau);
            state = state*alpha + (1.0 - alpha)*_cmd;

            prevTime = _t;

            return state;
        }


        public: void SetReference(
        const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
        {
            if (std::isnan(_msg->pose.position.x))
            {
                RCLCPP_WARN(myRosNode->get_logger(), "SetReference Vel: Ignoring nan command");
                return;
            }
            this->inputCommand = _msg->pose.position.x;
            // RCLCPP_WARN(myRosNode->get_logger(), "I heard");
        }


        // Pointer to the model
        private: physics::ModelPtr model;

        private: physics::JointPtr joint;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        
        private: double inputCommand;

        private: double ClampMin;
        private: double ClampMax;
        private: double gain;

        private: double _t;
        private: double prevTime;
        private: double _cmd;
        private: double state;
        private: double jointPosition;

        private: float tau;

        protected: gazebo_ros::Node::SharedPtr myRosNode;

        // private: rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mySubVel;
        private: rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mySubPose;

        protected: transport::SubscriberPtr commandSubscriber;
    };
    
    GZ_REGISTER_MODEL_PLUGIN(GazeboExample)
}