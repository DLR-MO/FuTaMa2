// SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
//
// SPDX-License-Identifier: MIT

// Node that allows full control on the robot to change the controller and move its joints
// depending on the selected controller (follow instructions printed on the terminal)

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_C 0x63
#define KEYCODE_J 0x6A
#define KEYCODE_T 0x74

using std::placeholders::_1;

namespace futama2_teleop
{

  // A class for reading the key inputs from the terminal
  class KeyboardReader
  {
  public:
    KeyboardReader() : kfd(0)
    {
      // get the console in raw mode
      tcgetattr(kfd, &cooked);
      struct termios raw;
      memcpy(&raw, &cooked, sizeof(struct termios));
      raw.c_lflag &= ~(ICANON | ECHO);
      // Setting a new line, then end of file
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      tcsetattr(kfd, TCSANOW, &raw);
    }
    void readOne(char *c)
    {
      int rc = read(kfd, c, 1);
      if (rc < 0)
      {
        throw std::runtime_error("read failed");
      }
    }
    void shutdown() { tcsetattr(kfd, TCSANOW, &cooked); }

  private:
    int kfd;
    struct termios cooked;
  };

  // Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
  class KeyboardTeleop : public rclcpp::Node
  {
  public:
    KeyboardTeleop(const rclcpp::NodeOptions &options)
        : Node("keyboard_teleop", options), joint_vel_cmd_(1.0)
    {
      base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
      ee_frame_ = declare_parameter<std::string>("ee_frame", "realsense_center_link");
      frame_to_publish_ = base_frame_;
      control_mode = "";            // starts in unknown state
      previous_msg_button_left = 0; // to help getting just the button change (pull-up) and not full-time state
      previous_msg_button_left = 0; // to help getting just the button change (pull-up) and not full-time state
      button_left_counter = 1;      // Joints: 1=shoulder_pan, 2=shoulder_lift, 3=elbow, 4= wrist_1, 5=wrist_2, 6=wrist_3
      button_right_counter = 1;     // 1="Trajectory Mode, 2="Cartesian Mode", 3="Joint Mode"

      // subscriber to get the axis[6] and the buttons[2] from the spacemouse
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
          "joy", 10, std::bind(&KeyboardTeleop::joy_callback, this, _1));

      // for servo_node
      twist_pub_ =
          create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
      joint_pub_ = 
          create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);

      servo_type_client_ =
          create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
      if (!servo_type_client_->wait_for_service(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(
            get_logger(), "Could not connect to servo_node. Will try again with indefinite timeout.");
        servo_type_client_->wait_for_service();
      }
      RCLCPP_INFO(get_logger(), "Connected to servo_node.");

      controller_type_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
          "controller_manager/switch_controller");
      if (!controller_type_client_->wait_for_service(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(
            get_logger(),
            "Could not connect to controller_manager. Will try again with indefinite timeout.");
        controller_type_client_->wait_for_service();
      }
      RCLCPP_INFO(get_logger(), "Connected to controller_manager.");

      // make sure we start in trajectory mode
      switch_to_trajectory_mode();

      key_thread_ = new std::thread([this]
                                    { quit(keyLoop()); });
      key_thread_->detach();
    }

    void switch_command_type(int type)
    {
      moveit_msgs::srv::ServoCommandType::Request::SharedPtr req =
          std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
      req->command_type = type;
      servo_type_client_->async_send_request(req);
    }
    void switch_to_forward_controller()
    {
      controller_manager_msgs::srv::SwitchController::Request::SharedPtr req =
          std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      req->deactivate_controllers = {"joint_trajectory_controller"};
      req->activate_controllers = {"forward_position_controller"};
      req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      controller_type_client_->async_send_request(req);
    }

    // Mode 1: Trajectory mode
    void switch_to_trajectory_mode()
    {
      if (control_mode != "T")
      {
        controller_manager_msgs::srv::SwitchController::Request::SharedPtr req =
            std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        req->deactivate_controllers = {"forward_position_controller"};
        req->activate_controllers = {"joint_trajectory_controller"};
        req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        controller_type_client_->async_send_request(req);
        puts("\x1b[ATrajectory mode                             \r");
        control_mode = "T";
      }
    }

    // Mode 2: Cartesian mode
    void switch_to_cartesian_mode()
    {
      if (control_mode != "C")
      {
        switch_command_type(1);
        if (control_mode == "T")
        {
          switch_to_forward_controller();
        }
        puts("\x1b[ACartesian mode                             \r");
        control_mode = "C";
      }
    }

    // Mode 3: Joint mode
    void switch_to_joint_mode()
    {
      if (control_mode != "J")
      {
        switch_command_type(0);
        if (control_mode == "T")
        {
          switch_to_forward_controller();
        }
        puts("\x1b[AJoint mode                                \r");
        control_mode = "J";
      }
    }

    int keyLoop()
    {
      char c;
      publish_twist = false;
      publish_joint = false;

      puts("Reading from keyboard (THE COMMANDS WILL BE CHANGED IN THE FUTURE)");
      puts("---------------------------");
      puts("Use 'C' to go into Cartesian mode and 'J' to go into Joint mode.");
      puts("Use 'T' to disable servo and go back into trajectory mode, e.g. for using planners.");
      puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
      puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
      puts("Use 1|2|3|4|5|6 keys to joint jog. 'R' to reverse the direction of jogging.");
      puts("'Q' to quit.");
      puts("---------------------------\n");
      puts("\x1b[ATrajectory mode                         \r");

      for (;;)
      {
        // get the next event from the keyboard
        try
        {
          input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
          perror("read():");
          return -1;
        }

        RCLCPP_DEBUG(get_logger(), "value: 0x%02X\n", c);

        // Create the messages we might publish
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // Use read key-press
        switch (c)
        {
        case KEYCODE_C:
          RCLCPP_DEBUG(get_logger(), "C");
          switch_to_cartesian_mode();
          button_right_counter = 2;
          break;
        case KEYCODE_J:
          RCLCPP_DEBUG(get_logger(), "J");
          switch_to_joint_mode();
          button_right_counter = 3;
          break;
        case KEYCODE_T:
          RCLCPP_DEBUG(get_logger(), "T");
          switch_to_trajectory_mode();
          button_right_counter = 1;
          break;
        case KEYCODE_LEFT:
          RCLCPP_DEBUG(get_logger(), "LEFT");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.x = 1.0;
          publish_twist = true;
          break;
        case KEYCODE_RIGHT:
          RCLCPP_DEBUG(get_logger(), "RIGHT");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.x = -1.0;
          publish_twist = true;
          break;
        case KEYCODE_UP:
          RCLCPP_DEBUG(get_logger(), "UP");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.y = -1.0;
          publish_twist = true;
          break;
        case KEYCODE_DOWN:
          RCLCPP_DEBUG(get_logger(), "DOWN");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.y = 1.0;
          publish_twist = true;
          break;
        case KEYCODE_PERIOD:
          RCLCPP_DEBUG(get_logger(), "PERIOD");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.z = -1.0;
          publish_twist = true;
          break;
        case KEYCODE_SEMICOLON:
          RCLCPP_DEBUG(get_logger(), "SEMICOLON");
          keyboard_cmd_flag = true;
          twist_msg->twist.linear.z = 1.0;
          publish_twist = true;
          break;
        case KEYCODE_E:
          RCLCPP_DEBUG(get_logger(), "E");
          frame_to_publish_ = ee_frame_;
          break;
        case KEYCODE_W:
          RCLCPP_DEBUG(get_logger(), "W");
          frame_to_publish_ = base_frame_;
          break;
        case KEYCODE_1:
          RCLCPP_DEBUG(get_logger(), "1");
          commanded_joint = "shoulder_pan_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 1; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): shoulder_pan_joint                \r");
          break;
        case KEYCODE_2:
          RCLCPP_DEBUG(get_logger(), "2");
          commanded_joint = "shoulder_lift_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 2; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): shoulder_lift_joint             \r");
          break;
        case KEYCODE_3:
          RCLCPP_DEBUG(get_logger(), "3");
          commanded_joint = "elbow_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 3; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): elbow_joint                      \r");
          break;
        case KEYCODE_4:
          RCLCPP_DEBUG(get_logger(), "4");
          commanded_joint = "wrist_1_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 4; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): wrist_1_joint                    \r");
          break;
        case KEYCODE_5:
          RCLCPP_DEBUG(get_logger(), "5");
          commanded_joint = "wrist_2_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 5; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): wrist_2_joint                   \r");
          break;
        case KEYCODE_6:
          RCLCPP_DEBUG(get_logger(), "6");
          commanded_joint = "wrist_3_joint";
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(joint_vel_cmd_);
          publish_joint = true;
          keyboard_cmd_flag = true;
          button_left_counter = 6; // assigning and matching the counter with the one the spacemouse is commanding
          puts("\x1b[AJoint mode (keyboard): wrist_3_joint                   \r");
          break;
        case KEYCODE_R:
          RCLCPP_DEBUG(get_logger(), "R");
          joint_vel_cmd_ *= -1;
          break;
        case KEYCODE_Q:
          RCLCPP_DEBUG(get_logger(), "quit");
          return 0;
        }

        // if spacemouse is disabled (no movement or button from the device), then publish from keyboard
        if (keyboard_cmd_flag == true)
        {
          // If a key requiring a publish was pressed, publish the message now
          if (publish_twist)
          {
            twist_msg->header.stamp = now();
            twist_msg->header.frame_id = frame_to_publish_;
            twist_pub_->publish(std::move(twist_msg));
            publish_twist = false;
          }
          else if (publish_joint)
          {
            joint_msg->header.stamp = now();
            joint_msg->header.frame_id = base_frame_;
            joint_pub_->publish(std::move(joint_msg));
            publish_joint = false;
          }
        }
      }

      return 0;
    }
    void quit(int sig)
    {
      (void)sig;
      input.shutdown();
      rclcpp::shutdown();
      exit(0);
    }

  private:
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_type_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_type_client_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

      int msg_button_left = msg->buttons[1];
      int msg_button_right = msg->buttons[0];
      float spacemouse_roll_vel = msg->axes[3];
      float spacemouse_pitch_vel = msg->axes[4];
      float spacemouse_yaw_vel = msg->axes[5];

      // Create the messages we might publish
      auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

      // Spacemouse: changing control_mode
      if (previous_msg_button_right == 0 && msg_button_right == 1)
      {
        previous_msg_button_right = msg_button_right;
        button_right_counter++;
        if (button_right_counter > 3)
          button_right_counter = 1;
      }
      if (msg_button_right == 0)
        previous_msg_button_right = 0;

      switch (button_right_counter)
      {
        case 1:
          switch_to_trajectory_mode();
          break;
        case 2:
          switch_to_cartesian_mode();
          break;
        case 3:
          switch_to_joint_mode();

        // Spacemouse_ moving the joints in Joint mode
        if (previous_msg_button_left == 0 && msg_button_left == 1)
        {
          previous_msg_button_left = msg_button_left;
          button_left_counter++;
          if (button_left_counter > 6)
            button_left_counter = 1;

          switch (button_left_counter)
          {
          case 1:
            commanded_joint = "shoulder_pan_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            puts("\x1b[AJoint mode (spacemouse): shoulder_pan_joint            \r");
            keyboard_cmd_flag = false;
            break;
          case 2:
            commanded_joint = "shoulder_lift_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            puts("\x1b[AJoint mode (spacemouse): shoulder_lift_joint           \r");
            keyboard_cmd_flag = false;
            break;
          case 3:
            commanded_joint = "elbow_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            puts("\x1b[AJoint mode (spacemouse): elbow_joint                  \r");
            keyboard_cmd_flag = false;
            break;
          case 4:
            commanded_joint = "wrist_1_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            puts("\x1b[AJoint mode (spacemouse): wrist_1_joint                \r");
            keyboard_cmd_flag = false;
            break;
          case 5:
            commanded_joint = "wrist_2_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            puts("\x1b[AJoint mode (spacemouse): wrist_2_joint                \r");
            keyboard_cmd_flag = false;
            break;
          case 6:
            commanded_joint = "wrist_3_joint";
            joint_msg->joint_names.push_back(commanded_joint);
            commanded_joint = "wrist_3_joint";
            puts("\x1b[AJoint mode (spacemouse): wrist_3_joint                \r");
            keyboard_cmd_flag = false;
            break;
          }
        }
        if (msg_button_left == 0)
          previous_msg_button_left = 0; // this helps to detect only the pull-up changed in the buttons

        // publish velocities from an "intuitive" spacemouse vel cmd ("3" is smoothened, reduce it to make it faster)
        spacemouse_joint_vel_cmd = (spacemouse_roll_vel + spacemouse_pitch_vel + spacemouse_yaw_vel) / 3;

        // if one sends spacemouse cmds, disable keyboard cmds
        if (spacemouse_joint_vel_cmd != 0)
        {
          keyboard_cmd_flag = false;
          // puts("\x1b[AJoint mode (spacemouse)                              \r");
        }

        // if keyboard is disabled (no logging), then publish from spacemouse (movement or button required)
        if (keyboard_cmd_flag == false)
        {
          joint_msg->joint_names.push_back(commanded_joint);
          joint_msg->velocities.push_back(spacemouse_joint_vel_cmd);

          twist_msg->header.stamp = now();
          twist_msg->header.frame_id = frame_to_publish_;
          twist_pub_->publish(std::move(twist_msg));

          joint_msg->header.stamp = now();
          joint_msg->header.frame_id = base_frame_;
          joint_pub_->publish(std::move(joint_msg));
        }
      }
    }

    // variable global declaration
    std::string base_frame_;
    std::string ee_frame_;
    std::string frame_to_publish_;
    double joint_vel_cmd_;
    double spacemouse_joint_vel_cmd;
    KeyboardReader input;
    std::thread *key_thread_;
    std::string control_mode;
    std::int8_t msg_button_left;
    std::int8_t msg_button_right;
    std::int8_t previous_msg_button_left;
    std::int8_t previous_msg_button_right;
    std::int8_t button_left_counter;
    std::int8_t button_right_counter;
    std::string commanded_joint = "shoulder_pan_joint";
    double spacemouse_roll_vel;
    double spacemouse_pitch_vel;
    double spacemouse_yaw_vel;
    bool publish_twist;
    bool publish_joint;
    bool keyboard_cmd_flag = false;
  };
} // namespace futama2_teleop closing

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<futama2_teleop::KeyboardTeleop>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_teleop::KeyboardTeleop)