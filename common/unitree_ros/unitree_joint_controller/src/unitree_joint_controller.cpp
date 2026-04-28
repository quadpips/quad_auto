#include "unitree_joint_controller/unitree_joint_controller.hpp"

namespace unitree_joint_controller
{
    UnitreeJointController::UnitreeJointController()
    : controller_interface::ControllerInterface(), dof_(0) // joint_names_({}), command_buffer_(nullptr)
    {  
        // Constructor implementation  
    }

    controller_interface::CallbackReturn UnitreeJointController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
    
            auto logger = this->get_node()->get_logger();

            ////////////
            // PARAMS //
            ////////////

            // // update the dynamic map parameters
            // param_listener_->refresh_dynamic_parameters();

            // // get parameters from the listener in case they were updated
            // params_ = param_listener_->get_params();

            // get degrees of freedom
            dof_ = params_.joints.size();

            command_joint_names_ = params_.command_joints;

            if (command_joint_names_.empty())
            {
                command_joint_names_ = params_.joints;
                RCLCPP_INFO(
                logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
            }
            else if (command_joint_names_.size() != params_.joints.size())
            {
                RCLCPP_ERROR(
                logger, "'command_joints' parameter has to have the same size as 'joints' parameter.");
                return CallbackReturn::FAILURE;
            }

            RCLCPP_INFO_STREAM(logger, "params_.joints: ");
            for (const auto & joint_name : params_.joints)
            {
                RCLCPP_INFO_STREAM(logger, "  " << joint_name);
            }

            RCLCPP_INFO_STREAM(logger, "command_joint_names_: ");
            for (const auto & joint_name : command_joint_names_)
            {
                RCLCPP_INFO_STREAM(logger, "  " << joint_name);
            }
            
            ////////////
            // JOINTS //
            ////////////

            if (params_.joints.empty())
            {
                // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
                RCLCPP_WARN(logger, "'joints' parameter is empty.");
            }       

            ////////////////////////
            // COMMAND INTERFACES //
            ////////////////////////

            if (params_.command_interfaces.empty())
            {
                RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
                return CallbackReturn::FAILURE;
            }

            // Check if only allowed interface types are used and initialize storage to avoid memory
            // allocation during activation
            joint_command_interface_.resize(allowed_interface_types_.size());       

            if (params_.state_interfaces.empty())
            {
                RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
                return CallbackReturn::FAILURE;
            }
            
            // Check if only allowed interface types are used and initialize storage to avoid memory
            // allocation during activation
            // Note: 'effort' storage is also here, but never used. Still, for this is OK.
            joint_state_interface_.resize(allowed_interface_types_.size());

            auto get_interface_list = [](const std::vector<std::string> & interface_types)
            {
                std::stringstream ss_interfaces;
                for (size_t index = 0; index < interface_types.size(); ++index)
                {
                    if (index != 0)
                    {
                        ss_interfaces << " ";
                    }
                    ss_interfaces << interface_types[index];
                }
                return ss_interfaces.str();
            };

            std::string node_namespace = get_node()->get_namespace();

            // Print output so users can be sure the interface setup is correct
            RCLCPP_INFO(
                logger, "Command interfaces are [%s] and state interfaces are [%s].",
                get_interface_list(params_.command_interfaces).c_str(),
                get_interface_list(params_.state_interfaces).c_str());

            joint_command_subscriber_ = 
                get_node()->create_subscription<unitree_go::msg::MotorCmds>(
                        "~/command", rclcpp::SystemDefaultsQoS(),
                        std::bind(&UnitreeJointController::command_callback, this, std::placeholders::_1));

            // State publisher
            publisher_ = get_node()->create_publisher<unitree_go::msg::MotorStates>(
                            "~/state", 100);
            state_publisher_ = std::make_unique<StatePublisher>(publisher_);

            state_current_.states.resize(dof_);
            state_tmin1_.states.resize(dof_);
            state_tmin2_.states.resize(dof_);
            command_current_.cmds.resize(dof_);

            // unitree_go::msg::MotorCmds command_current;
            // command_current.cmds.resize(dof_);

            command_buffer_.reset();
            command_buffer_.initRT(command_current_);

            // Load URDF model
            std::string urdf_string = get_node()->get_parameter("robot_description").value_to_string();
            if (!model_.initString(urdf_string))
            {
                RCLCPP_ERROR(logger, "Failed to parse URDF model from 'robot_description' parameter.");
                return CallbackReturn::ERROR;
            }
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        // Initialization logic
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration UnitreeJointController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if (dof_ == 0)
        {
            fprintf(
            stderr,
            "During ros2_control interface configuration, degrees of freedom is not valid;"
            " it should be positive. Actual DOF is %zu\n",
            dof_);
            std::exit(EXIT_FAILURE);
        }
        conf.names.reserve(dof_ * params_.command_interfaces.size());
        for (const auto & joint_name : command_joint_names_)
        {
            for (const auto & interface_type : params_.command_interfaces)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return conf;
    }

    controller_interface::InterfaceConfiguration UnitreeJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        conf.names.reserve(dof_ * params_.state_interfaces.size());
        for (const auto & joint_name : params_.joints)
        {
            for (const auto & interface_type : params_.state_interfaces)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return conf;
    }

    controller_interface::return_type UnitreeJointController::update(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // rclcpp::Time now = get_node()->now();
        // rclcpp::Duration period = now - last_command_time_;
        // timeSinceLastLog_ += period.seconds();
        // bool logCheck = (timeSinceLastLog_ > 1.0);

        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            return controller_interface::return_type::OK;
        }

        auto logger = this->get_node()->get_logger();

        unitree_go::msg::MotorCmds last_command = *(command_buffer_.readFromRT());

        unitree_go::msg::MotorCmds final_cmds;
        final_cmds.cmds.resize(dof_);

        for (size_t index = 0; index < dof_; ++index)
        {
            // grab last command
            unitree_go::msg::MotorCmd & raw_cmd = last_command.cmds[index];
            unitree_go::msg::MotorCmd final_cmd;

            // pre-process command
            if (raw_cmd.mode == PMSM)
            {
                // RCLCPP_INFO(logger, "Processing PMSM command for joint %zu", index);
                final_cmd.mode = PMSM;

                // set position
                final_cmd.q = raw_cmd.q;
                positionLimits(params_.joints[index], final_cmd.q);
                final_cmd.kp = raw_cmd.kp;
                if (fabs(raw_cmd.q - PosStopF) < 0.00001)
                {
                    // RCLCPP_WARN(logger, "Position command is close to stop position, setting kp to 0.");
                    final_cmd.kp = 0;
                }
                
                // set velocity
                final_cmd.dq = raw_cmd.dq;
                velocityLimits(params_.joints[index], final_cmd.dq);
                final_cmd.kd = raw_cmd.kd;
                if (fabs(raw_cmd.dq - VelStopF) < 0.00001)
                {
                    // RCLCPP_WARN(logger, "Velocity command is close to stop velocity, setting kd to 0.");
                    final_cmd.kd = 0;
                }

                // set torque
                final_cmd.tau = raw_cmd.tau;
                effortLimits(params_.joints[index], final_cmd.tau);
            } else if (raw_cmd.mode == BRAKE)
            {
                // RCLCPP_INFO(logger, "Processing BRAKE command for joint %zu", index);
                final_cmd.mode = BRAKE;

                final_cmd.kp = 0.0;

                final_cmd.dq = 0.0;
                final_cmd.kd = 0.0;

                final_cmd.tau = 0.0;

            } else
            {
                RCLCPP_ERROR(logger, "Unknown motor mode: %d", raw_cmd.mode);
            }

            // assign to final command
            final_cmds.cmds[index] = final_cmd;
        }

        // grab current state
        read_state_from_state_interfaces(state_current_);

        // if (logCheck)
        // {
        // RCLCPP_INFO(this->get_node()->get_logger(), "[UnitreeJointController::update()]");

        // RCLCPP_INFO(logger, "Processed command with %zu joints.", final_cmds.cmds.size());
        // for (size_t i = 0; i < final_cmds.cmds.size(); ++i)
        // {
        //     RCLCPP_INFO(logger, "Joint %zu: mode=%d, q=%.2f, dq=%.2f, tau=%.2f, kp=%.2f, kd=%.2f",
        //         i, final_cmds.cmds[i].mode, final_cmds.cmds[i].q,
        //         final_cmds.cmds[i].dq, final_cmds.cmds[i].tau, final_cmds.cmds[i].kp, final_cmds.cmds[i].kd);
        // }
    
        // RCLCPP_INFO(logger, "Current state: %zu joints.", state_current_.states.size());
        // for (size_t i = 0; i < state_current_.states.size(); ++i)
        // {
        //     RCLCPP_INFO(logger, "Joint %zu: q=%.2f, dq=%.2f, tau_est=%.2f",
        //         i, state_current_.states[i].q, state_current_.states[i].dq, state_current_.states[i].tau_est);
        // }
        // }

        // if (logCheck)
        // {        
        //     RCLCPP_INFO(logger, "Computing error and applying commands...");
        // }

        // compute error
        for (size_t index = 0; index < dof_; ++index)
        {
            unitree_go::msg::MotorCmd & final_cmd = final_cmds.cmds[index];
            const unitree_go::msg::MotorState state = state_current_.states[index];
            const unitree_go::msg::MotorState state_tmin1 = state_tmin1_.states[index];
            const unitree_go::msg::MotorState state_tmin2 = state_tmin2_.states[index];

            float current_position = state.q;
            float current_velocity = computeVel(current_position, 
                                                    state_tmin1.q, 
                                                    state_tmin1.dq, 
                                                    period.seconds());

            // filter velocity
            // float current_velocity = (current_velocity_raw + state_tmin1.dq + state_tmin2.dq) * 0.3333f;
            
            // compute torque
            float current_torque = computeTorqueCustom(current_position, current_velocity, final_cmd);
            effortLimits(params_.joints[index], current_torque);

            // assign to interface
            joint_command_interface_[3][index].get().set_value(current_torque);

            // if (logCheck)
            // {                
            // RCLCPP_INFO(logger, "Joint %zu: %s", index, params_.joints[index].c_str());
            // RCLCPP_INFO(logger, "  Command: mode=%d, q=%.2f, dq=%.2f, tau=%.2f, kp=%.2f, kd=%.2f",
            //                 final_cmd.mode, final_cmd.q, final_cmd.dq,
            //                 final_cmd.tau, final_cmd.kp, final_cmd.kd);

            // RCLCPP_INFO(logger, "  State: q=%.2f, dq=%.2f, tau=%.2f",
            //                 current_position, current_velocity, current_torque);

            // RCLCPP_INFO(logger, "Joint %zu: command set to %.2f", index, current_torque);
            // }

            state_current_.states[index].dq = current_velocity;
            state_current_.states[index].tau_est = current_torque;
        }

        // Update the state for the next iteration
        state_tmin2_ = state_tmin1_;
        state_tmin1_ = state_current_;

        // Publish state
        if (state_publisher_ && state_publisher_->trylock())
        {
            state_publisher_->msg_ = state_current_;
            state_publisher_->unlockAndPublish();
        }
        else
        {
            RCLCPP_WARN(logger, "Failed to lock state publisher for publishing.");
        }

        // if (logCheck)
        // {
        //     timeSinceLastLog_ = 0.0;
        // }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_configure(
        const rclcpp_lifecycle::State & previous_state)
    {
        // auto logger = get_node()->get_logger();

        // if (!param_listener_)
        // {
        //     RCLCPP_ERROR(logger, "Error encountered during init");
        //     return controller_interface::CallbackReturn::ERROR;
        // }


        return controller_interface::CallbackReturn::SUCCESS;
    }

    void UnitreeJointController::command_callback(const unitree_go::msg::MotorCmds & msg)
    {
        // RCLCPP_INFO(get_node()->get_logger(), "[command_callback()] Received command with %li commands",
        //     msg.cmds.size());

        // for (size_t i = 0; i < msg.cmds.size(); ++i)
        // {
        //     RCLCPP_INFO(get_node()->get_logger(), "Command for joint %li", i);
        //     RCLCPP_INFO(get_node()->get_logger(), "         q: %.2f, dq: %.2f, tau: %.2f",
        //         msg.cmds[i].q, msg.cmds[i].dq, msg.cmds[i].tau);
        // }
        
        command_current_ = msg;

        // Update the command buffer with the new command
        command_buffer_.writeFromNonRT(command_current_);
        return;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        auto logger = get_node()->get_logger();

        // update the dynamic map parameters
        param_listener_->refresh_dynamic_parameters();

        // get parameters from the listener in case they were updated
        params_ = param_listener_->get_params();

        // order all joints in the storage
        for (const auto & interface : params_.command_interfaces)
        {
            auto it =
            std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
            auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
            if (!controller_interface::get_ordered_interfaces(
                command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
            {
                RCLCPP_ERROR(
                    logger, "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(),
                    joint_command_interface_[index].size());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO_STREAM(
                logger, "Ordered command interface for '" << interface << "' with size "
                << joint_command_interface_[index].size() << " and names: ");
            for (const auto & cmd_interface : joint_command_interface_[index])
            {
                RCLCPP_INFO_STREAM(logger, "  " << cmd_interface.get().get_name());
                RCLCPP_INFO_STREAM(logger, "  " << cmd_interface.get().get_value());
            }
        }

        for (const auto & interface : params_.state_interfaces)
        {
            auto it =
            std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
            auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
            if (!controller_interface::get_ordered_interfaces(
                state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
            {
                RCLCPP_ERROR(
                    logger, "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(),
                    joint_state_interface_[index].size());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO_STREAM(
                logger, "Ordered state interface for '" << interface << "' with size "
                << joint_state_interface_[index].size() << " and names: ");
            for (const auto & state_interface : joint_state_interface_[index])
            {
                RCLCPP_INFO_STREAM(logger, "  " << state_interface.get().get_name());
                RCLCPP_INFO_STREAM(logger, "  " << state_interface.get().get_value());
            }
        }

        // Initialize current state storage from hardware
        read_state_from_state_interfaces(state_current_);
        // read_state_from_state_interfaces(state_tmin1_);
        // read_state_from_state_interfaces(state_tmin2_);
        state_tmin1_ = state_current_;
        state_tmin2_ = state_current_;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool UnitreeJointController::read_state_from_state_interfaces(unitree_go::msg::MotorStates & state)
    {    
        auto assign_point_from_interface =
            [&](unitree_go::msg::MotorStates & states, const auto & joint_interface)
        {
            for (size_t index = 0; index < dof_; ++index)
            {
                states.states[index].q = joint_interface[index].get().get_value();
            }
        };

        assign_point_from_interface(state, joint_state_interface_[0]);

        return true;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
    {
        // Deactivation logic

        for (size_t index = 0; index < dof_; ++index)
        {
            joint_command_interface_[0][index].get().set_value(0.0);
        }

        for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
        {
            joint_command_interface_[index].clear();
            joint_state_interface_[index].clear();
        }
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_cleanup(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_error(
        const rclcpp_lifecycle::State & previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool UnitreeJointController::contains_interface_type(const std::vector<std::string> & interface_type_list, 
                                                            const std::string & interface_type)
    {
        return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
                interface_type_list.end();
    }

    void UnitreeJointController::positionLimits(const std::string & joint_name, float &position)
    {
        urdf::JointConstSharedPtr joint_urdf = model_.getJoint(joint_name);
        if (!joint_urdf)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in URDF model.", joint_name.c_str());
            return;
        }

        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
        {
            clampInPlaceCustom(position, joint_urdf->limits->lower, joint_urdf->limits->upper, joint_name, "position");
        }
    }

    void UnitreeJointController::velocityLimits(const std::string & joint_name, float &velocity)
    {
        urdf::JointConstSharedPtr joint_urdf = model_.getJoint(joint_name);
        if (!joint_urdf)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in URDF model.", joint_name.c_str());
            return;
        }

        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clampInPlaceCustom(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity, joint_name, "velocity");
    }

    void UnitreeJointController::effortLimits(const std::string & joint_name, float &effort)
    {
        urdf::JointConstSharedPtr joint_urdf = model_.getJoint(joint_name);
        if (!joint_urdf)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in URDF model.", joint_name.c_str());
            return;
        }

        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
        {
            // RCLCPP_INFO(get_node()->get_logger(), "Applying effort limits for joint '%s'.", joint_name.c_str());
            // RCLCPP_INFO(get_node()->get_logger(), "Effort limits: %.2f, %.2f",
            //     -joint_urdf->limits->effort, joint_urdf->limits->effort);
            clampInPlaceCustom(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort, joint_name, "effort");
        }
    }    

} // namespace unitree_joint_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_joint_controller::UnitreeJointController, controller_interface::ControllerInterface)
