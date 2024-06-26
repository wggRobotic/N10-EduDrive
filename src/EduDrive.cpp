#include "EduDrive.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include <fcntl.h>
#include <geometry_msgs/msg/detail/accel_stamped__struct.hpp>
#include <linux/gpio.h>
#include <sys/ioctl.h>


namespace edu
{

    EduDrive::EduDrive() : Node("n10_edu_drive_node")
    {

    }

    EduDrive::~EduDrive()
    {
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            delete *it;
        }
        delete _pwr_mgmt;
        delete _adapter;

    }

    void EduDrive::initDrive(std::vector<ControllerParams> cp, SocketCAN &can, bool using_pwr_mgmt, bool verbosity)
    {
        _can = &can;

        _using_pwr_mgmt = using_pwr_mgmt;
        _verbosity = verbosity;
        _enabled = false;
        
        _subJoy     = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&EduDrive::joyCallback, this, std::placeholders::_1));
        _subVel     = this->create_subscription<geometry_msgs::msg::Twist>("vel/teleop", 10, std::bind(&EduDrive::velocityCallback, this, std::placeholders::_1));
        _subMotorVel= this->create_subscription<std_msgs::msg::Float32MultiArray>("/n10/motor_vel", 10, std::bind(&EduDrive::controlMotorsIndividually, this, std::placeholders::_1));
        _srvEnable  = this->create_service<std_srvs::srv::SetBool>("eduard/enable", std::bind(&EduDrive::enableCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publisher of motor shields
        _pubEnabled = this->create_publisher<std_msgs::msg::ByteMultiArray>("enabled", 1);
        _pubRPM     = this->create_publisher<std_msgs::msg::Float32MultiArray>("rpm", 1);

        // Publisher of carrier shield
        _pubTemp             = this->create_publisher<std_msgs::msg::Float32>("temperature", 1);
        _pubVoltageAdapter   = this->create_publisher<std_msgs::msg::Float32>("voltageAdapter", 1);
        _pubOrientation      = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
        _pubAccel            = this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);
		
        //Publisher of power management shield
        _pubVoltagePwrMgmt = this->create_publisher<std_msgs::msg::Float32>("voltagePwrMgmt", 1);
        _pubCurrentPwrMgmt = this->create_publisher<std_msgs::msg::Float32>("currentPwrMgmt", 1);

        _adapter = new RPiAdapterBoard(&can, verbosity);
        _pwr_mgmt = new PowerManagementBoard(&can, verbosity);
        
        _vMax = 0.f;

        bool isKinematicsValid = true;
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            std::vector<MotorParams> motorParams = cp[i].motorParams;

            for (unsigned int j = 0; j < motorParams.size(); ++j)
            {
                isKinematicsValid &= (motorParams[j].kinematics.size()==3);
				}
        }
        if(!isKinematicsValid)
        {
            //std::cout << "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected." << std::endl;
            RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected.");

            exit(1);
        }
        
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            _mc.push_back(new MotorController(&can, cp[i], verbosity));
            
            for(unsigned int j=0; j<_mc[i]->getMotorParams().size(); j++)
            {
            	std::vector<double> kinematics = _mc[i]->getMotorParams()[j].kinematics;
	         	double kx = kinematics[0];
	         	double kw = kinematics[2];
	         	if(fabs(kx)>1e-3)
	         	{
	            	float vMax = fabs(cp[i].rpmMax / 60.f * (2.f * M_PI) / kx);
	            	if(vMax > _vMax) _vMax = vMax;
	            }
	            if(fabs(kw)>1e-3)
	            {
	            	float omegaMax = fabs(cp[i].rpmMax / 60.f * (2.f * M_PI) / kw);
	            	if(omegaMax > _omegaMax) _omegaMax = omegaMax;
		         }
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Instanciated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");
    }

    void EduDrive::run()
    {
        _lastCmd = this->get_clock()->now();

        rclcpp::TimerBase::SharedPtr timerReceiveCAN = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&EduDrive::receiveCAN, this));
        rclcpp::TimerBase::SharedPtr timerCheckLaggyConnection = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EduDrive::checkLaggyConnection, this));

        rclcpp::spin(shared_from_this());

        _can->clearObservers();
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            (*it)->stop();
            (*it)->disable();
        }

        rclcpp::shutdown();
    }

    void EduDrive::enable()
    {
        RCLCPP_INFO(this->get_logger(), "Enabling robot");

        if(_using_pwr_mgmt){
            // Let power management board set hardware enable
            // if the power management board is not used, the user needs to take care about this pin.
            // The adapter board is designed to treat this pin as a input pin
            _pwr_mgmt->enable();
        }

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            if(!(*it)->isInitialized())
                (*it)->reinit();
            (*it)->enable();
        }
    }

    void EduDrive::disable()
    {
        RCLCPP_INFO(this->get_logger(), "Disabling robot");

        if(_using_pwr_mgmt){
            // Let power management board reset hardware enable
            _pwr_mgmt->disable();
        }

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
            (*it)->disable();
    }

    void EduDrive::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        // Assignment of joystick axes to motor commands
        float fwd = joy->axes[1];                    // Range of values [-1:1]
        float left = joy->axes[0];                   // Range of values [-1:1]
        float turn = joy->axes[2];                   // Range of values [-1:1]
        float throttle = (joy->axes[3] + 1.0) / 2.0; // Range of values [0:1]

        // Enable movement in the direction of the y-axis only when the button 12 is pressed
        if (!joy->buttons[11])
            left = 0;

        static int32_t btn9Prev = joy->buttons[9];
        static int32_t btn10Prev = joy->buttons[10];

        if (joy->buttons[9] && !btn9Prev)
        {
            disable();
        }
        else if (joy->buttons[10] && !btn10Prev)
        {
            enable();
        }

        btn9Prev = joy->buttons[9];
        btn10Prev = joy->buttons[10];

        float vFwd = throttle * fwd * _vMax;
        float vLeft = throttle * left * _vMax;
        float omega = throttle * turn * _omegaMax;

        controlMotors(vFwd, vLeft, omega);
    }

    void EduDrive::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
    }

    bool EduDrive::enableCallback(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request, const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
    {
        // suppress warning about unused variable header
        (void)header;

       if(request->data==true)
       {
           RCLCPP_INFO(this->get_logger(), "%s", "Enabling robot");
           enable();
       }
       else
       {
           RCLCPP_INFO(this->get_logger(),  "%s", "Disabling robot");
           disable();
       }
       response->success = true;
       return true;
    }

    void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
    {
        _lastCmd = this->get_clock()->now();
            
        for (unsigned int i = 0; i < _mc.size(); ++i)
        {
            std::vector<double> kinematics0 = _mc[i]->getMotorParams()[0].kinematics;
            std::vector<double> kinematics1 = _mc[i]->getMotorParams()[1].kinematics;
            float w[2];
            w[0] = kinematics0[0] * vFwd + kinematics0[1] * vLeft + kinematics0[2] * omega;
            w[1] = kinematics1[0] * vFwd + kinematics1[1] * vLeft + kinematics1[2] * omega;

            // Convert from rad/s to rpm
            w[0] *= 60.f / (2.f * M_PI);
            w[1] *= 60.f / (2.f * M_PI);
            _mc[i]->setRPM(w);
            
            if (_verbosity)
                //std::cout << "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1] << std::endl;
                RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1]);
        }
    }

    void EduDrive::controlMotorsIndividually(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

      _lastCmd = this->get_clock()->now();

      if(_mc.size() == 3) {
        
        float wfront[2] = {msg->data[0], msg->data[1]};
        float wmiddle[2] = {msg->data[2], msg->data[3]};
        float wback[2] = {msg->data[4], msg->data[5]};

        _mc[0]->setRPM(wfront);
        _mc[1]->setRPM(wmiddle);
        _mc[2]->setRPM(wback);

        if (_verbosity) {
          RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << 0 << " to " << wfront[0] << " " << wfront[1]);
          RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << 1 << " to " << wmiddle[0] << " " << wmiddle[1]);
          RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << 2 << " to " << wback[0] << " " << wback[1]);
        }
      }

    }

    void EduDrive::receiveCAN()
    {
        float voltageAdapter = _adapter->getVoltageSys();
        float voltagePwrMgmt = _pwr_mgmt->getVoltage();
        
        std_msgs::msg::Float32MultiArray msgRPM;
        std_msgs::msg::ByteMultiArray msgEnabled;

        bool controllersInitialized = true;
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {        
	        controllersInitialized = controllersInitialized && (*it)->isInitialized();
	     }
        
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            float response[2] = {0, 0};
            bool enableState = false;
            if(controllersInitialized)
            {
                if(voltageAdapter > 3.0 || voltagePwrMgmt > 3.0) //@ToDo: find nicer solution
                {                    
                    if((*it)->checkConnectionStatus(200))
                    {
                        (*it)->getWheelResponse(response);
                        enableState = (*it)->getEnableState();
                    }
                    else
                    {
                        //std::cout << "#EduDrive Error synchronizing with device" << (*it)->getCanId() << std::endl;
                        RCLCPP_WARN_STREAM(this->get_logger(), "#EduDrive Error synchronizing with device" << (*it)->getCanId());   
                    }
                }
                else
                {
                    //std::cout << "#EduDrive Low voltage on drive power supply rail for device " << (*it)->getCanId() << std::endl;
                    RCLCPP_WARN_STREAM(this->get_logger(), "#EduDrive Low voltage on drive power supply rail for device " << (*it)->getCanId());
                    
                    (*it)->deinit();
                    disable();
                }
            }
            msgRPM.data.push_back(response[0]);
            msgRPM.data.push_back(response[1]);
            msgEnabled.data.push_back(enableState);
        }
        
        rclcpp::Time stampReceived = this->get_clock()->now();

        _enabled = false;
        if(msgEnabled.data.size()>0)
        {
            _enabled = msgEnabled.data[0];
            for(unsigned int i=1; i<msgEnabled.data.size(); i++)
            {
                _enabled &= msgEnabled.data[i];
            }
        }

        _pubRPM->publish(msgRPM);
        _pubEnabled->publish(msgEnabled);

        std_msgs::msg::Float32 msgTemperature;
        msgTemperature.data = _adapter->getTemperature();
        _pubTemp->publish(msgTemperature);

        std_msgs::msg::Float32 msgVoltageAdapter;
        msgVoltageAdapter.data = _adapter->getVoltageSys();
        _pubVoltageAdapter->publish(msgVoltageAdapter);

        double q[4];
        _adapter->getOrientation(q);
        geometry_msgs::msg::PoseStamped msgOrientation;
        //Sequence number not supported in std_msgs::msg::header in ros2
        //static unsigned int seq = 0;
        //msgOrientation.header.seq = seq++;
        msgOrientation.header.stamp = stampReceived;
        msgOrientation.header.frame_id = "base_link";
        msgOrientation.pose.position.x = 0;
        msgOrientation.pose.position.y = 0;
        msgOrientation.pose.position.z = 0;
        msgOrientation.pose.orientation.w = q[0];
        msgOrientation.pose.orientation.x = q[1];
        msgOrientation.pose.orientation.y = q[2];
        msgOrientation.pose.orientation.z = q[3];
        _pubOrientation->publish(msgOrientation);

        double a[3];
        _adapter->getAcceleration(a);
        geometry_msgs::msg::AccelStamped msgAccel;
        msgAccel.header.stamp = msgOrientation.header.stamp;
        msgAccel.header.frame_id = msgOrientation.header.frame_id;
        msgAccel.accel.linear.x = a[0];
        msgAccel.accel.linear.y = a[1];
        msgAccel.accel.linear.z = a[2];
        _pubAccel->publish(msgAccel);

        std_msgs::msg::Float32 msgVoltagePwrMgmt;
        msgVoltagePwrMgmt.data = voltagePwrMgmt;
        _pubVoltagePwrMgmt->publish(msgVoltagePwrMgmt);

        std_msgs::msg::Float32 msgCurrentPwrMgmt;
        msgCurrentPwrMgmt.data = _pwr_mgmt->getCurrent();
        _pubCurrentPwrMgmt->publish(msgCurrentPwrMgmt);
    }

    void EduDrive::checkLaggyConnection()
    {
        rclcpp::Duration dt = this->get_clock()->now() - _lastCmd;
        bool lag = (dt.seconds() > 0.5);
        if(lag  && _enabled)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Lag detected ... deactivate motor control");
            disable();
        }
    }

    int EduDrive::gpio_write(const char *dev_name, int offset, int value)
    {
        struct gpiohandle_request rq;
        struct gpiohandle_data data;
        int fd, ret;
        
        fd = open(dev_name, O_RDONLY);
        if (fd < 0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unabled to open " << dev_name << ": " << strerror(errno) << std::endl);
            return -1;
        }
        rq.lineoffsets[0] = offset;
        rq.flags = GPIOHANDLE_REQUEST_OUTPUT;
        rq.lines = 1;
        ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
        close(fd);
        if (ret == -1)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to line handle from ioctl: " << strerror(errno) << std::endl);
            return -1;
        }
        data.values[0] = value;
        ret = ioctl(rq.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        if (ret == -1)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to set line value using ioctl: " << strerror(errno) << std::endl);
            return -1;
        }else{
            if(_verbosity) std::cout << "Wrote value " << value << " to GPIO at offset " << offset << " (OUTPUT mode) on chip " << dev_name << std::endl;
        }

        close(rq.fd);
        return 1;
    }

    int EduDrive::gpio_read(const char *dev_name, int offset, int &value)
    {
        struct gpiohandle_request rq;
        struct gpiohandle_data data;
        int fd, ret;
        fd = open(dev_name, O_RDONLY);
        if (fd < 0)
        {
             RCLCPP_WARN_STREAM(this->get_logger(), "Unabled to open " << dev_name << ", " << strerror(errno) << std::endl);
            return -1;
        }
        rq.lineoffsets[0] = offset;
        rq.flags = GPIOHANDLE_REQUEST_INPUT;
        rq.lines = 1;
        ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
        close(fd);
        if (ret == -1)
        {
             RCLCPP_WARN_STREAM(this->get_logger(), "Unable to get line handle from ioctl : " << strerror(errno) << std::endl);
            return -1;
        }
        ret = ioctl(rq.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
        if (ret == -1)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to get line value using ioctl : " << strerror(errno) << std::endl);
            return -1;
        }
        else
        {
            if(_verbosity) std::cout << "Value of GPIO at offset " << offset << " (INPUT mode) on chip " << dev_name << ": " << data.values[0] << std::endl;
        }

        close(rq.fd);
        value = data.values[0];
        return 1;    
    }

} // namespace
