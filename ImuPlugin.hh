#ifndef _GAZEBO_IMU_PLUGIN_HH_
#define _GAZEBO_IMU_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief A plugin for a IMU sensor.
  class ImuPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ImuPlugin();

    /// \brief Destructor.
    public: virtual ~ImuPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the IMU sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the IMU sensor
    private: sensors::ImuSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the IMU sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    public: transport::PublisherPtr imuPubNoisy, imuPubTrue;
    public: transport::NodePtr node;
  };
}
#endif
