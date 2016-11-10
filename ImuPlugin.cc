#include "ImuPlugin.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

/////////////////////////////////////////////////
ImuPlugin::ImuPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ImuPlugin::~ImuPlugin()
{
}

/////////////////////////////////////////////////
void ImuPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ImuPlugin requires a ImuSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ImuPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  //node_noisy = transport::NodePtr(new transport::Node());
  //node_noisy->Init();
  node = transport::NodePtr(new transport::Node());
  node->Init();
  imuPubNoisy = node->Advertise<msgs::IMU>("~/imu_plugin/noisy");
  imuPubTrue = node->Advertise<msgs::IMU>("~/imu_plugin/true");
}

/////////////////////////////////////////////////
void ImuPlugin::OnUpdate()
{
  ignition::math::Vector3d 	true_acceleration, noisy_acceleration;
  true_acceleration = this->parentSensor->LinearAcceleration(true);
  noisy_acceleration = this->parentSensor->LinearAcceleration(false);
  /*
  std::cout << "Linear Acceleration without noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << true_acceleration[i] << " ";
  }
  std::cout << "\n";

  std::cout << "Linear Acceleration with noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << noisy_acceleration[i] << " ";
  }
  std::cout << "\n";
  */
  
  ignition::math::Vector3d 	true_velocity, noisy_velocity;
  true_velocity = this->parentSensor->AngularVelocity(true);
  noisy_velocity = this->parentSensor->AngularVelocity(false);
  /*
  std::cout << "Angular Velocity without noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << true_velocity[i] << " ";
  }
  std::cout << "\n";

  std::cout << "Angular Velocity with noise:";
  for (unsigned int i = 0; i < 3; ++i)
  {
    std::cout << noisy_velocity[i] << " ";
  }
  std::cout << "\n";
  */

  msgs::IMU msg1;
  msgs::Set(msg1.mutable_stamp(), this->parentSensor->LastMeasurementTime());
  msg1.set_entity_name(this->parentSensor->ParentName());
  msgs::Set(msg1.mutable_orientation(), this->parentSensor->Orientation());
  msgs::Set(msg1.mutable_angular_velocity(), this->parentSensor->AngularVelocity(false));
  msgs::Set(msg1.mutable_linear_acceleration(), this->parentSensor->LinearAcceleration(false));

  msgs::IMU msg2;
  msgs::Set(msg2.mutable_stamp(), this->parentSensor->LastMeasurementTime());
  msg2.set_entity_name(this->parentSensor->ParentName());
  msgs::Set(msg2.mutable_orientation(), this->parentSensor->Orientation());
  msgs::Set(msg2.mutable_angular_velocity(), this->parentSensor->AngularVelocity(true));
  msgs::Set(msg2.mutable_linear_acceleration(), this->parentSensor->LinearAcceleration(true));

  imuPubNoisy->Publish(msg1);
  imuPubTrue->Publish(msg2);

}
