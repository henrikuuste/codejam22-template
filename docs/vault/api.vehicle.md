## Overview
How do we specify vehicle model/contraints?

## Requirements and use cases
- We need to know the geometric properties of the vehicle
  - W, H, L
  - Changes in geometric properties (eg. payload in some position)
- We need to know the kinematic properties of the vehicle
  - vehicle kinematic constraints
    - Ackermann vs differential drive
    - Max velocity
    - Min turning radius
    - Max acceleration (positive and negative?)

- We need to know the dynamic properties of the vehicle?
  - E.g. If we want to compute energy cost
  - mass
  - inertia tensor?
  - max torque/torque limits

- We need to know the states of all the extra equipment
  - payloads
  - sensors
  - lights

- we need to know the energy balance of the vehicle
  - how much is it gaining/loosing energy
  - battery level/total amount of energy
  - recharge capabilities
## Design options

```cpp
struct Payload {
  VehicleGeometricProperties deployed_payload_geometry;
  bool deployed;
  // payload might have its own state
  Real energy_consumption; // ~kwh - average in deployed state
};

struct VehicleGeometricProperties {
  Real height;
  Real width;
  Real lenght;
};

struct VehicleKinematicProperties {};
struct VehicleDynamicProperties {
  Real mass;
}

struct Sensor {
  bool is_passive;
  bool is_working;
  Real energy_consumption; // ~kwh
};

struct Vehicle {
  vector<Payload> payloads;
  void addPayload();
  void addSensor();

  VehicleGeometricProperties geometry;
  // kinematic properties
  // dynamic properties

  float battery_level; // 0.0 - 100.0 or 0.0 - 1.0
  bool generator_working; 

  Real passive_power_consumption; // kwh/s or kwh/min power expanded from just idling (PC + sensors + lights + etc). This has to be changed only if sensor/lights/etc turned on/off. Total energy loss rate is passive_power_consumption + power consumption from movement
 
  Real power_gain; // kwh/s or kwh/min gained from generator or other sources. Describes the generators capability to provide energy

};


Vehicle vehicle;
Payload manipulator;
Sensor camera;

vehicle.addPayload(manipulator);
vehicle.addSensor(camera);


```
