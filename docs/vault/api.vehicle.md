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
        -  
- We need to know the dynamic properties of the vehicle?
    - E.g. If we want to compute energy cost
    - mass
    - inertia tensor?

## Design options


```cpp
struct Payload {
    VehicleGeometricProperties deployed_payload_geometry;
    bool deployed;
    // payload might have its own state
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

struct Vehicle {
    vector<Payload> payloads;
    void setPayloads();

    VehicleGeometricProperties geometry;
};


Vehicle vehicle;
vehicle.setPayloads()
vehice.payloads[MANIPULATOR]

```
