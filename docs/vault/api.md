---
id: 3akp1f97ditnk60n0seb19f
title: API
desc: ''
updated: 1650543198622
created: 1649872895087
---
Path planner API trade study for CodeJam22
## Domain definition
* For **UGV** (not UxV)
  * Vehicles with different dimensions
    * May have different wheelbases and clearance
  * May or may not have independent control over rotation (differential vs Ackermann)
  * Has no direct control over elevation
    * Changes in 2D position affect elevation
  * Controllable payloads (turrets, masts, ISR, etc.) and platform capabilities (generator, lights etc.)
* Path planner will be **invoked by the vehicle in real-time**
  * At multiple resolution levels/scales (soft real-time)
    * Hierarchical iterative process
  * Some of the small scale invocations will have hard real-time constraints (obstacle avoidance)
* Path planner will be **invoked by a mission solver**
  * On C2, simulation or on the vehicle
  * Probably invoked 1000+ times per solver run
* Path planner will be given an ordered set of goals
  * The path planner **does not change the number or order of goals**

### [[use-cases]]

## Structure and context
#### Dependencies
```mermaid
flowchart TD;
  PP[Path planner]
  CM[Cost provider]
  V[Vehicle]
  EDB[Environment DB]
  PF[Path following]
  S[Search space]
  CM-- provides cost -->PP
  CM-- vehicle data? -->PP
  CM-- search space -->PP
  PP-->PF
  EDB-- provides data -->CM
  V-->S
  S-->CM
```
#### Usage
```mermaid
sequenceDiagram
  participant U as User
  participant PP as Path planner
  participant CM as Cost provider
  participant S as Search space
  participant EDB as Environment DB
  S ->> CM: search space spec
  U ->> PP: setCostProvider
  CM ->> PP: search space spec
  U ->> PP: plan(initial state, waypoints...)
  activate PP
  S ->> PP: update space
  PP ->> CM: lock
  loop every search iteration
    PP ->> CM: cost(???)
    activate CM
    CM ->> EDB: query data
    CM -->> PP: cost or error
    deactivate CM
  end
  PP ->> CM: release
  PP -->> U: path or error
  U ->> PP: getDiagnostics()
  deactivate PP
```

## Tests
* [[tests.unit]]
* [[tests.scenarios]]
* [[tests.proposals]]

## API studies
* [[api.planner]]
  * [📝Proposed planner API](assets/proposals.planner.hpp)
* [[api.cost]]
  * [📝Proposed cost API](assets/proposals.cost.hpp)
  * [[api.cost.access]]
    * [📝Proposed cost access API](assets/proposals.cost.access.hpp)
* [[api.search-space]]
  * [📝 Proposed search space API](assets/proposals.search.space.hpp)
  * [[api.search-space.state]]
  * [[api.search-space.target]]
  * [[api.search-space.path]]
* [[api.vehicle]]
  * [📝Proposed vehicle API](assets/proposals.vehicle.hpp)
* [[api.diagnostics]]

