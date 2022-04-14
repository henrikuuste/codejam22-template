---
id: 3akp1f97ditnk60n0seb19f
title: API
desc: ''
updated: 1649910591922
created: 1649872895087
---
Path planner API trade study for CodeJam22

## Structure and context
#### Dependencies
```mermaid
flowchart TD;
  PP[Path planner]
  CM[Cost map]
  V[Vehicle]
```
#### Usage
```mermaid
sequenceDiagram
  participant PP as Path planner
  participant CM as Cost map
```

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

