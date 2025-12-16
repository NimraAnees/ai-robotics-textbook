# ROS 2 Architecture Diagrams

This document contains ASCII diagrams and descriptions of the ROS 2 architecture to help visualize the concepts.

## Basic Node Architecture

```
┌─────────────────┐
│   ROS 2 Node    │
├─────────────────┤
│  Node Class     │
│  ┌─────────────┐ │
│  │   Node      │ │
│  │  Instance   │ │
│  └─────────────┘ │
│                 │
│  Publishers     │
│  ┌─────────────┐ │
│  │ Publisher 1 │ │─ ─ ─ ─ ─ ┐
│  ├─────────────┤ │           │
│  │ Publisher 2 │ │─ ─ ─ ─ ─ ─┼─ Topic A
│  ├─────────────┤ │           │
│  │ Publisher N │ │─ ─ ─ ─ ─ ┘
│  └─────────────┘ │
│                 │
│  Subscribers    │
│  ┌─────────────┐ │
│  │ Subscriber  │ │─ ─ ─ ─ ─ ┐
│  │     1       │ │           │
│  ├─────────────┤ │           ├─ Topic B
│  │ Subscriber  │ │           │
│  │     2       │ │─ ─ ─ ─ ─ ─┤
│  ├─────────────┤ │           │
│  │ Subscriber  │ │           ├─ Topic C
│  │     N       │ │─ ─ ─ ─ ─ ┘
│  └─────────────┘ │
│                 │
│  Services/Actions│
│  ┌─────────────┐ │
│  │ Service Srv │ │─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
│  ├─────────────┤ │                   │
│  │ Service Cli │ │─ ─ ─ ─ ─ ─ ─ ─ ─ ─┼─ Service Call
│  ├─────────────┤ │                   │
│  │ Action Srv  │ │─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
│  └─────────────┘ │
└─────────────────┘
```

## Distributed System Architecture

```
         Network/DDS Layer
    ┌─────────────────────────┐
    │      DDS/RMW Layer      │
    └─────────────────────────┘
              │ │ │
    ┌─────────┘ │ └─────────┐
    │                       │
┌───▼────────┐    ┌────────▼────────┐
│            │    │               │
│  Node A    │    │    Node B     │
│            │    │               │
├────────────┤    ├───────────────┤
│Publisher   │    │Subscriber     │
│─ ─ ─ ─ ─ ─▶│    │◀─ ─ ─ ─ ─ ─ ─ │
│            │    │               │
│Subscriber ││    │Publisher      │
│◀─ ─ ─ ─ ─ ┼┤    │─ ─ ─ ─ ─ ─ ─ ▶│
│            │    │               │
└────────────┘    └───────────────┘
        │                      │
        └──────────────────────┘
                Topic X
```

## Publisher-Subscriber Pattern

```
┌─────────────┐
│   Node A    │
│             │
│  Publisher  │
│     │       │
│     ▼       │
│  ┌─────┐    │
│  │ Msg │    │
│  └─────┘    │
└─────────────┘
       │
       │ Topic: /sensor_data
       │
┌─────────────┐
│   Node B    │
│             │
│  Subscriber │
│     ▲       │
│  ┌─────┐    │
│  │ Msg │    │
│  └─────┘    │
└─────────────┘
```

## Service Client-Server Pattern

```
┌─────────────┐                    ┌─────────────┐
│   Client    │                    │   Server    │
│             │                    │             │
│ Service     │                    │ Service     │
│ Request     │                    │ Response    │
│     │       │     Service Call   │       │     │
│     ▼       │    ┌ ─ ─ ─ ─ ─ ─   │       ▼     │
│  ┌─────┐    │    │  Service    │  │    ┌─────┐  │
│  │Req│  │    │ ◄──┼─ ─ ─ ─ ─ ─ ─┼──►  │Resp│ │  │
│  └─────┘    │    │  Interface  │  │    └─────┘  │
│             │    └ ─ ─ ─ ─ ─ ─   │             │
└─────────────┘                    └─────────────┘
```

## Action Client-Server Pattern

```
┌─────────────┐                           ┌─────────────┐
│   Client    │                           │   Server    │
│             │                           │             │
│ Action      │                           │ Action      │
│ Goal        │                           │ Result/     │
│     │       │                           │ Feedback    │
│     ▼       │                           │       │     │
│  ┌─────┐    │    Action Goal            │       ▼     │
│  │Goal │    │   ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─    │    ┌─────┐  │
│  └─────┘    │◄──┼─ ─ ─ ─ ─ ─ ─ ─ ─ ─┼───►│Resp │ │  │
│             │   │ Action Interface  │    │     │  │
│ Action      │   └ ─ ─ ─ ─ ─ ─ ─ ─ ─    │    │Feed │  │
│ Result/     │                           │    │back │  │
│ Feedback    │                           │    └─────┘  │
│     ▲       │                           │       │     │
│  ┌─────┐    │    Action Result/         │       ▼     │
│  │Res││    │◄──┼─ ─ ─ ─ ─ ─ ─ ─ ─ ─    │    ┌─────┐  │
│  │ult│    │   │ Feedback Interface │    │    │Proc │  │
│  └─────┘    │   └ ─ ─ ─ ─ ─ ─ ─ ─ ─    │    │ess│  │
└─────────────┘                           │    │ing│  │
                                          │    └─────┘  │
                                          └─────────────┘
```

## Complete System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 System                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│  │  Node   │    │  Node   │    │  Node   │             │
│  │   A     │    │   B     │    │   C     │             │
│  └─────────┘    └─────────┘    └─────────┘             │
│      │              │              │                   │
│      │              │              │                   │
│  ┌───▼───┐      ┌───▼───┐      ┌───▼───┐             │
│  │Topic  │◄─────┤Topic  │─────►│Topic  │             │
│  │Pub/Sub│      │Pub/Sub│      │Pub/Sub│             │
│  └───────┘      └───────┘      └───────┘             │
│                                                         │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│  │Service  │    │Service  │    │Service  │             │
│  │Client/  │    │Client/  │    │Client/  │             │
│  │Server   │    │Server   │    │Server   │             │
│  └─────────┘    └─────────┘    └─────────┘             │
│                                                         │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│  │Action   │    │Action   │    │Action   │             │
│  │Client/  │    │Client/  │    │Client/  │             │
│  │Server   │    │Server   │    │Server   │             │
│  └─────────┘    └─────────┘    └─────────┘             │
│                                                         │
└─────────────────────────────────────────────────────────┘
                          │
                    ┌─────▼─────┐
                    │  DDS/RMW  │
                    │ Middleware│
                    └───────────┘
```

## Quality of Service (QoS) Settings

```
┌─────────────────────────────────────────────────────────┐
│              QoS Policy Configuration                   │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Publisher                 Subscriber                   │
│  ┌─────────────────┐    ┌─────────────────┐            │
│  │  QoS Settings   │    │  QoS Settings   │            │
│  │                 │    │                 │            │
│  │ • Reliability   │    │ • Reliability   │            │
│  │ • Durability    │    │ • Durability    │            │
│  │ • History       │    │ • History       │            │
│  │ • Depth         │    │ • Depth         │            │
│  └─────────────────┘    └─────────────────┘            │
│                                                         │
│        Topic Connection (Based on Matching QoS)         │
│  ┌─────────────────────────────────────────────────┐    │
│  │   Match: Both use 'reliable' and 'transient'   │    │
│  │   Result: Guaranteed delivery of historical data│    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

These diagrams illustrate the key architectural concepts of ROS 2, including the distributed nature of nodes, the various communication patterns, and how Quality of Service settings affect communication.