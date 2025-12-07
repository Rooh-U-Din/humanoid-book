# ROS 2 Service Example: Add Two Ints

This example demonstrates **service-based** communication in ROS 2 - a synchronous request/response pattern.

## What This Example Does

- **Service Server**: Provides `/add_two_ints` service that adds two integers
- **Client**: Sends requests with two numbers and receives the sum

## Prerequisites

Same as talker/listener example - see `examples/ros2/talker_listener/README.md`

## Running the Example

### Terminal 1: Start Service Server

```bash
cd ~/my_book/examples/ros2/add_two_ints
python3 service_server.py
```

**Expected Output:**
```
[INFO] [add_two_ints_server]: Service server started - waiting for requests on /add_two_ints
```

### Terminal 2: Call Service

```bash
# Call service with two integers
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Expected Response:**
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=3)
response:
example_interfaces.srv.AddTwoInts_Response(sum=8)
```

**Server Log:**
```
[INFO] [add_two_ints_server]: Incoming request: 5 + 3 = 8
```

## Difference from Topics

| Topics (Pub/Sub) | Services (Req/Res) |
|------------------|-------------------|
| Asynchronous | Synchronous |
| Many-to-many | One-to-one |
| Continuous data stream | One-time computation |
| Best for: sensor data | Best for: configuration, queries |

## Next Steps

- Modify to multiply instead of add
- Create a Python client
- Try action examples (navigation, manipulation)

---

**License**: MIT
