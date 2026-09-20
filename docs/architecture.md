# Architecture

```
KUKA Robot Controller
        |
     UDP/XML (4ms cycle)
        |
   NetworkProcess          <- multiprocessing.Process, owns the socket
        |
   shared state            <- send_variables (Manager dict),
        |                     receive_variables (shared memory)
     RSIClient             <- orchestrator: config, safety, network
        |
      RSIAPI               <- runs RSIClient in daemon thread
     /  |  \  \
motion  io  krl  safety  monitoring  logging  viz  diagnostics  tools
```

- **NetworkProcess** runs in a separate OS process. It receives XML from the robot, parses it into `send_variables` (what the robot tells us), and builds the response XML from `receive_variables` (what we tell the robot). IPOC synchronization -- echoing the robot's IPOC value unchanged each cycle (the robot advances its own clock) -- is handled automatically.
- **RSIClient** creates the cross-process variable stores, initializes the `ConfigParser` and `SafetyManager`, and manages the network process lifecycle. `send_variables` is a `multiprocessing.Manager` dict, which the network process updates every 10 cycles after a reply has gone. `receive_variables` is a `SharedVariables` store: the same dict interface over one block of shared memory. The network process reads it inside every reply window, and a Manager read there cost a pipe round trip per key, up to the whole 4 ms. Nothing between receiving a packet and sending its reply talks to another process. `diagnostics.get_stats()["snapshot_p99"]` reports what that read costs.
- **RSIAPI** wraps RSIClient in a daemon thread and exposes the namespaced sub-APIs (`motion`, `io`, `krl`, etc.).

The 4 ms cycle is driven by the robot controller, not by RSIPI. If a response is not sent within the cycle window, the robot uses the last held values (for `HOLDON="1"` variables) or drops to zero.
