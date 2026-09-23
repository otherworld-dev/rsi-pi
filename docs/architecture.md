# Architecture

```
KUKA Robot Controller
        |
     UDP/XML (4ms cycle)
        |
   NetworkProcess          <- multiprocessing.Process, owns the socket
        |
   shared memory           <- send_variables / receive_variables / metrics
        |
     RSIClient             <- orchestrator: config, safety, network
        |
      RSIAPI               <- runs RSIClient in daemon thread
     /  |  \  \
motion  io  krl  safety  monitoring  logging  viz  diagnostics  tools
```

- **NetworkProcess** runs in a separate OS process. It receives XML from the robot, parses it into `send_variables` (what the robot tells us), and builds the response XML from `receive_variables` (what we tell the robot). IPOC synchronization -- echoing the robot's IPOC value unchanged each cycle (the robot advances its own clock) -- is handled automatically.
- **RSIClient** creates the cross-process variable stores, initializes the `ConfigParser` and `SafetyManager`, and manages the network process lifecycle. `send_variables`, `receive_variables` and the metrics are `SharedVariables` stores: a dict interface over shared memory, read without IPC or a lock. There is no `multiprocessing.Manager` process. The network process reads `receive_variables` inside every reply window, where a Manager read cost a pipe round trip per key, up to the whole 4 ms, and it publishes the robot's state after every reply, so the client sees every cycle. Nothing in the loop waits on another process. `diagnostics.get_stats()["snapshot_p99"]` reports what the per-cycle read costs. The network process also watches its parent: if the client is killed, it stops replying and the controller sees a comms loss.
- **RSIAPI** wraps RSIClient in a daemon thread and exposes the namespaced sub-APIs (`motion`, `io`, `krl`, etc.).

The 4 ms cycle is driven by the robot controller, not by RSIPI. If a response is not sent within the cycle window, the robot uses the last held values (for `HOLDON="1"` variables) or drops to zero.
