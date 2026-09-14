# Architecture

```
KUKA Robot Controller
        |
     UDP/XML (4ms cycle)
        |
   NetworkProcess          <- multiprocessing.Process, owns the socket
        |
   Manager().dict()        <- shared send_variables / receive_variables
        |
     RSIClient             <- orchestrator: config, safety, network
        |
      RSIAPI               <- runs RSIClient in daemon thread
     /  |  \  \
motion  io  krl  safety  monitoring  logging  viz  diagnostics  tools
```

- **NetworkProcess** runs in a separate OS process. It receives XML from the robot, parses it into `send_variables` (what the robot tells us), and builds the response XML from `receive_variables` (what we tell the robot). IPOC synchronization -- echoing the robot's IPOC value unchanged each cycle (the robot advances its own clock) -- is handled automatically.
- **RSIClient** creates the `multiprocessing.Manager` dicts for cross-process variable sharing, initializes the `ConfigParser` and `SafetyManager`, and manages the network process lifecycle.
- **RSIAPI** wraps RSIClient in a daemon thread and exposes the namespaced sub-APIs (`motion`, `io`, `krl`, etc.).

The 4 ms cycle is driven by the robot controller, not by RSIPI. If a response is not sent within the cycle window, the robot uses the last held values (for `HOLDON="1"` variables) or drops to zero.
