# RSIPI: Robot Sensor Interface - Python Integration

RSIPI is a high-performance, Python-based communication and control system designed for real-time interfacing with KUKA robots using the Robot Sensor Interface (RSI) protocol. It provides both a robust **API** for developers and a powerful **Command Line Interface (CLI)** for researchers and engineers who need to monitor, control, and analyse robotic movements in real time.

---

🛡️ Safety Notice
RSIPI is a powerful tool that directly interfaces with industrial robotic systems. Improper use can lead to dangerous movements, property damage, or personal injury.

⚠️ Safety Guidelines
- **Test in Simulation First:** Always verify your RSI communication and trajectories using simulation tools before deploying to a live robot.
- **Enable Emergency Stops:** Ensure all safety hardware (E-Stop, fencing, light curtains) is active and functioning correctly.
- **Supervised Operation Only:** Run RSIPI only in supervised environments with trained personnel present.
- **Limit Movement Ranges:** Use KUKA Workspaces or software limits to constrain movement, especially when testing new code.
- **Use Logging for Debugging:** Avoid debugging while RSI is active; instead, enable CSV logging and review logs post-run.
- **Secure Network Configuration:** Ensure your RSI network is on a closed, isolated interface to avoid external interference or spoofing.
- **Never Rely on RSIPI for Safety:** RSIPI is not a safety-rated system. Do not use it in applications where failure could result in harm.

---

## 📄 Description

RSIPI allows users to:
- Communicate with KUKA robots using the RSI XML-based protocol.
- Dynamically update control variables (TCP position, joint angles, I/O, external axes, etc.).
- Log and visualise robot movements with live graphs and static plots.
- Analyse motion data and compare planned vs actual trajectories.
- Parse and inject RSI into KRL programs.
- Simulate robot behaviour using a realistic Echo Server.
- Enforce safety limits and manage emergency stops.

### Target Audience
- **Researchers** working on advanced robotic applications, control algorithms, and feedback systems.
- **Engineers** developing robotic workflows or automated processes.
- **Educators** using real robots in coursework or lab environments.
- **Students** learning about robot control systems and data-driven motion planning.

---

## 📊 Features
- Real-time network communication with KUKA RSI over UDP.
- Structured logging to CSV with British date formatting.
- Background execution and live variable updates.
- Fully-featured Python API for scripting or external integration.
- CLI for interactive control, trajectory planning, and live monitoring.
- Real-time and post-analysis graphing (live TCP, joints, force, acceleration).
- Safety management: emergency stop, limit enforcement, safety override.
- KUKA KRL `.src/.dat` parsing and RSI injection tools.
- Echo Server and GUI for offline simulation and testing.
- Deviation and force spike alerts during live operation.

---

## 📊 API Overview (`rsi_api.py`)

### Initialization
```python
from src.RSIPI import rsi_api
api = rsi_api.RSIAPI(config_file='examples/RSI_EthernetConfig.xml')
```

### Selected Methods
| Method | CLI | API | Description |
|--------|-----|-----|-------------|
| `start_rsi()` | ✅ | ✅ | Starts RSI communication (non-blocking). |
| `stop_rsi()` | ✅ | ✅ | Stops RSI communication. |
| `update_variable(path, value)` | ✅ | ✅ | Dynamically updates a send variable (e.g. `RKorr.X`). |
| `get_variable(path)` | ✅ | ✅ | Retrieves the latest value of any variable. |
| `plan_linear_cartesian(start, end, steps)` | ✅ | ✅ | Create Cartesian paths. |
| `plan_linear_joint(start, end, steps)` | ✅ | ✅ | Create Joint-space paths. |
| `execute_trajectory(traj, rate)` | ✅ | ✅ | Execute planned trajectory live. |
| `enable_alerts(True/False)` | ✅ | ✅ | Enable or disable deviation/force alerts. |
| `start_live_plot(mode)` | ✅ | ✅ | Live graph position, velocity, force, etc. |
| `generate_plot(csv, type)` | ✅ | ✅ | Static graphing from CSV files. |
| `export_movement_data(filename)` | ✅ | ✅ | Export recorded motion as CSV. |
| `parse_krl_to_csv(src, dat, output)` | ✅ | ✅ | Extract TCP points from KRL programs. |
| `inject_rsi(input, output, config)` | ✅ | ✅ | Add RSI startup code to a KRL file. |

_(Full API details available in `rsi_api.py`.)_

---

## 🔧 CLI Overview (`rsi_cli.py`)

Start the CLI:
```bash
python main.py --cli
```

### Selected Commands
| Command | Description |
|---------|-------------|
| `start` / `stop` | Start or stop RSI client. |
| `set <var> <value>` | Update send variable. |
| `get <var>` | Get latest receive variable. |
| `move_cartesian`, `move_joint` | Move robot using planned trajectories. |
| `queue_cartesian`, `queue_joint` | Queue trajectory steps. |
| `execute_queue` | Run queued trajectories. |
| `alerts on/off` | Enable or disable alerts. |
| `graph show/compare` | Plot or compare test runs. |
| `log start/stop/status` | Manage CSV logging. |
| `plot <type> <csv>` | Static plotting (position, velocity, deviation, etc.). |
| `safety-stop`, `safety-reset`, `safety-status` | Emergency stop and limit management. |
| `krlparse <src> <dat> <out>` | Parse KRL to CSV. |
| `inject_rsi <src> [out] [config]` | Inject RSI code into KRL file. |

---

## 📃 Example Usage

### Update TCP position live
```python
api.start_rsi()
api.update_variable('RKorr.X', 100.0)
api.update_variable('RKorr.Y', 50.0)
```

### Plan and execute a Cartesian move
```python
start_pose = {'X': 0, 'Y': 0, 'Z': 500}
end_pose = {'X': 200, 'Y': 0, 'Z': 500}
traj = api.plan_linear_cartesian(start_pose, end_pose, steps=100)
api.execute_trajectory(traj, rate=0.012)
```

### CLI session sample
```bash
> start
> set RKorr.X 150
> move_cartesian X=0,Y=0,Z=500 X=200,Y=0,Z=500 steps=100 rate=0.012
> graph show my_log.csv
> log start
> stop
```

---

## 📅 Output and Logs
- CSV logs saved to `logs/` folder.
- Each log includes British timestamp, sent/received variables.
- Static plots exportable as PNG/PDF.
- Live plots include alert messages and deviation tracking.

---

## 🚀 Getting Started
1. Connect robot and PC via Ethernet.
2. Deploy KUKA RSI program with matching config.
3. Install Python dependencies:
```bash
pip install -r requirements.txt
```
4. Run `main.py` or import `RSIAPI` in your Python scripts.

---

## 🔖 Citation
If you use RSIPI in your research, please cite:
```bibtex
@software{rsipi2025,
  author = {RSIPI Development Team},
  title = {RSIPI: Robot Sensor Interface - Python Integration},
  year = {2025},
  url = {https://github.com/your-org/rsipi},
  note = {Accessed: [insert date]}
}
```

---

## ⚖️ License
RSIPI is licensed under the MIT License.

---

## 🚧 Disclaimer
RSIPI is intended for research and experimental purposes only. Always ensure safe operation with appropriate safety measures in place.

