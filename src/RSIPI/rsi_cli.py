from RSIPI.rsi_api import RSIAPI

class RSICommandLineInterface:
    """Command-Line Interface for controlling RSI Client."""

    def __init__(self, input_config_file):
        self.client = RSIAPI(input_config_file)
        self.running = True

    def run(self):
        print("RSI Command-Line Interface Started. Type 'help' for commands.")
        while self.running:
            try:
                command = input("RSI> ").strip()
                self.process_command(command)
            except KeyboardInterrupt:
                self.exit()

    def process_command(self, command):
        parts = command.split()
        if not parts:
            return

        cmd = parts[0].lower()
        args = parts[1:]

        try:
            match cmd:
                case "start":
                    print(self.client.start_rsi())
                case "stop":
                    print(self.client.stop_rsi())
                case "exit":
                    self.exit()
                case "set":
                    var, val = args[0], args[1]
                    print(self.client.update_variable(var, val))
                case "show":
                    print("📤 Send Variables:")
                    self.client.show_variables()
                case "reset":
                    print(self.client.reset_variables())
                case "status":
                    print(self.client.show_config_file())
                case "ipoc":
                    print(f"🛰 IPOC: {self.client.get_ipoc()}")
                case "watch":
                    duration = float(args[0]) if args else None
                    self.client.watch_network(duration)
                case "reconnect":
                    print(self.client.reconnect())
                case "alerts":
                    state = args[0].lower()
                    self.client.enable_alerts(state == "on")
                case "set_alert_threshold":
                    alert_type, value = args[0], float(args[1])
                    self.client.set_alert_threshold(alert_type, value)
                case "toggle":
                    group, name, value = args
                    print(self.client.toggle_digital_io(group, name, value))
                case "move_external":
                    axis, value = args
                    print(self.client.move_external_axis(axis, value))
                case "correct":
                    corr_type, axis, value = args
                    print(self.client.correct_position(corr_type, axis, value))
                case "speed":
                    tech_param, value = args
                    print(self.client.adjust_speed(tech_param, value))
                case "override":
                    state = args[0]
                    self.client.override_safety(state in ["on", "true", "1"])
                case "log":
                    subcmd = args[0]
                    if subcmd == "start":
                        print(f"✅ Logging to {self.client.start_logging()}")
                    elif subcmd == "stop":
                        print(self.client.stop_logging())
                    elif subcmd == "status":
                        print("📋", "ACTIVE" if self.client.is_logging_active() else "INACTIVE")
                case "graph":
                    sub = args[0]
                    if sub == "show":
                        self.client.visualise_csv_log(args[1])
                    elif sub == "compare":
                        print(self.client.compare_test_runs(args[1], args[2]))
                case "plot":
                    plot_type, csv_path = args[0], args[1]
                    overlay = args[2] if len(args) > 2 else None
                    print(self.client.generate_plot(csv_path, plot_type, overlay))
                case "move_cartesian":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.client.move_cartesian_trajectory(start, end, steps, rate)
                case "move_joint":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.client.move_joint_trajectory(start, end, steps, rate)
                case "queue_cartesian":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.client.queue_cartesian_trajectory(start, end, steps, rate)
                case "queue_joint":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.client.queue_joint_trajectory(start, end, steps, rate)
                case "execute_queue":
                    self.client.execute_queued_trajectories()
                case "clear_queue":
                    self.client.clear_trajectory_queue()
                case "show_queue":
                    print(self.client.get_trajectory_queue())
                case "export_movement_data":
                    print(self.client.export_movement_data(args[0]))
                case "compare_test_runs":
                    print(self.client.compare_test_runs(args[0], args[1]))
                case "generate_report":
                    print(self.client.generate_report(args[0], args[1]))
                case "safety-stop":
                    self.client.safety_stop()
                case "safety-reset":
                    self.client.safety_reset()
                case "safety-status":
                    print(self.client.safety_status())
                case "safety-set-limit":
                    var, lo, hi = args
                    self.client.safety_set_limit(var, lo, hi)
                case "krlparse":
                    self.client.parse_krl_to_csv(args[0], args[1], args[2])
                case "inject_rsi":
                    input_krl = args[0]
                    output_krl = args[1] if len(args) > 1 else None
                    rsi_cfg = args[2] if len(args) > 2 else "RSIGatewayv1.rsi"
                    self.client.inject_rsi(input_krl, output_krl, rsi_cfg)
                case "visualize":
                    self.client.visualise_csv_log(args[0], export="export" in args)
                case "help":
                    self.show_help()
                case _:
                    print("❌ Unknown command. Type 'help'.")
        except Exception as e:
            print(f"❌ Error: {e}")

    def parse_pose(self, pose_string):
        return dict(item.split("=") for item in pose_string.split(","))

    def extract_value(self, args, key, default, cast_type):
        for arg in args[2:]:
            if arg.startswith(f"{key}="):
                try:
                    return cast_type(arg.split("=")[1])
                except ValueError:
                    return default
        return default

    def exit(self):
        print("🛑 Exiting RSI CLI...")
        self.client.stop_rsi()
        self.running = False

    def show_help(self):
        print("""
Available Commands:
  start, stop, exit
  set <var> <value>
  show, status, ipoc, watch, reset, reconnect
  alerts on/off, set_alert_threshold <type> <value>
  toggle <group> <name> <state>
  move_external <axis> <value>, correct <RKorr/AKorr> <axis> <value>
  speed <TechParam> <value>
  log start|stop|status
  graph show <csv> | graph compare <csv1> <csv2>
  plot <type> <csv> [overlay]
  move_cartesian, move_joint, queue_cartesian, queue_joint
  execute_queue, clear_queue, show_queue
  export_movement_data <file>
  compare_test_runs <file1> <file2>
  generate_report <file> <format>
  safety-stop, safety-reset, safety-status, safety-set-limit
  krlparse <src> <dat> <output>
  inject_rsi <input> [output] [rsi_config]
  visualize <csv> [export]
  help
        """)

if __name__ == "__main__":
    cli = RSICommandLineInterface("../../examples/RSI_EthernetConfig.xml")
    cli.run()
