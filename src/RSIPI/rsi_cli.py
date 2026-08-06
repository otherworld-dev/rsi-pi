from RSIPI.rsi_api import RSIAPI


class RSICommandLineInterface:
    """Command-Line Interface for controlling RSI Client via the namespaced RSIAPI facade."""

    def __init__(self, input_config_file):
        self.api = RSIAPI(input_config_file)
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
                    print(self.api.start())
                case "stop":
                    print(self.api.stop())
                case "exit":
                    self.exit()
                case "set":
                    var, val = args[0], args[1]
                    print(self.api.tools.update_variable(var, float(val)))
                case "show":
                    self.api.tools.show_variables()
                case "reset":
                    print(self.api.tools.reset_variables())
                case "status":
                    print(self.api.tools.show_config())
                case "ipoc":
                    print(f"🛰 IPOC: {self.api.monitoring.get_ipoc()}")
                case "watch":
                    duration = float(args[0]) if args else None
                    self.api.monitoring.watch_network(duration)
                case "reconnect":
                    print(self.api.reconnect())
                case "alerts":
                    print("❌ 'alerts' is not available in this version "
                          "(no namespaced equivalent for real-time deviation/force alerts).")
                case "set_alert_threshold":
                    print("❌ 'set_alert_threshold' is not available in this version "
                          "(no namespaced equivalent for real-time deviation/force alerts).")
                case "toggle":
                    group, name, value = args
                    state = self._parse_bool(value)
                    print(self.api.io.toggle(group, name, state))
                case "move_external":
                    axis, value = args
                    print(self.api.motion.move_external_axis(axis, float(value)))
                case "correct":
                    corr_type, axis, value = args
                    print(self.api.motion.correct_position(corr_type, axis, float(value)))
                case "speed":
                    tech_param, value = args
                    print(self.api.motion.adjust_speed(tech_param, float(value)))
                case "override":
                    state = self._parse_bool(args[0])
                    self.api.safety.override(state)
                    print(f"⚠️ Safety override {'ENABLED' if state else 'disabled'}")
                case "log":
                    subcmd = args[0]
                    if subcmd == "start":
                        filename = args[1] if len(args) > 1 else None
                        print(f"✅ Logging to {self.api.logging.start(filename)}")
                    elif subcmd == "stop":
                        print(self.api.logging.stop())
                    elif subcmd == "status":
                        print("📋", "ACTIVE" if self.api.logging.is_active() else "INACTIVE")
                case "graph":
                    sub = args[0]
                    if sub == "show":
                        self.api.viz.visualize_csv_log(args[1])
                    elif sub == "compare":
                        print(self.api.viz.compare_runs(args[1], args[2]))
                case "plot":
                    plot_type, csv_path = args[0], args[1]
                    overlay = args[2] if len(args) > 2 else None
                    print(self.api.viz.plot_static(csv_path, plot_type, overlay))
                case "move_cartesian":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.api.motion.move_cartesian_trajectory(end_pose=end, start_pose=start, steps=steps, rate=rate)
                case "move_joint":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.api.motion.move_joint_trajectory(end_joints=end, start_joints=start, steps=steps, rate=rate)
                case "queue_cartesian":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.api.motion.queue_cartesian_trajectory(start_pose=start, end_pose=end, steps=steps, rate=rate)
                case "queue_joint":
                    start = self.parse_pose(args[0])
                    end = self.parse_pose(args[1])
                    steps = self.extract_value(args, "steps", 50, int)
                    rate = self.extract_value(args, "rate", 0.04, float)
                    self.api.motion.queue_joint_trajectory(start_joints=start, end_joints=end, steps=steps, rate=rate)
                case "execute_queue":
                    self.api.motion.execute_queued_trajectories()
                case "clear_queue":
                    self.api.motion.clear_queue()
                case "show_queue":
                    print(self.api.motion.get_queue())
                case "export_movement_data":
                    print(self.api.logging.export(args[0]))
                case "compare_test_runs":
                    print(self.api.tools.compare_runs(args[0], args[1]))
                case "generate_report":
                    fmt = args[1] if len(args) > 1 else "csv"
                    print(self.api.tools.generate_report(args[0], fmt))
                case "safety-stop":
                    self.api.safety.stop()
                    print("🛑 Emergency stop activated")
                case "safety-reset":
                    self.api.safety.reset()
                    print("✅ Emergency stop reset")
                case "safety-status":
                    print(self.api.safety.status())
                case "safety-set-limit":
                    var, lo, hi = args
                    self.api.safety.set_limit(var, float(lo), float(hi))
                    print(f"✅ Safety limit set for {var}: [{lo}, {hi}]")
                case "krlparse":
                    print(self.api.krl.parse_to_csv(args[0], args[1], args[2]))
                case "inject_rsi":
                    input_krl = args[0]
                    output_krl = args[1] if len(args) > 1 else None
                    rsi_cfg = args[2] if len(args) > 2 else "RSIGatewayv1.rsi"
                    print(self.api.krl.inject_rsi(input_krl, output_krl, rsi_cfg))
                case "visualize":
                    self.api.viz.visualize_csv_log(args[0], export="export" in args)
                case "help":
                    self.show_help()
                case _:
                    print("❌ Unknown command. Type 'help'.")
        except Exception as e:
            print(f"❌ Error: {e}")

    @staticmethod
    def _parse_bool(value):
        """Interpret a CLI argument as a boolean state (on/true/1/yes -> True)."""
        return str(value).strip().lower() in ("on", "true", "1", "yes")

    def parse_pose(self, pose_string):
        return {k: float(v) for k, v in (item.split("=") for item in pose_string.split(","))}

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
        self.api.stop()
        self.running = False

    def show_help(self):
        print("""
Available Commands:
  start, stop, exit
  set <var> <value>
  show, status, ipoc, watch, reset, reconnect
  toggle <group> <name> <state>
  move_external <axis> <value>, correct <RKorr/AKorr> <axis> <value>
  speed <TechParam> <value>
  override on/off
  log start [file]|stop|status
  graph show <csv> | graph compare <csv1> <csv2>
  plot <type> <csv> [overlay]
  move_cartesian, move_joint, queue_cartesian, queue_joint
  execute_queue, clear_queue, show_queue
  export_movement_data <file>
  compare_test_runs <file1> <file2>
  generate_report <file> [format]
  safety-stop, safety-reset, safety-status, safety-set-limit
  krlparse <src> <dat> <output>
  inject_rsi <input> [output] [rsi_config]
  visualize <csv> [export]
  help

Not available in this version:
  alerts, set_alert_threshold (no namespaced equivalent for real-time deviation/force alerts)
        """)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="RSI Command-Line Interface")
    parser.add_argument("--config", type=str, default="RSI_EthernetConfig.xml",
                        help="Path to RSI config XML file (default: RSI_EthernetConfig.xml)")
    args = parser.parse_args()

    cli = RSICommandLineInterface(args.config)
    cli.run()
