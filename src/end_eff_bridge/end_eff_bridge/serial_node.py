#!/usr/bin/env python3
import time, threading, queue, re
import serial, serial.tools.list_ports
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Float32
from std_msgs.msg import Float32MultiArray

FORCE_TILDE_RE = re.compile(r'~g=(?P<g>\d+(?:\.\d+)?)')

class SerialBridge(Node):
    def __init__(self):
        super().__init__('end_eff_serial_bridge')

        # ---- params ----
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('auto_hint', 'usb')

        # prong preset parameters (ROS is source of truth)
        self.declare_parameter('open_left_deg',        150.0)
        self.declare_parameter('open_right_deg',        30.0)
        self.declare_parameter('cp_left_deg',           55.0)
        self.declare_parameter('cp_right_deg',         115.0)
        self.declare_parameter('cf_left_deg',            0.0)
        self.declare_parameter('cf_right_deg',         180.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').value)
        self.hint = self.get_parameter('auto_hint').get_parameter_value().string_value

        # ---- topics ----
        self.create_subscription(String, '/prongs/cmd', self.on_cmd, 10)             # raw passthrough
        self.create_subscription(String, '/prongs/mode', self.on_mode, 10)           # "o" | "cp" | "cf"
        self.create_subscription(Float32MultiArray, '/prongs/pose_deg', self.on_pose_deg, 10)  # [L, R]

        self.pub_line  = self.create_publisher(String,  '/prongs/line',  10)         # raw serial lines
        self.pub_force = self.create_publisher(Float32, '/prongs/force_g', 10)       # parsed grams

        # react to parameter updates (dynamic reconfig style)
        self.add_on_set_parameters_callback(self.on_param_change)

        # ---- serial infra ----
        self.txq = queue.Queue()
        self.ser = None
        self.stop = threading.Event()
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.writer, daemon=True).start()

        # timer to (re)push presets once serial is open
        self.push_timer = self.create_timer(1.0, self.try_push_presets_once)
        self.presets_pushed = False

        self.get_logger().info('end_eff_bridge up. Publish /prongs/mode: o|cp|cf, or /prongs/pose_deg [L,R]')

    # ---------- helpers ----------
    def p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def build_def_cmds(self):
        # Build the 3 "def ..." lines for Arduino
        def line(preset, l, r):
            return f"def {preset} L{int(round(l))} R{int(round(r))}\r\n".encode('utf-8')
        return [
            line("o",  self.p('open_left_deg'), self.p('open_right_deg')),
            line("cp", self.p('cp_left_deg'),   self.p('cp_right_deg')),
            line("cf", self.p('cf_left_deg'),   self.p('cf_right_deg')),
        ]

    def try_push_presets_once(self):
        if self.presets_pushed:  # done
            return
        if not self.ser:
            return
        # push definitions to Arduino
        for chunk in self.build_def_cmds():
            try:
                self.ser.write(chunk); self.ser.flush()
                time.sleep(0.03)
            except Exception as e:
                self.get_logger().warn(f'Preset push failed: {e}')
                return
        self.presets_pushed = True
        self.get_logger().info('Presets pushed to Arduino.')

    # ---------- topic handlers ----------
    def on_cmd(self, msg: String):
        # raw passthrough
        self.txq.put((msg.data.strip() + '\r\n').encode('utf-8'))

    def on_mode(self, msg: String):
        s = msg.data.strip().lower()
        if s in ('o', 'cp', 'cf'):
            self.txq.put((s + '\r\n').encode('utf-8'))
        else:
            self.get_logger().warn(f'Unknown mode "{msg.data}" (use o|cp|cf)')

    def on_pose_deg(self, msg: Float32MultiArray):
        # expects length 2: [Ldeg, Rdeg]
        if len(msg.data) < 2:
            self.get_logger().warn('pose_deg expects [L,R] degrees')
            return
        L = msg.data[0]; R = msg.data[1]
        line = f"pose L{int(round(L))} R{int(round(R))}\r\n"
        self.txq.put(line.encode('utf-8'))

    # ---------- parameter change callback ----------
    def on_param_change(self, params):
        # if any of the six preset params changed, push updated "def ..." lines
        preset_names = {
            'open_left_deg', 'open_right_deg',
            'cp_left_deg', 'cp_right_deg',
            'cf_left_deg', 'cf_right_deg'
        }
        changed = any((p.name in preset_names and p.type_ != Parameter.Type.NOT_SET) for p in params)
        if changed:
            self.presets_pushed = False  # trigger a re-push on next timer tick
            self.get_logger().info('Preset params changed → will push new defs.')
        return rclpy.parameter.SetParametersResult(successful=True)

    # ---------- serial ----------
    def open_serial(self):
        if not self.port:
            ports = list(serial.tools.list_ports.comports())
            cand = [p.device for p in ports if self.hint.lower() in p.device.lower() or 'usb' in p.device.lower()]
            if cand: self.port = cand[0]
            elif ports: self.port = ports[0].device
        if not self.port:
            self.get_logger().warn('No serial ports found.'); return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)
            try: self.ser.setDTR(False); self.ser.setRTS(False)
            except: pass
            self.get_logger().info(f'Opened {self.port} @ {self.baud}')
            return True
        except Exception as e:
            self.get_logger().warn(f'Open failed: {e}'); self.ser=None; return False

    def close_serial(self):
        if self.ser:
            try: self.ser.close()
            finally: self.ser = None

    def reader(self):
        buf = b''
        while not self.stop.is_set():
            if not self.ser and not self.open_serial():
                time.sleep(1.0); continue
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        s = line.rstrip(b'\r').decode('utf-8', errors='replace')
                        self.pub_line.publish(String(data=s))

                        # parse ~g=... OR CSV "t,f"
                        m = FORCE_TILDE_RE.search(s)
                        g = None
                        if m:
                            try: g = float(m.group('g'))
                            except: g = None
                        elif ',' in s:
                            parts = s.split(',')
                            if len(parts) == 2:
                                try: g = float(parts[1])
                                except: pass
                        if g is not None:
                            self.pub_force.publish(Float32(data=g))
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')
                self.close_serial()
                time.sleep(0.5)

    def writer(self):
        while not self.stop.is_set():
            if not self.ser:
                time.sleep(0.1); continue
            try:
                chunk = self.txq.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.ser.write(chunk); self.ser.flush()
            except Exception as e:
                self.get_logger().warn(f'Write error: {e}')
                self.close_serial()
                time.sleep(0.5)

    def destroy_node(self):
        self.stop.set()
        time.sleep(0.1)
        self.close_serial()
        return super().destroy_node()

def main():
    rclpy.init()
    n = SerialBridge()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
