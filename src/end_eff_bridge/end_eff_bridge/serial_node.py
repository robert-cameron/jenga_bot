#!/usr/bin/env python3
import time, threading, queue, re
import serial, serial.tools.list_ports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

FORCE_RE = re.compile(r'~g=(?P<g>\d+(?:\.\d+)?)')  # parses “… ~g=123.4”

class SerialBridge(Node):
    def __init__(self):
        super().__init__('end_eff_serial_bridge')
        # params
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('auto_hint', 'usb')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').value)
        self.hint = self.get_parameter('auto_hint').get_parameter_value().string_value

        # topics
        self.create_subscription(String, '/prongs/cmd', self.on_cmd, 10)   # send lines to Arduino
        self.pub_line  = self.create_publisher(String,  '/prongs/line',  10)   # raw serial lines
        self.pub_force = self.create_publisher(Float32, '/prongs/force_g', 10) # parsed grams

        # serial infra
        self.txq = queue.Queue()
        self.ser = None
        self.stop = threading.Event()
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.writer, daemon=True).start()
        self.get_logger().info('end_eff_bridge up. Publish to /prongs/cmd, e.g. "open 30"')

    def on_cmd(self, msg: String):
        self.txq.put((msg.data.strip() + '\r\n').encode('utf-8'))

    # --- serial helpers ---
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
                        m = FORCE_RE.search(s)
                        if m:
                            try: self.pub_force.publish(Float32(data=float(m.group('g'))))
                            except: pass
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}'); self.close_serial(); time.sleep(0.5)

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
                self.get_logger().warn(f'Write error: {e}'); self.close_serial(); time.sleep(0.5)

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
