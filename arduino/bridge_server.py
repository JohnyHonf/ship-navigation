"""
智能船舶控制系统 - Arduino桥接服务器
功能: 串口 ↔ WebSocket 双向数据桥接
依赖: pip install pyserial websockets aiohttp

运行: python bridge_server.py [--port COM3] [--ws-port 8080]

协议:
  WebSocket → 浏览器  : JSON传感器数据 (实时推送)
  浏览器 → WebSocket  : 控制命令 (电机/舵机/参数)
  Python  → Arduino   : 串口原始命令 (M/S/P/T/H格式)
  Arduino → Python    : 串口响应 + 传感器JSON
"""

import asyncio
import json
import sys
import time
import argparse
import logging
from typing import Optional, Set

import serial
import serial.tools.list_ports
import websockets
from websockets.server import WebSocketServerProtocol

# ==================== 配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger('BridgeServer')

# ==================== Arduino串口管理 ====================
class ArduinoSerial:
    """Arduino串口通信管理器"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.read_buffer = ""
    
    def auto_detect_port(self) -> Optional[str]:
        """自动检测Arduino串口"""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Arduino特征: VID/PID或描述中包含"Arduino"/"CH340"/"CP210"
            desc = f"{p.description} {p.hardware_id}".lower()
            if any(kw in desc for kw in ['arduino', 'ch340', 'cp210', 'usb serial', 'usb-serial']):
                return p.device
        # 返回第一个可用串口作为备选
        return ports[0].device if ports else None
    
    def connect(self) -> bool:
        """连接Arduino"""
        if self.port is None:
            self.port = self.auto_detect_port()
        if self.port is None:
            log.error("未检测到可用串口")
            return False
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.5
            )
            # 等待Arduino启动
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.connected = True
            log.info(f"Arduino已连接: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            log.error(f"串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            log.info("Arduino已断开")
    
    def send_command(self, cmd: str) -> bool:
        """发送命令到Arduino"""
        if not self.connected or not self.serial:
            return False
        try:
            cmd_bytes = (cmd + '\n').encode('utf-8')
            self.serial.write(cmd_bytes)
            return True
        except serial.SerialException as e:
            log.error(f"发送失败: {e}")
            self.connected = False
            return False
    
    def read_line(self) -> Optional[str]:
        """非阻塞读取一行"""
        if not self.connected or not self.serial:
            return None
        try:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='replace')
                self.read_buffer += data
                
                # 提取完整行
                while '\n' in self.read_buffer:
                    line, self.read_buffer = self.read_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        return line
            return None
        except serial.SerialException:
            self.connected = False
            return None
    
    async def send_and_wait(self, cmd: str, timeout: float = 0.5) -> Optional[str]:
        """发送命令并等待响应"""
        if not self.send_command(cmd):
            return None
        start = time.time()
        while time.time() - start < timeout:
            reply = self.read_line()
            if reply:
                return reply
            await asyncio.sleep(0.01)
        return None

# ==================== WebSocket 服务 ====================
class BridgeServer:
    """串口-WebSocket桥接服务器"""
    
    def __init__(self, arduino_port: Optional[str] = None, ws_port: int = 8080):
        self.arduino = ArduinoSerial(port=arduino_port)
        self.ws_port = ws_port
        self.clients: Set[WebSocketServerProtocol] = set()
        self.running = False
        
        # 统计信息
        self.stats = {
            'serial_rx': 0,
            'serial_tx': 0,
            'ws_rx': 0,
            'ws_tx': 0,
            'start_time': time.time()
        }
    
    async def start(self):
        """启动服务器"""
        self.running = True
        
        # 连接Arduino
        if not self.arduino.connect():
            log.warning("Arduino未连接，仅WebSocket可用")
        
        # 启动WebSocket服务器
        log.info(f"WebSocket服务器启动: ws://localhost:{self.ws_port}")
        async with websockets.serve(
            self.handle_client, "0.0.0.0", self.ws_port,
            max_size=1024 * 1024,  # 1MB消息限制
            ping_interval=20,
            ping_timeout=10
        ):
            # 启动串口读取循环
            await self.serial_read_loop()
    
    async def serial_read_loop(self):
        """串口数据读取循环"""
        while self.running:
            line = self.arduino.read_line()
            if line:
                self.stats['serial_rx'] += 1
                await self.broadcast_serial_data(line)
            else:
                await asyncio.sleep(0.005)  # 5ms轮询
    
    async def broadcast_serial_data(self, line: str):
        """广播串口数据到所有WebSocket客户端"""
        if not self.clients:
            return
        
        # 检测是否为JSON传感器数据
        if line.startswith('{'):
            try:
                data = json.loads(line)
                ws_msg = json.dumps({"type": "sensor", "data": data})
                await self.broadcast(ws_msg)
                return
            except json.JSONDecodeError:
                pass
        
        # 非JSON数据作为原始消息转发
        ws_msg = json.dumps({"type": "serial", "data": line})
        await self.broadcast(ws_msg)
    
    async def broadcast(self, message: str):
        """向所有客户端广播消息"""
        if not self.clients:
            return
        self.stats['ws_tx'] += 1
        disconnected = set()
        for client in self.clients.copy():
            try:
                await client.send(message)
            except websockets.ConnectionClosed:
                disconnected.add(client)
        self.clients -= disconnected
    
    async def handle_client(self, websocket: WebSocketServerProtocol):
        """处理WebSocket客户端连接"""
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
        log.info(f"客户端连接: {client_ip} (共{len(self.clients)}个)")
        
        # 发送Arduino状态
        await websocket.send(json.dumps({
            "type": "status",
            "arduino": self.arduino.connected,
            "port": self.arduino.port
        }))
        
        try:
            async for message in websocket:
                self.stats['ws_rx'] += 1
                await self.process_client_message(websocket, message)
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            log.info(f"客户端断开: {client_ip} (剩余{len(self.clients)}个)")
    
    async def process_client_message(self, client: WebSocketServerProtocol, message: str):
        """处理客户端消息"""
        try:
            msg = json.loads(message)
            msg_type = msg.get("type", "")
            
            if msg_type == "command":
                await self.handle_command(client, msg)
            elif msg_type == "param":
                await self.handle_param(client, msg)
            elif msg_type == "ping":
                await client.send(json.dumps({"type": "pong"}))
            elif msg_type == "estop":
                await self.handle_estop(client, msg)
            else:
                # 尝试作为原始命令发送
                self.arduino.send_command(message)
                self.stats['serial_tx'] += 1
                
        except json.JSONDecodeError:
            # 非JSON消息作为原始命令
            self.arduino.send_command(message)
            self.stats['serial_tx'] += 1
    
    async def handle_command(self, client, msg: dict):
        """处理控制命令"""
        data = msg.get("data", {})
        cmd_type = data.get("cmd", "")
        
        if cmd_type == "motor":
            left = data.get("left", 0)
            right = data.get("right", 0)
            cmd = f"M{left},{right}"
            reply = await self.arduino.send_and_wait(cmd)
            await client.send(json.dumps({
                "type": "response",
                "cmd": "motor",
                "status": "ok" if reply and reply.startswith("OK") else "error",
                "reply": reply
            }))
            
        elif cmd_type == "servo":
            angle = data.get("angle", 90)
            cmd = f"S{angle}"
            reply = await self.arduino.send_and_wait(cmd)
            await client.send(json.dumps({
                "type": "response",
                "cmd": "servo",
                "status": "ok" if reply and reply.startswith("OK") else "error",
                "reply": reply
            }))
            
        elif cmd_type == "sensor":
            self.arduino.send_command("R")
            
        elif cmd_type == "test":
            mode = data.get("mode", 0)
            self.arduino.send_command(f"T{mode}")
            
        elif cmd_type == "heartbeat":
            self.arduino.send_command("H")
        
        self.stats['serial_tx'] += 1
    
    async def handle_param(self, client, msg: dict):
        """处理参数设置"""
        data = msg.get("data", {})
        param_id = data.get("id", "")
        value = data.get("value", 0)
        
        cmd = f"P{param_id}={value}"
        reply = await self.arduino.send_and_wait(cmd)
        await client.send(json.dumps({
            "type": "response",
            "cmd": "param",
            "param": param_id,
            "status": "ok" if reply and reply.startswith("OK") else "error",
            "reply": reply
        }))
        self.stats['serial_tx'] += 1
    
    async def handle_estop(self, client, msg: dict):
        """处理紧急停止"""
        active = msg.get("active", True)
        self.arduino.send_command("M0,0")
        await client.send(json.dumps({
            "type": "response",
            "cmd": "estop",
            "active": active
        }))
        self.stats['serial_tx'] += 1

# ==================== 主程序 ====================
def main():
    parser = argparse.ArgumentParser(
        description="智能船舶 - Arduino桥接服务器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python bridge_server.py                          # 自动检测串口
  python bridge_server.py --port COM3              # 指定串口
  python bridge_server.py --port /dev/ttyUSB0      # Linux串口
  python bridge_server.py --ws-port 9090           # 自定义WebSocket端口
  python bridge_server.py --baud 9600              # 自定义波特率
        """
    )
    parser.add_argument("--port", help="Arduino串口 (如 COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率 (默认115200)")
    parser.add_argument("--ws-port", type=int, default=8080, help="WebSocket端口 (默认8080)")
    
    args = parser.parse_args()
    
    # 创建服务器
    server = BridgeServer(
        arduino_port=args.port,
        ws_port=args.ws_port
    )
    server.arduino.baudrate = args.baud
    
    # 列出可用串口
    ports = serial.tools.list_ports.comports()
    if ports:
        log.info("可用串口:")
        for p in ports:
            log.info(f"  {p.device}: {p.description}")
    else:
        log.warning("未检测到任何串口")
    
    # 运行
    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        log.info("服务器已停止")
    finally:
        server.arduino.disconnect()

if __name__ == "__main__":
    main()