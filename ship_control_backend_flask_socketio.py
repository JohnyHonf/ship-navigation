from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
import threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# 硬件接口配置
ser = serial.Serial('/dev/ttyUSB0', 9600)  # 根据实际硬件修改

@app.route('/')
def index():
    return render_template('index.html')

# 接收前端控制指令
@socketio.on('control')
def handle_control(data):
    x = data['x']
    z = data['z']
    # 转换为硬件指令
    cmd = f"CTRL,{x},{z}\n"
    ser.write(cmd.encode())
    
# 传感器数据采集线程
def sensor_thread():
    while True:
        # 从串口读取传感器数据
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            parts = line.split(',')
            if parts[0] == 'GPS':
                data = {'lat': parts[1], 'lon': parts[2]}
                socketio.emit('sensor_update', data)
            elif parts[0] == 'INFRARED':
                socketio.emit('infrared_update', {'value': parts[1]})

if __name__ == '__main__':
    threading.Thread(target=sensor_thread, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000)