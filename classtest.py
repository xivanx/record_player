#  test
from tools import CANMotorController
from esp32 import CAN
import time
# TX,GPIO15,PIN 21
# RX,GPIO13,PIN 20
bus = CAN(0, tx=18, rx=19, baudrate=1000000, mode=CAN.NORMAL) #tx=15, rx=13
print(bus)
motor = CANMotorController(bus, motor_id=127, main_can_id=254)

class JogController:
    def __init__(self, motor, accel_step=0.1, accel_interval=0.05):
        """
        motor: 你的 motor 实例
        accel_step: 每次增加的 rad/s
        accel_interval: 每次增加之间的时间（秒）
        """
        self.motor = motor
        self.accel_step = accel_step
        self.accel_interval = accel_interval
        self.current_vel = 0
        self.target_vel = 0

    def _send_velocity(self, vel):
        """把速度转成 CAN 指令并发送"""
        # 1) float → uint
        uint_value = self.motor._float_to_uint(vel, self.motor.V_MIN, self.motor.V_MAX, 16)
        # 2) 编码为字节并取反序
        jog_vel_bytes = self.motor.format_data([uint_value], "u16", "encode")[:2][::-1]

        # 3) 组 CAN 数据
        data1 = [0x05, 0x70, 0x00, 0x00, 0x07, 0x01] + jog_vel_bytes

        # 4) 发送并解析
        self.motor.clear_can_rx()
        recv_data, recv_id = self.motor.send_receive_can_message(
            cmd_mode=self.motor.CmdModes.SINGLE_PARAM_WRITE,
            data2=self.motor.MAIN_CAN_ID,
            data1=data1
        )
        self.motor.parse_received_msg(recv_data, recv_id)

    def _set_limit_current(self, value=27):
        """设置限流参数"""
        self.motor.write_single_param(param_name="limit_cur", value=value)

    def set_target(self, jog_vel):
        """设定目标速度（rad/s）"""
        self.target_vel = float(jog_vel)

    def ramp_to_target(self):
        """逐步加速到目标速度"""
        import time

        # 确保已设定限流等参数
        self._set_limit_current(27)

        # 加速或减速过程（这里写加速为主）
        while abs(self.target_vel - self.current_vel) > 1e-3:
            # 每次向目标方向逐步逼近
            if self.current_vel < self.target_vel:
                self.current_vel += self.accel_step
                if self.current_vel > self.target_vel:
                    self.current_vel = self.target_vel
            else:
                self.current_vel -= self.accel_step
                if self.current_vel < self.target_vel:
                    self.current_vel = self.target_vel

            # 发送当前速度
            self._send_velocity(self.current_vel)

            # 等待下一步（控制平滑度）
            time.sleep(self.accel_interval)

    def stop(self):
        """立即停止（设 0）"""
        self.current_vel = 0
        self._send_velocity(0)

    def soft_stop(self, stop_time=3.0):
        """
        缓慢停止到 0
        stop_time: 停止总时间（秒）
        """
        steps = int(stop_time / self.accel_interval)
        if steps <= 0:
            steps = 1
        vel_step = self.current_vel / steps

        for i in range(steps):
            self.current_vel -= vel_step
            if self.current_vel < 0:
                self.current_vel = 0
            self._send_velocity(self.current_vel)
            time.sleep(self.accel_interval)

        self.current_vel = 0
        self._send_velocity(0)

controller = JogController(motor, accel_step=0.1, accel_interval=0.1)
controller.set_target(3.49)     # 设置目标速度
controller.ramp_to_target()     # 平滑加速到 3.49 rad/s

try:
    while True:
        """ # 获取当前速度（rad/s）
        vel = controller.current_vel()  # 或者 controller.current_vel

        # 打印
        print("当前速度: {:.3f} rad/s".format(vel))

        # 等待 0.1 秒 """
        time.sleep(0.1)

except KeyboardInterrupt:
    print("主循环停止")
    controller.soft_stop()          # 停止
    motor.disable()  

#time.sleep(15)                # 维持 5 秒
motor.disable()          # 关闭电机
#exec(open('classtest.py').read())