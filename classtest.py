#  test
from tools import CANMotorController
from esp32 import CAN
import time
import math
# TX,GPIO15,PIN 21
# RX,GPIO13,PIN 20
bus = CAN(0, tx=18, rx=19, baudrate=1000000, mode=CAN.NORMAL)  # tx=15, rx=13
print(bus)
motor = CANMotorController(bus, motor_id=127, main_can_id=254)

class JogController:
    def __init__(self, motor, accel_step=0.5, accel_interval=0.25):
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

    def _set_limit_current(self, value=45):
        """设置限流参数"""
        self.motor.write_single_param(param_name="limit_cur", value=value)

    def set_target(self, jog_vel):
        """设定目标速度（rad/s）"""
        self.target_vel = float(jog_vel)

    def ramp_to_target(self):
        """逐步加速到目标速度"""
        import time

        # 确保已设定限流等参数
        self._set_limit_current(45)

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

    def ramp_to_target_one_sec(self, duration=1.0, sample_interval=0.02):
        """
        在指定持续时间（默认 1 秒）内线性过渡到目标速度。
        duration: 总过渡时间（秒）
        sample_interval: 更新间隔（秒）
        """
        import time

        if duration <= 0 or sample_interval <= 0:
            self.current_vel = self.target_vel
            self._send_velocity(self.current_vel)
            return

        self._set_limit_current(27)

        start_vel = self.current_vel
        target_vel = self.target_vel
        delta = target_vel - start_vel

        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / sample_interval))
        vel_step = delta / steps

        for _ in range(steps):
            self.current_vel += vel_step
            self._send_velocity(self.current_vel)
            time.sleep(sample_interval)

        self.current_vel = target_vel
        self._send_velocity(target_vel)

    def ramp_to_target_soft(
        self,
        duration=2.5,
        sample_interval=0.02,
        max_step=0.03,
    ):
        """
        使用 raised-cosine（半周期余弦）曲线配合自适应步长限制，进一步抑制中后段的噪音与振动。
        duration: 目标速度的总过渡时间（秒）
        sample_interval: 发送命令的时间间隔（秒）
        max_step: 单次速度变化的上限（rad/s），避免后段突跳
        """
        if duration <= 0 or sample_interval <= 0:
            self.ramp_to_target()
            return

        self._set_limit_current(27)

        start_vel = self.current_vel
        target_vel = self.target_vel
        delta = target_vel - start_vel

        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / sample_interval))
        last_sent = start_vel

        for step in range(1, steps + 1):
            progress = step / steps
            # Raised cosine (Hann) 曲线：0.5 - 0.5*cos(pi*t)，首尾一阶/二阶导数均为0
            ease = 0.5 - 0.5 * math.cos(math.pi * progress)
            desired = start_vel + delta * ease

            # 自适应步长限制 —— 越接近目标，允许的变化越小，从而抑制中后段噪音
            taper = max(0.2, min(progress, 1 - progress) * 2)
            allowed_step = max_step * taper
            delta_cmd = desired - last_sent
            if abs(delta_cmd) > allowed_step:
                desired = last_sent + allowed_step * (1 if delta_cmd > 0 else -1)

            self.current_vel = desired
            self._send_velocity(desired)
            last_sent = desired
            time.sleep(sample_interval)

        self.current_vel = target_vel
        self._send_velocity(target_vel)

    def ramp_to_target_simple(self, interval=0.1, scale=1.05):
        """
        每隔 interval 秒把当前速度提高 scale 倍，直到达到目标速度（简单测试用）。
        """
        import time

        if interval <= 0 or scale <= 1.0:
            self.current_vel = self.target_vel
            self._send_velocity(self.current_vel)
            return

        self._set_limit_current(27)

        if self.current_vel <= 0:
            self.current_vel = min(self.accel_step, self.target_vel)
            self._send_velocity(self.current_vel)
            time.sleep(interval)

        while self.current_vel < self.target_vel:
            self.current_vel *= scale
            if self.current_vel > self.target_vel:
                self.current_vel = self.target_vel
            self._send_velocity(self.current_vel)
            time.sleep(interval)

        self._send_velocity(self.target_vel)

    def ramp_to_target_jerk(
        self,
        duration=2.0,
        sample_interval=0.02,
    ):
        """
        七阶（jerk-limited）S 曲线，确保速度/加速度/加加速度在首尾都为 0。
        duration: 过渡总时间（秒）
        sample_interval: 指令间隔（秒）
        """
        if duration <= 0 or sample_interval <= 0:
            self.ramp_to_target()
            return

        self._set_limit_current(27)
        start_vel = self.current_vel
        target_vel = self.target_vel
        delta = target_vel - start_vel
        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / sample_interval))

        for step in range(1, steps + 1):
            t = step / steps
            # jerk-limited polynomial: 35t^4 - 84t^5 + 70t^6 - 20t^7
            ease = t ** 4 * (35 - 84 * t + 70 * t ** 2 - 20 * t ** 3)
            desired = start_vel + delta * ease
            self.current_vel = desired
            self._send_velocity(desired)
            time.sleep(sample_interval)

        self.current_vel = target_vel
        self._send_velocity(target_vel)

    def ramp_to_target_exp(
        self,
        duration=2.0,
        accel_portion=0.25,
        sample_interval=0.03,
        k=3.5,
    ):
        """
        分段梯形速度曲线，首尾用指数尾巴过渡，降低起停噪音。
        accel_portion: 起步/收尾所占总时间的比例（0.05~0.45）
        k: 指数尾巴陡峭程度，越大越陡
        """
        if duration <= 0 or sample_interval <= 0:
            self.ramp_to_target()
            return

        accel_portion = min(max(accel_portion, 0.05), 0.45)
        decel_portion = accel_portion
        mid_portion = max(0.05, 1 - accel_portion - decel_portion)

        self._set_limit_current(27)
        start_vel = self.current_vel
        target_vel = self.target_vel
        delta = target_vel - start_vel
        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / sample_interval))

        def exp_tail(u):
            if u <= 0:
                return 0.0
            if u >= 1:
                return 1.0
            denom = 1 - math.exp(-k)
            return (1 - math.exp(-k * u)) / denom

        for step in range(1, steps + 1):
            progress = step / steps
            if progress < accel_portion:
                local = progress / accel_portion
                ease = accel_portion * exp_tail(local)
            elif progress > 1 - decel_portion:
                local = (progress - (1 - decel_portion)) / decel_portion
                ease = 1 - decel_portion * (1 - exp_tail(local))
            else:
                if mid_portion <= 0:
                    ease = progress
                else:
                    linear = (progress - accel_portion) / mid_portion
                    ease = accel_portion + linear * mid_portion

            desired = start_vel + delta * ease
            self.current_vel = desired
            self._send_velocity(desired)
            time.sleep(sample_interval)

        self.current_vel = target_vel
        self._send_velocity(target_vel)

    def ramp_to_target_pid(
        self,
        run_time=3.0,
        sample_interval=0.02,
        kp=0.9,
        ki=0.15,
        kd=0.0,
        feedforward=0.7,
        clamp_step=0.1,
    ):
        """
        基于离散 PI(D) 回路 + 前馈的速度追踪方式，对非理想负载更友好。
        clamp_step: 每次允许的最大速度改变量（rad/s）
        """
        if run_time <= 0 or sample_interval <= 0:
            self.ramp_to_target()
            return

        self._set_limit_current(27)
        steps = max(1, int(run_time / sample_interval))
        integral = 0.0
        prev_error = self.target_vel - self.current_vel

        for _ in range(steps):
            error = self.target_vel - self.current_vel
            integral += error * sample_interval
            derivative = (error - prev_error) / sample_interval
            command = (
                feedforward * self.target_vel
                + kp * error
                + ki * integral
                + kd * derivative
            )

            delta_cmd = command - self.current_vel
            if delta_cmd > clamp_step:
                delta_cmd = clamp_step
            elif delta_cmd < -clamp_step:
                delta_cmd = -clamp_step

            desired = self.current_vel + delta_cmd
            self.current_vel = desired
            self._send_velocity(desired)
            prev_error = error
            time.sleep(sample_interval)

            if abs(error) < 5e-3 and abs(delta_cmd) < 1e-3:
                break

        self.current_vel = self.target_vel
        self._send_velocity(self.target_vel)

    def ramp_to_target_filtered(
        self,
        duration=1.8,
        sample_interval=0.02,
        window=5,
    ):
        """
        先按线性 ramp 生成指令，再用滑动平均滤波抑制高频抖动。
        window: 滤波窗口长度
        """
        if duration <= 0 or sample_interval <= 0:
            self.ramp_to_target()
            return

        window = max(2, int(window))
        self._set_limit_current(27)
        start_vel = self.current_vel
        target_vel = self.target_vel
        delta = target_vel - start_vel
        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / sample_interval))
        history = []

        for step in range(1, steps + 1):
            progress = step / steps
            base = start_vel + delta * progress
            history.append(base)
            if len(history) > window:
                history.pop(0)
            desired = sum(history) / len(history)
            self.current_vel = desired
            self._send_velocity(desired)
            time.sleep(sample_interval)

        self.current_vel = target_vel
        self._send_velocity(target_vel)

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

def create_controller(accel_step=0.1, accel_interval=0.1):
    """Return a JogController bound to the shared motor instance."""
    return JogController(motor, accel_step=accel_step, accel_interval=accel_interval)


def run_demo():
    """Replicate the original jogging demo for quick manual tests."""
    controller = create_controller()
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
    finally:
        motor.disable()          # 关闭电机


if __name__ == "__main__":
    run_demo()
#exec(open('switch.py').read())
