# ====== CyberGear Jog - Low-Memory, Jerk S-curve, 3s to 3.49 rad/s ======
# 重点优化：循环零分配（预分配 CAN 帧），周期性 gc.collect()，固定周期调度
from tools import CANMotorController
from esp32 import CAN
import time, math, gc

# ---------- User Config ----------
CAN_ID          = 0
CAN_TX_PIN      = 18        # 如实际接 15/13，请改这里
CAN_RX_PIN      = 19
CAN_BAUD        = 1_000_000

MOTOR_ID        = 127
MAIN_CAN_ID     = 254

TARGET_VEL      = 3.49      # rad/s
RAMP_TIME       = 3.0       # s
HOLD_TIME       = 2.0       # s
SAMPLE_DT       = 0.02      # s (50 Hz)

LIMIT_CUR       = 35        # 空载建议 30~40；必要时可升至 45
DEADBAND        = 0.005     # rad/s 变化小于此阈值不下发
MAX_STEP        = 0.03      # rad/s 单步上限
ENDIAN_SWAP     = True      # 若端序无需反转，改为 False

GC_EVERY_STEPS  = 50        # 每 N 步做一次 gc.collect()

# ---------- CAN Harden ----------
def _errcode(e):
    try: return getattr(e, "args", [None])[0]
    except: return None

def _is_invalid_state(e):  # ESP_ERR_INVALID_STATE
    return _errcode(e) == -259

def _safe_call(fn, *a, **kw):
    try: return fn(*a, **(kw or {}))
    except Exception as ex: return ex

def can_safe_kill_all():
    try:
        inst = CAN(CAN_ID)
    except Exception:
        inst = None
    if inst:
        _safe_call(getattr(inst, "stop", lambda: None))
        _safe_call(inst.deinit)

def can_fresh_init(max_retries=3, retry_delay=0.15):
    for _ in range(max_retries):
        can_safe_kill_all()
        _safe_call(lambda: CAN(CAN_ID).deinit())
        try:
            bus = CAN(CAN_ID, tx=CAN_TX_PIN, rx=CAN_RX_PIN, baudrate=CAN_BAUD, mode=CAN.NORMAL)
        except OSError as e:
            if _is_invalid_state(e):
                time.sleep(retry_delay); continue
            raise
        try:
            if hasattr(bus, "start"): bus.start()
        except OSError as e:
            if _is_invalid_state(e):
                _safe_call(getattr(bus, "stop", lambda: None))
                _safe_call(bus.deinit)
                time.sleep(retry_delay)
                continue
            else:
                raise
        return bus
    raise OSError(-259, "ESP_ERR_INVALID_STATE (after retries in can_fresh_init)")

def can_safe_deinit(bus):
    if not bus: return
    _safe_call(getattr(bus, "stop", lambda: None))
    _safe_call(bus.deinit)

# ---------- Controller ----------
class JogController:
    def __init__(self, motor):
        self.motor = motor
        self.current_vel = 0.0
        self.target_vel  = 0.0
        # 预分配 CAN 帧（8字节）：[0x05,0x70,0x00,0x00,0x07,0x01,vel_lo,vel_hi]
        self.frame = bytearray((0x05, 0x70, 0x00, 0x00, 0x07, 0x01, 0x00, 0x00))

    def _put_u16(self, val):
        # 将 u16 写入 frame[6:8]，端序可切换
        lo = val & 0xFF
        hi = (val >> 8) & 0xFF
        if ENDIAN_SWAP:
            self.frame[6] = lo
            self.frame[7] = hi
        else:
            self.frame[6] = hi
            self.frame[7] = lo

    def _send_velocity_once(self, vel):
    # float -> u16
        val = self.motor._float_to_uint(vel, self.motor.V_MIN, self.motor.V_MAX, 16)
        self._put_u16(val)

        # ---- 关键修改：强制转为 list[int]，并打印一次自检 ----
        data1_list = [self.frame[0], self.frame[1], self.frame[2], self.frame[3],
                    self.frame[4], self.frame[5], self.frame[6], self.frame[7]]
        # 可先打印一次确认不为全 0（调试好后注释掉）
        # print("TX data1:", data1_list)

        self.motor.clear_can_rx()
        recv_data, recv_id = self.motor.send_receive_can_message(
            cmd_mode=self.motor.CmdModes.SINGLE_PARAM_WRITE,
            data2=self.motor.MAIN_CAN_ID,
            data1=data1_list
        )
        self.motor.parse_received_msg(recv_data, recv_id)

    def _send_velocity(self, vel):
        try:
            self._send_velocity_once(vel)
        except OSError as e:
            if _is_invalid_state(e):
                try: can_safe_deinit(self.motor.bus)
                except: pass
                self.motor.bus = can_fresh_init()
                self._send_velocity_once(vel)
            else:
                raise

    def set_limit_current(self, value=LIMIT_CUR):
        self.motor.write_single_param(param_name="limit_cur", value=value)

    def set_target(self, v):
        self.target_vel = float(v)

    def s_curve_to_target(self, duration=RAMP_TIME, dt=SAMPLE_DT,
                          deadband=DEADBAND, max_step=MAX_STEP):
        """7阶 S 曲线 + 固定周期 + 零分配发送 + 周期性 GC"""
        if duration <= 0 or dt <= 0:
            self.current_vel = self.target_vel
            self._send_velocity(self.current_vel)
            return

        self.set_limit_current(LIMIT_CUR)
        start = self.current_vel
        goal  = self.target_vel
        delta = goal - start
        if abs(delta) < 1e-6:
            return

        steps = max(1, int(duration / dt))
        next_t = time.ticks_us()
        PERIOD_US = int(dt * 1_000_000)

        last_sent = start
        for i in range(1, steps + 1):
            # jerk-limited polynomial: 35t^4 - 84t^5 + 70t^6 - 20t^7
            t = i / steps
            ease = t*t*t*t * (35 - 84*t + 70*t*t - 20*t*t*t)
            desired = start + delta * ease

            dv = desired - last_sent
            if abs(dv) >= deadband:
                if dv >  max_step: desired = last_sent + max_step
                if dv < -max_step: desired = last_sent - max_step
                self.current_vel = desired
                self._send_velocity(desired)
                last_sent = desired

            if (i % GC_EVERY_STEPS) == 0:
                gc.collect()

            # 固定周期忙等
            next_t += PERIOD_US
            while time.ticks_diff(next_t, time.ticks_us()) > 0:
                pass

        self.current_vel = goal
        self._send_velocity(goal)
        gc.collect()

    def hold(self, t=HOLD_TIME, dt=SAMPLE_DT):
        if t <= 0: return
        goal = self.target_vel
        steps = max(1, int(t / dt))
        next_t = time.ticks_us()
        PERIOD_US = int(dt * 1_000_000)
        for i in range(steps):
            self._send_velocity(goal)
            if (i % GC_EVERY_STEPS) == 0:
                gc.collect()
            next_t += PERIOD_US
            while time.ticks_diff(next_t, time.ticks_us()) > 0:
                pass

    def soft_stop(self, stop_time=2.0, dt=SAMPLE_DT):
        if stop_time <= 0:
            self.current_vel = 0.0
            self._send_velocity(0.0); gc.collect(); return
        self.set_target(0.0)
        self.s_curve_to_target(duration=stop_time, dt=dt)

# ---------- Main ----------
bus = None
motor = None
ctrl = None

try:
    gc.collect()  # 启动前先做一次
    bus = can_fresh_init()
    print("CAN bus ready:", bus)

    motor = CANMotorController(bus, motor_id=MOTOR_ID, main_can_id=MAIN_CAN_ID)
    ctrl = JogController(motor)

    ctrl.set_target(TARGET_VEL)
    ctrl.s_curve_to_target(duration=RAMP_TIME, dt=SAMPLE_DT)
    ctrl.hold(HOLD_TIME, dt=SAMPLE_DT)
    ctrl.soft_stop(stop_time=2.0, dt=SAMPLE_DT)

except KeyboardInterrupt:
    print("KeyboardInterrupt → soft stop")
    try:
        if ctrl: ctrl.soft_stop(stop_time=2.0, dt=SAMPLE_DT)
    except Exception as e:
        print("soft_stop 异常：", e)

finally:
    try:
        if motor: motor.disable()
    except Exception as e:
        print("motor.disable() 异常：", e)
    try:
        can_safe_deinit(bus)
    except Exception as e:
        print("CAN deinit 异常：", e)
    gc.collect()
    print("清理完成，退出。")