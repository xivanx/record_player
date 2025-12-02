from tools import CANMotorController
from esp32 import CAN
# TX,GPIO15,PIN 21
# RX,GPIO13,PIN 20
bus = CAN(0, tx=18, rx=19, baudrate=1000000, mode=CAN.NORMAL) #tx=15, rx=13
print(bus)
motor = CANMotorController(bus, motor_id=127, main_can_id=254)
jog_vel = 3.49  # rad/s
uint_value = motor._float_to_uint(jog_vel, motor.V_MIN, motor.V_MAX, 16)
jog_vel_bytes = motor.format_data(data=[uint_value], format="u16", type="encode")[:2][::-1]

data1 = [0x05, 0x70, 0x00, 0x00, 0x07, 0x01] + jog_vel_bytes
motor.clear_can_rx()
received_msg_data, received_msg_arbitration_id = motor.send_receive_can_message(
    cmd_mode=motor.CmdModes.SINGLE_PARAM_WRITE, data2=motor.MAIN_CAN_ID, data1=data1
)
motor.parse_received_msg(received_msg_data, received_msg_arbitration_id)
motor.write_single_param(param_name="limit_cur", value=27)
motor.disable()