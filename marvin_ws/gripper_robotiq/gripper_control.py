import time
from robot.robotiq_sdk.HandE import HandEForRtu

if __name__ == '__main__':
    # 初始化夹爪
    gripper = HandEForRtu(port="/dev/ttyUSB0", autoInit=True)
    print("夹爪已初始化，输入 c 关闭 / o 打开 / q 退出")
    
    while True:
        cmd = input("> ").strip().lower()
        
        if cmd == 'c':  # close 关闭
            gripper.move(pos=0, speed=0x00, force=0x01)
            print("夹爪关闭")
        elif cmd == 'o':  # open 打开
            gripper.move(pos=50, speed=0x00, force=0x01)
            print("夹爪打开")
        elif cmd == 'q':  # quit 退出
            print("退出程序")
            break
        time.sleep(0.5) 
