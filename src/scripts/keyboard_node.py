#!/usr/bin/env python3

import sys,tty,termios,select
import rospy
from std_msgs.msg import Char,String



def publish_keyboard_input():
    fd  = sys.stdin.fileno()
    old_Settings = termios.tcgetattr(fd)
    # print(old_Settings)

    try:
        tty.setcbreak(fd)
        print('setcbreak is used so ctrl+c works and q for exit')
        while True:
            
            char = get_key()

            if char == 'q':
                break
            
            print(f'key = {char}')
            
    except Exception as e:
        print('exception raised ',e)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_Settings)


def get_key():
    read_list,_,_ = select.select([sys.stdin],[],[],0.1)
    if read_list:
        key = sys.stdin.read(1)
        if key == '\x1b':  ## special character like arrows
            key = sys.stdin.read(2)   ## beacuse special characters are expressed as two charaters [A
        sys.stdin.flush()
    if not read_list:
        key = ''    
    # print(key)
    return(key)
    
if __name__ == "__main__":
    
    # while 1:
    #     char = sys.stdin.read(1)
    #     if char == 'q':
    #         break
    #     print('key = ',char)
    #     if char == '\x1b':
    #         print('yeah')
    #     sys.stdin.flush()
    
    rospy.init_node('keyboard_commands_publisher',anonymous=True)
    key_pub = rospy.Publisher('keyboard_commands',String, queue_size=10)
    rate = rospy.Rate(100)
    fd  = sys.stdin.fileno()
    old_Settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        print('setcbreak is used so ctrl+c works and q for exit')
        while not rospy.is_shutdown():
            
            char = get_key()

            if char == 'q':
                break
            
            print(f'key = {char}')
            key_pub.publish(char)
            rate.sleep()

    except Exception as e:
        print('exception raised ',e)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_Settings)

    

    termios.tcsetattr(fd,termios.TCSADRAIN,old_Settings)