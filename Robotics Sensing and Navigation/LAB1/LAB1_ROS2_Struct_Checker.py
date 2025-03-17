import os
import pandas as pd
import time
import rclpy
import sys
import importlib


DATA = "BREAD"
argv=sys.argv
port = str(argv[1])

def callback(data):

    global DATA

    print("Callback occurred with data: ", data)
    #if DATA == "None":
    DATA = data

if __name__ == "__main__":

    home_dir = os.getcwd()
    home_files = os.listdir(os.getcwd())
    #global DATA

    print(home_files)

    assert "src" in  home_files, "No src folder found"

    os.chdir(os.getcwd()+"/src")

    files = os.listdir(os.getcwd())

    assert ( "gps_driver" in files )==True, "ROS Package naming is not correct"

    if ("gps_driver" in files):
        package = "gps_driver"
    else:
        package = "gps-driver"

    msg_package = [folder for folder in files if folder not in ["gps_driver", "gps-driver"]][0]
    print("msg_package: ", msg_package)
    
    os.chdir(os.getcwd()+"/")
    try:
        # Dynamically import the message module
        msg_module = importlib.import_module(f"{msg_package}.msg")
        # Access the GPSmsg class from the module
        GPSmsg = getattr(msg_module, 'GPSmsg')
    except (ImportError, AttributeError):
        assert False, "Unable to import GPSmsg"
    assert("msg" in os.listdir(os.getcwd()+"/"+msg_package+"/") and 
           "launch" in os.listdir(os.getcwd()+"/"+package+"/") and 
           package in os.listdir(os.getcwd()+"/"+package+"/"))==True, "Incorrect folder names, do you have the correct python package, 'msg' and 'launch' folders?"

    assert os.path.isfile(os.path.join(os.getcwd(), msg_package, "msg", "GPSmsg.msg")), "No GPSmsg.msg file found or naming convention incorrect"

    assert any(f.endswith(".py") and f != "__init__.py" for f in os.listdir(os.getcwd() + "/" + package + "/" + package)), "No <driver>.py file found in python folder"

    assert os.path.isfile(os.path.join(os.getcwd(), package, "launch", "gps_launch.py")), "No gps_launch.py file found or naming convention incorrect"

    os.chdir(home_dir)

    # Build the workspace
    if os.system("colcon build") != 0:
        print("Build failed!")
        sys.exit(1)

    os.system("bash -c 'source install/setup.bash'")

    os.system('screen -S ros2_evaluator_node -dm ros2 launch "'+package+'" gps_launch.py port:="'+port+'"')

    print("Screen Running, your ROS node should start within 10 seconds.")

    time.sleep(10)
    

    # Initialize ROS2 node
    rclpy.init(args=None)
    node = rclpy.create_node('Evaluator')

    try:
        msg_module = importlib.import_module(f"{msg_package}.msg")
        GPSmsg = getattr(msg_module, 'GPSmsg')
    except (ImportError, AttributeError):
        assert False, "Unable to import GPSmsg"

    # from gps_msg.msg import GPSmsg
    
    node.create_subscription(GPSmsg, 'gps', callback, 10)

    cur_time = time.time()
    try :
        while "BREAD" in DATA and time.time() - cur_time < 30:

            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.5)  # Spin the node to process callbacks
            print("waiting for topic")
            print(" ")

        if "BREAD" in DATA:
            assert False, "not publishing over topic gps. Either node never initialized or topic name was incorrect"

    except:
        pass

    print("\n\n\n")
    os.system("clear")
    print("Screen Dumping Values received messages")

    try :

        if DATA.header.frame_id.upper() != "GPS1_FRAME":
            assert False, "Incorrect Frame ID"

        print("Seconds : ")
        print(DATA.header.stamp.sec)

        print("\nNano Seconds : ")
        print(DATA.header.stamp.nanosec)

        print("\nLatitude : ")
        print(abs(DATA.latitude))

        print("\nLongitude : ")
        print(abs(DATA.longitude))
        
        print("\nEasting : ")
        print(abs(DATA.utm_easting))

        print("\nNorthing : ")
        print(abs(DATA.utm_northing))

        print("\nZone : ")
        print(abs(DATA.zone))

        print("\nLetter : ")
        print(DATA.letter)

        ###############################################################################
        # Uncomment these lines if Hanu plans to use HDOP and UTC
        #print("\nHDOP : ")
        #print(DATA.hdop) 
        #print("\nUTC : ")
        #print(DATA.utc)
        print("\nAltitude : ")
        print(DATA.altitude)
               

    except:

        os.system("screen -S ros2_evaluator_node -X quit")

        assert False, "Structure Failed. Review Handout for Correct Structure"

    
    os.system("screen -S ros2_evaluator_node -X quit")
    node.destroy_node()
    rclpy.shutdown()

    print("\n\nEnd of screen dump\n\n\n")    
    print("The repository structure is correct\n\n\n")
