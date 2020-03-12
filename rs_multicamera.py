import sys
#sys.path.append('/usr/local/lib')
import usb.core
import usb.util
import pyrealsense2 as rs


ctx=rs.context()
devices=ctx.query_devices()
#sensors=ctx.query_all_sensors()

#print(devices[0])
#print(devices[1])
for device in devices:
    print("serial number:   ", rs.camera_info.serial_number)

    print("device", device)
    print(device.get_info(rs.camera_info.product_id))
    print(device.get_info(rs.camera_info.asic_serial_number))
    print(device.get_info(rs.camera_info.serial_number))
    print('-----')