import sys

sys.path.append('/usr/local/lib')
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import plantcv_3d_ir_live_newbasil_new
from apscheduler.schedulers.background import BackgroundScheduler
import time
import datetime


# Create scheduler
def tick():
    print('Tick! The time is:' + time.strftime("%Y-%m-%d %H:%M:%S"))


def killLogger():
    scheduler.shutdown()
    print("Scheduler Shutdown....")
    exit()


def capture3d():
    #    profile = pipeline.start(config)
    time.sleep(5)
    # Skip first 10 frames
    for i in range(10):
        pipeline.wait_for_frames()
    # Get color and depth frames

    frames = pipeline.wait_for_frames()

    # Align depth to colour
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    unaligned_depth_frame = frames.get_depth_frame()
    infrared_frame = frames.get_infrared_frame()
    ##    aligned_depth_frame = frames.get_depth_frame()
    ##    color_frame = frames.get_color_frame()

    # Validate both frames
    if not aligned_depth_frame or not color_frame or not infrared_frame:
        print('Not valid frames')
    ##        break

    # Filter the depth image
    # To come

    #    pipeline.stop()
    aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    unaligned_depth_image = np.asanyarray(unaligned_depth_frame.get_data())
    infrared_image = np.asanyarray(infrared_frame.get_data())

    # Remove background - set pixels to gray
    grey_color = 153
    # Need 3 channels cause color is 3 channels
    depth_image_3d = np.dstack((aligned_depth_image, aligned_depth_image, aligned_depth_image))
    bg_removed = color_image  # np.where((depth_image_3d > clipping_distance)
    # | (depth_image_3d <0), grey_color, color_image)
    # change back to <=0 if want to get rid of parts where depth unknown

    # Render images

    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(aligned_depth_image, alpha=.8), cv2.COLORMAP_JET)

    # images = np.hstack((bg_removed, depth_colormap, infrared_image))

    #    plt.imshow(images)
    #    plt.show()
    ##    cv2.namedWindow('Example', cv2.WINDOW_AUTOSIZE)
    ##    cv2.imshow('Example', images)
    ##    key = cv2.waitKey(2000)
    # Press esc or 'q' to close window
    ##    if key & 0xFF == ord('q') or key ==27:
    ##       cv2.destroyAllWindows()
    # break
    timestr = time.strftime("%Y-%m-%d-%H:%M")
    cv2.imwrite(logfile.dirName + '/uofa_rgb_' + timestr + '.png', bg_removed)
    cv2.imwrite(logfile.dirName + '/uofa_infrared_' + timestr + '.png', infrared_image)
    np.savez_compressed(logfile.dirName + '/uofa_aligned_depth_' + timestr + '', aligned_depth_image * depth_scale)
    np.savez_compressed(logfile.dirName + '/uofa_unaligned_depth_' + timestr + '', unaligned_depth_image * depth_scale)
    print('Frame captured! At: ' + time.strftime("%Y-%m-%d %H:%M:%S"))
    # Don't run plantCV right now
    # logline = plantcv_3d_ir_live_newbasil_new.plantcv_live(logfile.dirName+'/uofa_rgb_'+timestr+'.png')
    # logfile.write_log_line(logline)
    # Point cloud too


##    pc.map_to(color_frame)
##    points = pc.calculate(aligned_depth_frame)
##    points.export_to_ply('test_uofa.ply',color_frame)


# Create pipeline
pipeline = rs.pipeline()

# Create point cloud
pc = rs.pointcloud()

# Create config and configure pipeline for depth and colour

config = rs.config()
# For D415: 640x480 resolution, 6 frame/s, infrared still in colour
# config.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 6)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
# config.enable_stream(rs.stream.infrared, 640, 480, rs.format.bgr8, 6)

# For SR305: 640x480 resolution, 10 frame/s, infrared still in BW
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 10)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 10)
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 10)  # For SR305

## Start streaming. Want to, or just want to grab one frame at a time?
profile = pipeline.start(config)
####profile = pipeline.start()
##
### Get depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth scale is: ", depth_scale)
#
## Remove background more than clip_dist_m metres away
# clip_dist_m = .6
# clipping_distance = clip_dist_m/depth_scale

# Create align object to align depth and colour
align_to = rs.stream.color
align = rs.align(align_to)

# Stop until ready to get frame - doesn't work

# pipeline.stop()

if __name__ == '__main__':
    logfile = plantcv_3d_ir_live_newbasil_new.setup_logfile()
    scheduler = BackgroundScheduler()
    print("")
    print("Program Started at:" + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("")
    # prints out the date and time to console
    scheduler.add_job(tick, 'interval', seconds=600)
    # Captures and saves a rgb and depth image
    scheduler.add_job(capture3d, 'interval', seconds=3600, next_run_time=datetime.datetime.now())
    scheduler.start()
    scheduler.print_jobs()
try:
    while True:
        time.sleep(10)
except KeyboardInterrupt:
    print('Exiting')
    scheduler.shutdown()
finally:
    pipeline.stop()
