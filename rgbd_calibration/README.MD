## Author
- **Welin**  version 0.1.1 2019.01.17

## Branches

- **master** using Kinect to detect plane 

## Dependency

* PCL 1.7
* tf
* Eigen 3
* kinect-I driver (in dependency folder)

## Instruction

Firstly, we should open the kinect camera by using the OpenNI node or freenect node (these could be found on ROS official website) to get the point cloud data.


# topics:

subscribe:

* camera/depth/points    -- to get the origin point cloud data (type: sensor_msgs/PointCloud2)

publish:

* /camera/depth/points/voxelgrid_filtered   -- filtered points after using the voxel grid filter (type: sensor_msgs/PointCloud2)
* ~selected_points  -- selected points used to fit the plane (type: sensor_msgs/PointCloud)
* ~polygon_plane_axis    -- the fitting plane using selected points (type: geometry_msgs/PolygonStamped)
* ~camera_pose      -- kinect pose after calibration (type: geometry_msgs/PoseStamped)
* ~calibration_result  --result of calibration, namely height, roll, pitch, yaw (type: rgbd_calibration/RgbdMsg)

# parameters:
* origin_x: 0.26       # distance to base_footprint
* camera_z: 0          # give initial value 0 for the first calibration
* camera_roll: 0.02
* camera_pitch: 0.44
* roi_w_rate: 20       #  max 50%
* roi_h_rate: 10       #  max 50%
* z_diff: 0.02         #  unit m
* angle_diff: 0.02     #  unit rad
* ransac_threshold: 0.02   # RANSAC distance threshold
* ransac_iteration: 10000  # RANSAC iterations
* method: 2            # 0 only filters, 1 LMS, 2 ransac

## important
Remember to backup the param if you have done calibration.

