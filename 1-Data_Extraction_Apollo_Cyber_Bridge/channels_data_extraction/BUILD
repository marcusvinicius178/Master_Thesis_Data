load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "*.py",
    ])
)


py_binary(
    	name = "odometry",
    	srcs = ["odometry.py"],
    	deps = [
    		"//modules/localization/proto:gps_py_pb2",
    		"//modules/localization/proto:imu_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    	name = "channels_extraction",
    	srcs = ["channels_extraction.py"],
    	deps = [
    		"//modules/localization/proto:imu_py_pb2",
    		"//modules/localization/proto:gps_py_pb2",
    		"//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    	name = "test_channels",
    	srcs = ["test_channels.py"],
    	deps = [
    		"//modules/localization/proto:imu_py_pb2",
    		"//modules/localization/proto:gps_py_pb2",
    		"//modules/localization/proto:localization_py_pb2",
    		"//modules/localization/proto:localization_status_py_pb2",
    		"//modules/perception/proto:perception_obstacle_py_pb2",
    		"//modules/perception/proto:traffic_light_detection_py_pb2",
    		"//modules/prediction/proto:prediction_obstacle_py_pb2",
    		"//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
    		"//modules/drivers/gnss/proto:imu_py_pb2",
    		"//modules/drivers/gnss/proto:ins_py_pb2",
    		"//modules/drivers/proto:conti_radar_py_pb2",
    		"//modules/drivers/proto:pointcloud_py_pb2",
    		"//modules/routing/proto:routing_py_pb2",
    		"//modules/planning/proto:planning_py_pb2",
    		"//modules/control/proto:control_cmd_py_pb2",
    		"//modules/canbus/proto:chassis_py_pb2",
    		"//modules/guardian/proto:guardian_py_pb2",
    		"//modules/monitor/proto:system_status_py_pb2",
    		"//modules/common/monitor_log/proto:monitor_log_py_pb2",
    		"//modules/dreamview/proto:hmi_status_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)


py_binary(
    	name = "get_cyber_channels_data",
    	srcs = ["get_cyber_channels_data.py"],
    	deps = [
    		"//modules/localization/proto:imu_py_pb2",
    		"//modules/localization/proto:gps_py_pb2",
    		"//modules/localization/proto:localization_py_pb2",
    		"//modules/localization/proto:localization_status_py_pb2",
    		"//modules/perception/proto:perception_obstacle_py_pb2",
    		"//modules/perception/proto:traffic_light_detection_py_pb2",
    		"//modules/prediction/proto:prediction_obstacle_py_pb2",
    		"//modules/drivers/gnss/proto:gnss_best_pose_py_pb2",
    		"//modules/drivers/gnss/proto:imu_py_pb2",
    		"//modules/drivers/gnss/proto:ins_py_pb2",
    		"//modules/drivers/proto:conti_radar_py_pb2",
    		"//modules/drivers/proto:pointcloud_py_pb2",
    		"//modules/routing/proto:routing_py_pb2",
    		"//modules/planning/proto:planning_py_pb2",
    		"//modules/control/proto:control_cmd_py_pb2",
    		"//modules/canbus/proto:chassis_py_pb2",
    		"//modules/guardian/proto:guardian_py_pb2",
    		"//modules/monitor/proto:system_status_py_pb2",
    		"//modules/common/monitor_log/proto:monitor_log_py_pb2",
    		"//modules/dreamview/proto:hmi_status_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)





