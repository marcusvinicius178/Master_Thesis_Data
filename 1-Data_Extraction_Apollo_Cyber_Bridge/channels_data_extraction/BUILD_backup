load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])



py_binary(
    name = "corrected_imu",
    srcs = ["corrected_imu.py"],
    deps = [
    	"//modules/localization/proto:imu_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    	name = "odometry",
    	srcs = ["odometry.py"],
    	deps = [
    		"//modules/localization/proto:gps_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    	name = "channels_extraction",
    	srcs = ["channels_extraction.py"],
    	deps = [
    		"//modules/localization/proto:imu_py_pb2",
    		"//modules/localization/proto:gps_py_pb2",
        		"//cyber/python/cyber_py3:cyber",
    ],
)
