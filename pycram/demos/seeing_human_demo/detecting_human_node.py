import rclpy
from pycram.external_interfaces import robokudo_v2

rclpy.init()
location = robokudo_v2.query_human()
if location is None:
    print("Error or No Human seen")
else:
    print(f"Human seen at: {location}")
robokudo_v2.shutdown_robokudo_interface()
rclpy.shutdown()
