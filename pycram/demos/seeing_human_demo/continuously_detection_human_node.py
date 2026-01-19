import rclpy
from pycram.external_interfaces import robokudo


def main():
    """Testing"""
    rclpy.init()
    # Send goal
    print(robokudo.query_current_human_postion_in_continues())
    print(robokudo.query_current_human_postion_in_continues())
    print(robokudo.query_current_human_postion_in_continues())

    # Close every think
    robokudo.shutdown_robokudo_interface()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
