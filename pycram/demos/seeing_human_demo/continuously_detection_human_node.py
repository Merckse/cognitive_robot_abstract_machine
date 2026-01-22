import rclpy
from pycram.external_interfaces import robokudo
from time import sleep
from pycram.ros_utils.text_to_image import TextToImagePublisher


def main():
    """Testing"""
    text_pub = TextToImagePublisher()
    found_position = True
    # While human is seen print location
    while found_position:
        # Send goal
        position = robokudo.query_current_human_position_in_continues()
        if position is not None:
            x = round(position.point.x)
            y = round(position.point.y)
            z = round(position.point.z)
            text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
        else:
            text_pub.publish_text("No Human seen.")
            found_position = False
        sleep(0.5)

    # Close every think
    robokudo.shutdown_robokudo_interface()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
