import rospy
from sensor_msgs.msg import Image

FRONT = False


def image_callback(data):
    if FRONT:
        modified_image_topic = "/uav1/front_rgbd/color/modified/image_raw"
    else:     
        modified_image_topic = "/uav1/rgbd/color/modified/image_raw"
    
    modified_image_pub = rospy.Publisher(modified_image_topic, Image, queue_size=10)

    if FRONT:
        # uav1/front_rgbd/color_optical
        data.header.frame_id = "uav1/front_rgbd/aligned_depth_to_color_optical"
    else:
        # uav1/rgbd/color_optical
        data.header.frame_id = "uav1/rgbd/aligned_depth_to_color_optical"

    modified_image_pub.publish(data)
    print('republishing')

def republish_image():
    rospy.init_node('image_republisher', anonymous=True)
    
    if FRONT:
        original_image_topic = "/uav1/front_rgbd/color/image_raw"
    else:
        original_image_topic = "/uav1/rgbd/color/image_raw"

    rospy.Subscriber(original_image_topic, Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        republish_image()
    except rospy.ROSInterruptException:
        pass
