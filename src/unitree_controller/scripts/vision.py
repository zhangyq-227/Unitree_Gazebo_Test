#!/home/zhangyq227/miniconda3/envs/isaac/bin/python3.8
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from unitree_legged_msgs.msg import LowState
import torch
import torchvision
import cv2
import numpy as np
from cv_bridge import CvBridge

if not torch.cuda.is_available():
    print("must need cuda to compute vision!")
    exit(-1)

obs = None
depth_image = None
resize_transform = torchvision.transforms.Resize((58,87),interpolation=torchvision.transforms.InterpolationMode.BICUBIC)

# make observations
def lowstate_callback(msg):
    global obs
    prop = np.array(msg.data,dtype=np.float)
    prop = torch.from_numpy(prop).to(dtype=torch.float).to(device='cuda:0')
    obs = prop

def depth_callback(msg):
    global depth_image
    image = msg.data
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    image = torch.from_numpy(image.astype(np.float)).to(device='cuda:0').to(dtype=torch.float)
    image = image[:-16,32:-32]
    image = image/1000.0
    image = torch.clip(image,min=0,max=2)
     
    image = resize_transform(image[None,:]).squeeze() 
    image /= 2.0 
    cv2.imshow("Depth Image", image.to(device='cpu').numpy())
    # wait [int] milliseconds
    cv2.waitKey(1)

    depth_image = image


if __name__ == '__main__':
    try:
        rospy.init_node('vision',anonymous=True)
        lowstate_sub = rospy.Subscriber("/prop_obs",Float64MultiArray, lowstate_callback, queue_size=1)
        depth_sub = rospy.Subscriber("/d435/depth/image_raw", Image, depth_callback,queue_size=1)
        vision_latent_pub = rospy.Publisher("/vision_latent",Float32MultiArray,queue_size=1)
        rate = rospy.Rate(10)
        model = torch.jit.load("/home/zhangyq227/code/gazebo_sim2sim/models/Vision_Deploy.pt",map_location='cuda:0')
        model.eval()

        #  compute vision at 10Hz.
        while not rospy.is_shutdown():
            if depth_image is not None and obs is not None: 
                with torch.inference_mode():
                    result = model(depth_image.unsqueeze(dim=0), obs.unsqueeze(dim=0))
                    result = result.squeeze().to(device='cpu').tolist()

                    send_data = Float32MultiArray()

                    send_data.data = result

                    vision_latent_pub.publish(send_data)
            # rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        print("can not start rospy")
        pass
