#!/usr/bin/env python
# Authors: Marcey and Daniel
#
import os
import sys
import cv2
import rospy
import click
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def load_image(filename):
  img = cv2.imread(filename, -1)
  if img == None:
    rospy.logerr("ERROR image file didn't load or doesn't exist: %s" % filename)
    exit(1)
  return img

def resize_image(img, resize_percentage=None, min_width_px=None):
  if min_width_px:
    # # resize img to at least px pxiels wide
    resize_percentage = float(min_width_px)/img.shape[1]

  img = cv2.resize(img, (int(resize_percentage*img.shape[1]), int(resize_percentage*img.shape[0])), interpolation = cv2.INTER_CUBIC)
  return img

def rotate_image(img, rotation_angle):
  # Rotate compass based on heading
  num_rows, num_cols = img.shape[:2]
  rotation_matrix = cv2.getRotationMatrix2D((int(num_rows/2), int(num_cols/2)),rotation_angle,1)
  img = cv2.warpAffine(img, rotation_matrix, (num_cols, num_rows))
  return img

def warp_image(img):
  # Warp compass in 3D so it looks like its pointing into the image
  compass_width = int(img.shape[1])
  compass_height = int(img.shape[0])
  img1_square_corners = np.float32([[0,0], [compass_height,0], [compass_height,compass_width],[0,compass_width]])
  scale = 0.33
  img_quad_corners = np.float32([[int(153*scale),0], [int(429*scale),0], [int(600*scale),int(286*scale)], [0,int(286*scale)]]) # TODO: Set this sizes properly using a scale factor and image size (NOT static)
  # !!!!!!!!!!!!!!!!!!!!!
  
  h, mask = cv2.findHomography(img1_square_corners, img_quad_corners)
  warped_img = cv2.warpPerspective(img, h, (int(600*scale),int(286*scale)))
  return warped_img

def overlay_image(fg_img, bg_img, x, y):
  # Overlay a background image onto a foreground image (with transparency support)
  for c in range(0,3):
      # pseudo code explaination: bg_img[where the overlay is going to be placed] = (overlay_image[all pixcels] * transparency) + (original_image * transparency)
      bg_img[y:y+fg_img.shape[0], x:x+fg_img.shape[1], c] = fg_img[:,:,c] * (fg_img[:,:,3]/255.0) + bg_img[y:y+fg_img.shape[0], x:x+fg_img.shape[1], c] * (1.0 - fg_img[:,:,3]/255.0)
  return bg_img


class ImageOverlay():
  def __init__(self):
    self.scale_text = 0
    self.heading = 0
    self.img = np.zeros((700,1000,3),dtype=np.uint8) # default black background

    current_dir = os.path.dirname(os.path.abspath(__file__))
    compass_filename = '%s/compass.png' % current_dir
    scale_filename = '%s/scale.png' % current_dir

    # Load scale image
    scale_img = load_image(scale_filename)
    self.scale_img = resize_image(scale_img, min_width_px=300)

    # Load compass image
    compass_img = load_image(compass_filename)
    self.compass_img = resize_image(compass_img, resize_percentage=0.33)

  def image_callback(self, data):
    img_cv = CvBridge().compressed_imgmsg_to_cv2(data)
    self.img = resize_image(min_width_px=1000)

  def heading_callback(self, data):
    self.heading = data.data

  def scale_callback(self, data):
    self.scale_text = data.data

  def overlay_telemetry(self):
    im_out = self.img.copy()

    compass_img = self.compass_img
    compass_img = rotate_image(compass_img, self.heading)
    compass_img = warp_image(compass_img)
    
    # Overlay compass image
    x = im_out.shape[1] / 2 - compass_img.shape[1] / 2 # center width position
    y = im_out.shape[0] - int(compass_img.shape[0]) - 1 # near bottom
    im_out = overlay_image(compass_img, im_out, x, y)

    # Overlay scale image
    scale_img = self.scale_img
    x = im_out.shape[1] / 2 - scale_img.shape[1] / 2 # center width position
    y = im_out.shape[0] / 2 # # center height position
    im_out = overlay_image(scale_img, im_out, x, y)
    
    # Set text overlay near scale_img indicating the scale value
    font = 2
    scale = 1
    thick = 2
    line_type = 4
    color = (255,255,255)
    # Set text at the small end of the scale bar
    cv2.putText(im_out, "0", (x-25,y+16), font, scale, color, thick, line_type)
    # Set text at the large end of the scale bar
    scale_text_pretty = "%s cm" % self.scale_text
    cv2.putText(im_out, scale_text_pretty, (x+scale_img.shape[1]+5,y+16), font, scale, color, thick, line_type)

    ## Preview result
    # cv2.imshow("Image", im_out)
    # cv2.waitKey(0)
    # cv2.destroyAllWindo()

    return im_out

  def ros_loop(self):
    overlay_publisher = rospy.Publisher('/science/overlay/compressed', CompressedImage, queue_size=1)
    rospy.Subscriber(rospy.get_param('~image_topic', '/camera/rgb_fixed/compressed'), CompressedImage, self.image_callback)

    if not self.heading:
      rospy.Subscriber(rospy.get_param('~heading_topic','/gps/heading'), Float32, self.heading_callback)
    if not self.scale_text:
      rospy.Subscriber(rospy.get_param('~scale_topic', '/science/scale'), Float32, self.scale_callback)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      img = self.overlay_telemetry()
      compressed_image = CvBridge().cv2_to_compressed_imgmsg(img)
      overlay_publisher.publish(compressed_image)
      rate.sleep()

@click.command()
@click.option('--input-image', required=True, help='Input image file')
@click.option('--heading', required=True, type=float, help='Current heading relative to north')
@click.option('--scale_text', required=True, type=float, help='The maximum value on the scale bar in centemeters')
@click.option('--output-file', default='output.png', help='Save to file rather than publish over ROS')
def cli_main(input_image, heading, scale_text, output_file):
  # The CLI entry point to this code
  imgOverlay = ImageOverlay()
  imgOverlay.heading = heading
  imgOverlay.scale_text = scale_text
  imgOverlay.img = load_image(input_image)
  img = imgOverlay.overlay_telemetry()
  cv2.imwrite(output_file,img)

def ros_main():
  # The ROS entry point to this code
  rospy.init_node('overlay_scale_and_compass')
  imgOverlay = ImageOverlay()
  imgOverlay.ros_loop()

if __name__ == '__main__':
  cli_main()
