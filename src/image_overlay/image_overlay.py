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


def is_ros_node():
  return 'unnamed' not in rospy.get_name()

# Save the original rospy.log* functions for when ROS is enabled. This script also supports alternative use by command line and without ROS. For convienence I am overriding rospy.loginfo to support both use cases
rospy_loginfo = rospy.loginfo
def loginfo(message):
  if is_ros_node():
    rospy_loginfo(message)
  else:
    click.echo(click.style(str(message), fg='green'))
rospy.loginfo = loginfo

rospy_logerr = rospy.logerr
def logerr(message):
  if is_ros_node():
    rospy_logerr(message)
  else:
    click.echo(click.style(str(message), fg='red'))
rospy.logerr = logerr

def load_image(filename):
  img = cv2.imread(filename, -1)
  if img is None:
    rospy.logerr("ERROR image file didn't load or doesn't exist: %s" % filename)
    exit(1)
  return img

def resize_image(img, width_px):
  """Args:
       img: The image to be resized.
       width_px: The new width of the image in pixels relative to itself. Aspect ratio is not changed.
  """
  # we need to keep in mind aspect ratio so the image does
  # not look skewed or distorted -- therefore, we calculate
  # the ratio of the new image to the old image
  r = float(width_px) / img.shape[1]
  dimensions = (int(width_px), int(img.shape[0] * r))
  img = cv2.resize(img, dimensions, interpolation = cv2.INTER_CUBIC)
  return img

def rotate_image(img, rotation_angle):
  num_rows, num_cols = img.shape[:2]
  rotation_matrix = cv2.getRotationMatrix2D((int(num_rows/2), int(num_cols/2)),rotation_angle,1)
  img = cv2.warpAffine(img, rotation_matrix, (num_cols, num_rows))
  return img

def warp_image(img):
  """Warp image in 3D so it looks like its pointing into the image"""
  compass_width = int(img.shape[1])
  compass_height = int(img.shape[0])
  square_corners = np.float32([[0,0], [compass_height,0], [compass_height,compass_width],[0,compass_width]])
  quad_corners = np.float32([[153,0], [429,0], [600,286], [0,286]])
  h, mask = cv2.findHomography(square_corners, quad_corners)
  warped_img = cv2.warpPerspective(img, h, (600,286))
  return warped_img

def overlay_image(fg_img, bg_img, x, y):
  """Overlay a background image onto a foreground image (with transparency support)"""
  for c in range(0,3):
      # pseudo code explaination: bg_img[where the overlay is going to be placed] = (overlay_image[all pixcels] * transparency) + (original_image * transparency)
      bg_img[y:y+fg_img.shape[0], x:x+fg_img.shape[1], c] = fg_img[:,:,c] * (fg_img[:,:,3]/255.0) + bg_img[y:y+fg_img.shape[0], x:x+fg_img.shape[1], c] * (1.0 - fg_img[:,:,3]/255.0)
  return bg_img


class ImageOverlay():
  def __init__(self):
    self.scale_text = 0
    self.heading = 0
    self.img = np.ones((700,1000,3),dtype=np.uint8)*128 # default black background

    current_dir = os.path.dirname(os.path.abspath(__file__))
    compass_filename = '%s/compass.png' % current_dir
    scale_filename = '%s/scale.png' % current_dir

    # Load scale image
    self.scale_img = load_image(scale_filename)

    # Load compass image
    self.compass_img = load_image(compass_filename)

  def image_callback(self, data):
    self.img = CvBridge().compressed_imgmsg_to_cv2(data)

  def heading_callback(self, data):
    self.heading = data.data

  def scale_callback(self, data):
    self.scale_text = data.data

  def overlay_telemetry(self):
    im_out = np.uint8(self.img.copy())
    min_image_dimension = np.min([self.img.shape[1],self.img.shape[0]])

    compass_img = self.compass_img
    compass_img = rotate_image(compass_img, self.heading)
    compass_img = warp_image(compass_img)

    # Resize compass image
    width_percentage = 0.6
    width_px = width_percentage*min_image_dimension
    compass_img = resize_image(compass_img, width_px)

    # Resize scale image
    scale_img = resize_image(self.scale_img, width_px)

    # Overlay compass image
    x = im_out.shape[1] / 2 - compass_img.shape[1] / 2 # center width position
    y = im_out.shape[0] - int(compass_img.shape[0]) - 1 # near bottom
    im_out = overlay_image(compass_img, im_out, x, y)

    # Overlay scale image
    x = im_out.shape[1] / 2 - scale_img.shape[1] / 2 # center width position
    y = im_out.shape[0] / 2 # # center height position
    im_out = overlay_image(scale_img, im_out, x, y)
    
    # Set text overlay near scale_img indicating the scale value
    scale_factor = (min_image_dimension / 350.0) # scale font relative to img
    font = 2
    scale = 0.5 * scale_factor
    thick = 1 * scale_factor
    line_type = 4

    color = tuple(np.ones(im_out.shape[2])*255) # white text, ie. same number of 255s and number of image channels
    # Set text at the small end of the scale bar
    loc = (int(x-13*scale_factor),int(y+7*scale_factor))
    cv2.putText(im_out, "0", loc, font, scale, color, int(thick), line_type)
    # Set text at the large end of the scale bar
    scale_text_pretty = "%.1f cm" % self.scale_text
    loc = (x+scale_img.shape[1]+5,int(y+7*scale_factor))
    cv2.putText(im_out, scale_text_pretty, loc, font, scale, color, int(thick), line_type)

    ## Preview result
    # cv2.imshow("Image", im_out)
    # cv2.waitKey(0)
    # cv2.destroyAllWindo()

    return im_out

  def ros_loop(self):
    publisher = rospy.Publisher('overlay/compressed', CompressedImage, queue_size=1)
    rospy.Subscriber('camera/compressed', CompressedImage, self.image_callback, queue_size=1)

    if not self.heading:
      rospy.Subscriber('heading', Float32, self.heading_callback, queue_size=1)
    if not self.scale_text:
      rospy.Subscriber('scale', Float32, self.scale_callback, queue_size=1)

    rospy.loginfo('Setup complete. Image overlay process has started.')

    framerate = rospy.Rate(rospy.get_param('~framerate',3)) # default 3 Hz
    while not rospy.is_shutdown():
      img = self.overlay_telemetry()
      compressed_image = CvBridge().cv2_to_compressed_imgmsg(img)
      publisher.publish(compressed_image)
      framerate.sleep()

def ros_main():
  # The ROS entry point to this code
  rospy.init_node('overlay_scale_and_compass')
  imgOverlay = ImageOverlay()
  imgOverlay.ros_loop()

# Command line options
@click.command()
@click.option('--input-image', required=True, help='Path to input image file')
@click.option('--heading', required=True, type=float, help='Current heading relative to north in degrees')
@click.option('--scale-text', required=True, type=float, help='The value to be displayed on the right of the scale bar in centimeters')
@click.option('--output-file', default='output.png', help='Output filename to save result to  [default=\'output.png\']')
def cli_main(input_image, heading, scale_text, output_file):
  # The CLI entry point to this code
  imgOverlay = ImageOverlay()
  imgOverlay.heading = heading
  imgOverlay.scale_text = scale_text
  imgOverlay.img = load_image(input_image)
  img = imgOverlay.overlay_telemetry()
  cv2.imwrite(output_file,img)

if __name__ == '__main__':
  cli_main()
