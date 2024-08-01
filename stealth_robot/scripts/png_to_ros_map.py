import cv2
import os.path
 
#
#  This is a start for the map program
#

worldTemplate = """define block model
(
  size [0.5 0.5 0.5]
  gui_nose 0
)

define topurg ranger
(
  sensor (
    range [ 0 5.6 ]
    fov 180.0
    samples 500
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define pioneer position
(
  size [0.45 0.45 0.25]
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  topurg(pose [ 0.135 0 0.202 0 ])
)

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 1
)

# set the resolution of the underlying raytrace model in meters
resolution 0.05

interval_sim 100  # simulation timestep in milliseconds


window
( 
  size [ 1000.000 600.000 ] 

  rotate [ 0.000 0.000 ]
  scale 20.0
)

# load an environment bitmap
floorplan
( 
  name "{0}"
  bitmap "{1}.pgm"
  size [{2} {3} 0.5]
  pose [ 0 0 0 0 ]
)

# throw in a robot
pioneer( pose [ 0 0 0 0 ] name "robot" color "blue")
#block( pose [ -24.269 48.001 0 180.000 ] color "red")
"""


prompt = '> '
 
print("What is the name of your floor plan you want to convert to a ROS map:") 
file_name = input(prompt)
#
# Read in the image
#
image = cv2.imread(file_name)
#
# Some variables
#

font = cv2.FONT_HERSHEY_SIMPLEX
#
# mouse callback function
# This allows me to point and 
# it prompts me from the command line
#
def draw_point(event,x,y,flags,param):
      prompt = '> '
      print("Map width in (m): ")
      w = float(input(prompt))
      
      print("Map height in (m): ")
      h = float(input(prompt))

      dx = image.shape[1]*.05
      dy = image.shape[0]*.05
      #dx = dy = math.sqrt((image.shape[0])**2 + (image.shape[1])**2)*.05
      sy = h/dy 
      sx = w/dx

      print(sx, sy)


      res = cv2.resize(image, None, fx=sx, fy=sy, interpolation = cv2.INTER_CUBIC)
      # Convert to grey
      ret, res_thresh = cv2.threshold(res, 220, 255, cv2.THRESH_BINARY)
      final = cv2.cvtColor(res_thresh, cv2.COLOR_BGR2GRAY)

      cv2.imshow("Processed Image", final)

   

      prompt = '> '
      print("What is the name of the map?")
      mapName = input(prompt)
      print("Where should the map files be stored?")
      mapLocation = input(prompt)
      completeFileNameMap = os.path.join(mapLocation, mapName +".pgm")
      completeFileNameYaml = os.path.join(mapLocation, mapName +".yaml")
      completeFileNameWorld = os.path.join(mapLocation, mapName +".world")

      # write to the pgm file
      cv2.imwrite(completeFileNameMap, final );

      #write the yaml file 
      yaml = open(completeFileNameYaml, "w")
      yaml.write("image: " + mapName + ".pgm\n")
      yaml.write("resolution: 0.050000\n")
      yaml.write("origin: [" + str(0.00000) + "," +  str(0.00000) + ", 0.000000]\n")
      yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
      yaml.close()

      world = open(completeFileNameWorld, "w")
      world.write(worldTemplate.format(mapName, mapName, w, h))
      world.close()

      exit()
 
cv2.namedWindow('image', cv2.WINDOW_NORMAL)

cv2.setMouseCallback('image',draw_point)
#
#  Waiting for a Esc hit to quit and close everything
#
while(1):
  cv2.imshow('image',image)
  k = cv2.waitKey(20) & 0xFF
  if k == 27:
    break
  elif k == ord('a'):
    print('Done')
cv2.destroyAllWindows()

