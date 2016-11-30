#id
string frame_id

#object type
uint8 RED_LED=0
uint8 GREEN_LED=1
uint8 BLUE_LED=2
uint8 BUTTON=3
uint8 DOOR=4

uint8 type

#Position relative to (camera's) frame_id 
geometry_msgs/Point  point
