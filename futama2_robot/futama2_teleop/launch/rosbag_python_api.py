import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('rosbag2_2024_07_29-16_55_22', 'w')

try:
    s = String()
    s.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', s)
    bag.write('numbers', i)
finally:
    bag.close()