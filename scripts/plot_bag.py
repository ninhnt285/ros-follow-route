from pip import main
import rosbag
import matplotlib.pyplot as plt

if __name__ == "__main__":
  x = []
  y = []

  bag = rosbag.Bag("turtlebot.bag")
  for topic, msg, t in bag.read_messages(topics=['/odom']):
    position = msg.pose.pose.position
    x.append(position.x)
    y.append(position.y)
  bag.close()

  plt.plot(x, y)
  plt.show()
