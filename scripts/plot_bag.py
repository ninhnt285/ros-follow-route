from pip import main
import rosbag
import matplotlib.pyplot as plt

if __name__ == "__main__":
  x = []
  y = []

  bag = rosbag.Bag("8_path.bag")
  for topic, msg, t in bag.read_messages(topics=['/odom']):
    position = msg.pose.pose.position
    x.append(position.x)
    y.append(position.y)
  bag.close()

  plt.figure(1, figsize = (5, 10))
  plt.plot(x, y)
  plt.show()
