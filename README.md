# autoware_memory_monitor
### 在这个项目中，我们有两个主要的部分：
1. ROS2的Python节点(ros2_memory_monitor_py)
2. Web应用(ros2_memory_monitor_web)

ros2_memory_monitor_py 

是一个ROS2的Python包，它包含一个节点，这个节点负责监控内存使用情况。

monitor.py 

文件是我们的主要脚本，它包含我们的ROS2节点和监控逻辑。这是你可以放置之前给你的Python监控代码的地方。

setup.py 和 setup.cfg 文件

是用来配置这个Python包的。你需要在setup.py中声明你的节点和依赖项（psutil和rclpy）。

ros2_memory_monitor_web 

是我们的Web应用。它包含一个Flask应用和一些静态文件（JavaScript，CSS，HTML模板等）。

app.py 

是Flask应用的主要文件，它定义了路由并启动了应用。
你需要在这里设置WebSocket，并把内存使用数据发送到客户端。

requirements.txt 文件列出了Web应用所需的Python库。
你至少需要包括flask和flask-socketio。

static/ 
目录包含静态文件，如JavaScript和CSS文件。你可以在这里放置你的Chart.js代码。

templates/ 

目录包含HTML模板。你需要在这里创建一个HTML文件，显示内存使用的图表。