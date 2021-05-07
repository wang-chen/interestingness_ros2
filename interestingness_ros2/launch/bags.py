import os
basepath='/home/li/CMU_RISS'
SubT0=[basepath+'/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-32-27_0.bag',
basepath+'/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-34-13_1.bag',
basepath+'/SubT/0817-ugv0-tunnel0/objdet_2019-08-17-16-35-56_2.bag'
]
for subt in SubT0:
    os.system('ros2 bag play -s rosbag_v2 '+subt)