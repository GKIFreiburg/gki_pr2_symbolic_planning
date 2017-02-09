#!/usr/bin/env python
import roslib
import rospy
import rosbag

table_name = 'table1'
with rosbag.Bag('{}.bag'.format(table_name), 'w') as outbag:
	for topic, msg, t in rosbag.Bag('{}_original.bag'.format(table_name)).read_messages():
		for pose in msg.mesh_poses:
			pose.position.x -= 3.1
			pose.position.y -= 7.65
		outbag.write(topic, msg)
table_name = 'table2'
with rosbag.Bag('{}.bag'.format(table_name), 'w') as outbag:
	for topic, msg, t in rosbag.Bag('{}_original.bag'.format(table_name)).read_messages():
		for pose in msg.mesh_poses:
			pose.position.x -= 3.05
			pose.position.y -= 6.65
		msg.header.frame_id = 'map'
		outbag.write(topic, msg)
