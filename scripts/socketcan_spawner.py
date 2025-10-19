#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import roslaunch

"""
Read buses from ~socketcan_auto_cfg/buses and spawn one socketcan_bridge_node per entry
Requires each bus to define: can_device, rx_topic, tx_topic
"""

def main():
    rospy.init_node('socketcan_spawner')
    # buses are loaded under /socketcan_auto_cfg/buses by the launch file
    buses = rospy.get_param('/socketcan_auto_cfg/buses', [])
    if not isinstance(buses, list):
        rospy.logerr('buses must be a list in YAML')
        return

    # collect unique device/topic triples from per-actuator entries
    devices = []
    seen = set()
    for group in buses:
        acts = group.get('actuators', []) if isinstance(group, dict) else []
        for a in acts:
            can_dev = a.get('can_device')
            rx = a.get('rx_topic') or ('/%s_rx' % can_dev if can_dev else None)
            tx = a.get('tx_topic') or ('/%s_tx' % can_dev if can_dev else None)
            if not (can_dev and rx and tx):
                rospy.logwarn('skip actuator without can_device/rx/tx in spawner')
                continue
            key = (can_dev, rx, tx)
            if key in seen: continue
            seen.add(key)
            devices.append({'can_device': can_dev, 'rx': rx, 'tx': tx})

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    parent = roslaunch.parent.ROSLaunchParent(uuid, [])
    parent.start()

    processes = []
    for idx, d in enumerate(devices):
        can_dev = d['can_device']; rx = d['rx']; tx = d['tx']
        node = roslaunch.core.Node(
            package='socketcan_bridge',
            node_type='socketcan_bridge_node',
            name='socketcan_auto_%d' % idx,
            output='screen',
            args='',
            remap_args=[('can_rx', rx), ('can_tx', tx)],
            roslaunch_args=[]
        )
        # Inject param can_device
        node.params = {'can_device': can_dev}
        proc = parent.launch(node)
        processes.append(proc)
        rospy.loginfo('spawned socketcan node %s for %s (%s, %s)', node.name, can_dev, rx, tx)

    rospy.on_shutdown(lambda: [p.stop() for p in processes])
    rospy.spin()

if __name__ == '__main__':
    main()
