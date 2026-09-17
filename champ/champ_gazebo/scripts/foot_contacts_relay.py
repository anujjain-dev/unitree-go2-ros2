#!/usr/bin/env python3
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Aggregate per-foot Ignition contact sensors into champ_msgs/ContactsStamped.

Gazebo Classic had champ_gazebo's contact_sensor node for this; it uses Classic
APIs and is disabled for Ignition Fortress (see CMakeLists.txt). Without a
publisher on /foot_contacts, champ's state_estimation node never fires its
joint_states/foot_contacts synchronizer, so /odom stays pinned at the origin and
SLAM and Nav2 have no usable odometry.

Each foot is bridged from Ignition as a ros_gz_interfaces/Contacts message. A
foot counts as down when its most recent message carried at least one contact
and arrived within contact_timeout seconds -- the staleness check matters
because the Ignition contact system may stop publishing entirely rather than
sending an empty message while a foot is in the air.

The output order is fixed to [lf, rf, lh, rh] to match champ::QuadrupedBase,
which registers its legs in that order.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from champ_msgs.msg import ContactsStamped
from ros_gz_interfaces.msg import Contacts

LEG_ORDER = ['lf', 'rf', 'lh', 'rh']


class FootContactsRelay(Node):

    def __init__(self):
        super().__init__('foot_contacts_relay')

        self.declare_parameter('contact_topics',
                               ['foot_contacts/' + leg for leg in LEG_ORDER])
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('contact_timeout', 0.1)

        topics = self.get_parameter('contact_topics').value
        rate = self.get_parameter('publish_rate').value
        self.contact_timeout = self.get_parameter('contact_timeout').value

        if len(topics) != len(LEG_ORDER):
            raise ValueError(
                'contact_topics must list exactly %d topics in %s order, got %d'
                % (len(LEG_ORDER), '/'.join(LEG_ORDER), len(topics)))

        # Per foot: whether the last message reported contact, and when it arrived.
        self.in_contact = [False] * len(LEG_ORDER)
        self.last_stamp = [None] * len(LEG_ORDER)

        # Ignition Fortress's Contact system ignores the sensor's <update_rate> and
        # publishes every physics step -- ~1 kHz per foot, ~4000 msg/s in total. A
        # reliable, depth-10 subscription makes this node queue and deserialise all of
        # it, which costs a full CPU core and starves the EKF enough to stall TF. Only
        # the newest sample per foot matters here, so let the middleware drop the rest.
        contact_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for i, topic in enumerate(topics):
            self.create_subscription(
                Contacts, topic,
                lambda msg, idx=i: self.contact_callback(msg, idx), contact_qos)
            self.get_logger().info('%s foot <- %s' % (LEG_ORDER[i], topic))

        self.publisher = self.create_publisher(ContactsStamped, 'foot_contacts', 10)
        self.create_timer(1.0 / rate, self.publish_contacts)

    def contact_callback(self, msg, idx):
        self.in_contact[idx] = len(msg.contacts) > 0
        self.last_stamp[idx] = self.get_clock().now()

    def publish_contacts(self):
        now = self.get_clock().now()
        out = ContactsStamped()
        out.header.stamp = now.to_msg()
        contacts = []
        for i in range(len(LEG_ORDER)):
            stamp = self.last_stamp[i]
            if stamp is None:
                contacts.append(False)
                continue
            age = (now - stamp).nanoseconds * 1e-9
            contacts.append(self.in_contact[i] and age < self.contact_timeout)
        out.contacts = contacts
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FootContactsRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
