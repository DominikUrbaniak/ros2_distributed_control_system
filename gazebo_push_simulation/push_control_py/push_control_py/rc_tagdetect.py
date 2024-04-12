# Copyright 2020 Roboception GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy

from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rc_reason_msgs.srv import DetectTags
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
#from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rc_reason_clients.rest_client import RestClient
import time

def tag_to_tf(tag):
    tf = TransformStamped()
    tf.header.frame_id = tag.header.frame_id
    tf.child_frame_id = f"{tag.tag.id}_{tag.instance_id}"
    tf.header.stamp = tag.header.stamp
    tf.transform.translation.x = tag.pose.pose.position.x
    tf.transform.translation.y = tag.pose.pose.position.y
    tf.transform.translation.z = tag.pose.pose.position.z
    tf.transform.rotation = tag.pose.pose.orientation
    return tf

'''qos_profile_R1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)'''

class TagClient(RestClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)

        # client only parameters
        self.declare_parameter(
            "publish_tf",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish detected tags via TF"
            )
        )
        self.declare_parameter(
            "publish_markers",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish detected tags as visalization markers"
            )
        )
        self.tag_markers = []

        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
        self.pub_markers = self.create_publisher(MarkerArray, "visualization_marker_array", QoSProfile(depth=10))

        self.start()

        self.add_rest_service(DetectTags, 'detect', self.detect_callback)

    def start(self):
        self.get_logger().info(f"starting {self.rest_name}")
        self.call_rest_service('start')

    def stop(self):
        self.get_logger().info(f"stopping {self.rest_name}")
        self.call_rest_service('stop')

    def detect_callback(self, srv_name, request, response):
        start_time = time.time()
        self.call_rest_service(srv_name, request, response)
        self.get_logger().info(f"calling rest-service latency: {time.time()-start_time}")
        #self.publish_tags(response.tags)
        return response

    def publish_tags(self, tags):
        if tags and self.get_parameter('publish_tf').value:
            transforms = [tag_to_tf(tag) for tag in tags]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.get_parameter('publish_markers').value:
            self.publish_tag_markers(tags)

    def publish_tag_markers(self, tags):
        def create_marker(tag, id):
            m = Marker(action=Marker.ADD, type=Marker.CUBE)
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
            m.header.stamp = tag.pose.header.stamp
            m.header.frame_id = f"{tag.tag.id}_{tag.instance_id}"
            m.pose.orientation.w = 1.0
            m.pose.position.x = tag.tag.size / 2
            m.pose.position.y = tag.tag.size / 2
            m.pose.position.z = 0.001 / 2
            m.scale.x = tag.tag.size
            m.scale.y = tag.tag.size
            m.scale.z = 0.001
            m.id = id
            m.ns = f"{self.rest_name}_tags"
            return m

        new_markers = []
        for i, tag in enumerate(tags):
            m = create_marker(tag, i)
            if i < len(self.tag_markers):
                self.tag_markers[i] = m
            else:
                self.tag_markers.append(m)
            new_markers.append(m)
        for i in range(len(tags), len(self.tag_markers)):
            # delete old markers
            self.tag_markers[i].action = Marker.DELETE
        self.pub_markers.publish(MarkerArray(markers=self.tag_markers))
        self.tag_markers = new_markers


def main(args=None, rest_node='rc_april_tag_detect'):
    rclpy.init(args=args)

    client = TagClient(rest_node)

    rclpy.get_default_context().on_shutdown(client.stop)

    host = client.get_parameter('host').value
    if not host:
        client.get_logger().error('host is not set')
        rclpy.shutdown()
        exit(1)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


def rc_april_tag_detect_client(args=None):
    main(rest_node='rc_april_tag_detect')


def rc_qr_code_detect_client(args=None):
    main(rest_node='rc_qr_code_detect')


if __name__ == '__main__':
    main()
