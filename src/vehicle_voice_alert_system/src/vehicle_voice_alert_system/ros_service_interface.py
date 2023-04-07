# -*- coding: utf-8 -*-
# Copyright (c) 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import threading

from autoware_adapi_v1_msgs.srv import AcceptStart

from std_srvs.srv import Trigger


class RosServiceInterface:
    def __init__(self, node):
        self._node = node
        self._created_client_dict = {}
        self._lock = threading.RLock()

        self._cli_accept_start = self.__create_client(AcceptStart, "/api/motion/accept_start")

    # service call function
    def accept_start(self):
        request = AcceptStart.Request()
        self.__service_call(self._cli_accept_start, request, True)

    # common denominator
    def __create_client(self, service_type, service_name):
        client = self._node.create_client(service_type, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("waiting for service: {}".format(service_name))
        self._created_client_dict[service_name] = client

        return client

    def __service_call(self, client, request, call_async=False):

        try:
            with self._lock:
                if not client.service_is_ready():
                    self._node.get_logger().error(
                        "service is unavailable, request: {}".format(str(request))
                    )
                    return

            if call_async:
                with self._lock:
                    client.call_async(request)
                self._node.get_logger().info("service sent with request: {}".format(str(request)))
                return True
            else:
                with self._lock:
                    future = client.call_async(request)

                while not future.done():
                    time.sleep(0.01)

                response = future.result()

                self._node.get_logger().info("service response: {}".format(response.status))
                return response
        except Exception as exception:
            self._node.get_logger().info("service call failed: {}".format(exception))
