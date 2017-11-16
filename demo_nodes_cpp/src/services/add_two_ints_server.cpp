// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

using srv_type = rcl_interfaces::srv::SetParameters;

void print_usage()
{
  printf("Usage for add_two_ints_server app:\n");
  printf("add_two_ints_server [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s service_name : Specify the service name for this server. Defaults to add_two_ints.\n");
}

void handle_add_two_ints(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<srv_type::Request> request,
  std::shared_ptr<srv_type::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
  //int idx = 1;
  std::cout << "Incoming request" << std::endl;
  std::cout << "Primitive value " << request->parameters[0].name << std::endl;
  std::cout << "Primitive value " << request->parameters[0].value.integer_value << std::endl;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = request->parameters[0].name + "_Response";
  result.successful = true;
  response->results.push_back(result);
  //test_msgs::msg::Primitives p;
  //p.int32_value = request->primitives[idx].int32_value + 1;
  //p.float32_value = request->primitives[idx].float32_value + 1.1;
  //p.float64_value = request->primitives[idx].float64_value + 2.2;
  //for (auto i = 0; i < idx+1; ++i) {
  //  response->primitives.push_back(p);
  //}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_server");

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  auto topic = std::string("add_two_ints");
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-s"));
  }
  auto server =
    node->create_service<srv_type>(topic, handle_add_two_ints);

  rclcpp::spin(node);

  return 0;
}
