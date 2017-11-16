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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

using srv_type = rcl_interfaces::srv::SetParameters;
using srv_type_request = rcl_interfaces::srv::SetParameters_Request;

void
fill_request(std::shared_ptr<srv_type::Request> request)
{
  (void) request;
  rcl_interfaces::msg::Parameter p;
  p.name = "my_parameter_name";
  rcl_interfaces::msg::ParameterValue pv;
  pv.type = 2;
  pv.integer_value = 13;
  p.value = pv;
  request->parameters.push_back(p);
  //test_msgs::msg::Primitives p;
  //p.int32_value = 32;
  //p.float32_value = 32.32f;
  //p.float64_value = 64.64f;
  //int idx = 1;
  //for (auto i = 0; i < idx+1; ++i) {
  //  request->primitives.push_back(p);
  //}
}

void
print_response(const std::shared_ptr<srv_type::Response> response)
{
  (void) response;
  std::cout << "Incoming response" << std::endl;
  std::cout << "Primitive value " << response->results[0].reason << std::endl;
}

void print_usage()
{
  printf("Usage for add_two_ints_client app:\n");
  printf("add_two_ints_client [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s service_name : Specify the service name for this client. Defaults to add_two_ints.\n");
}

// TODO(wjwwood): make this into a method of rclcpp::client::Client.
std::shared_ptr<srv_type::Response>
send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::client::Client<srv_type>::SharedPtr client,
  std::shared_ptr<srv_type::Request> request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  auto topic = std::string("add_two_ints");
  if (rcutils_cli_option_exist(argv, argv + argc, "-s")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-s"));
  }
  auto client = node->create_client<srv_type>(topic);

  auto request = std::make_shared<srv_type::Request>();
  fill_request(request);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf("add_two_ints_client was interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    printf("service not available, waiting again...\n");
  }

  // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
  // TODO(wjwwood): consider error condition
  auto response = send_request(node, client, request);
  if (response) {
    print_response(response);
  } else {
    printf("add_two_ints_client was interrupted. Exiting.\n");
  }

  rclcpp::shutdown();
  return 0;
}
