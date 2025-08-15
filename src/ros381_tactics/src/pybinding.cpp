#include "ros381_tactics/global.hpp"
#include <pybind11/embed.h>

namespace py = pybind11;

PYBIND11_EMBEDDED_MODULE(ros381_tactics_py, m)
{
    py::class_<TacticGlobalNode, std::shared_ptr<TacticGlobalNode>>(m, "TacticGlobalNode")
        .def("send_goal", &TacticGlobalNode::send_goal)
        .def("cancel_goal", &TacticGlobalNode::cancel_goal)
        .def("update_pose", &TacticGlobalNode::update_pose)
        .def("publish_pose_offset", &TacticGlobalNode::publish_pose_offset)
        .def_readonly("move_result_", &TacticGlobalNode::move_result_)
        .def_readonly("update_pose_result_", &TacticGlobalNode::update_pose_result_);

    m.def(
        "get_node_instance",
        [](TacticGlobalNode *node) { return std::shared_ptr<TacticGlobalNode>(node, [](TacticGlobalNode *) {}); },
        py::return_value_policy::reference);
}

void init_python(TacticGlobalNode *node)
{
    try
    {
        std::string package_share = ament_index_cpp::get_package_share_directory("ros381_tactics");

        py::module sys = py::module::import("sys");
        sys.attr("path").attr("append")(package_share + "/scripts");
        sys.attr("path").attr("append")(package_share + "/../lib/python3.10/site-packages");

        node->tactics_module_ = new py::module(py::module::import("ros381_tactics.individual_tactics"));
        node->tactics_module_->attr("hello_tactics")();
        py::module::import("sys").attr("stdout").attr("flush")();

        auto embedded = py::module::import("ros381_tactics_py");
        node->tactics_module_->attr("node_instance") = embedded.attr("get_node_instance")(node);

        RCLCPP_INFO(node->get_logger(), "Python initialized successfully");
    }
    catch (const py::error_already_set &e)
    {
        RCLCPP_FATAL(node->get_logger(), "Python error: %s", e.what());
        rclcpp::shutdown();
    }
}