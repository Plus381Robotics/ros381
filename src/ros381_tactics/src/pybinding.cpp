#include "ros381_tactics/global.hpp"

namespace py = pybind11;

PYBIND11_EMBEDDED_MODULE(ros381_tactics_py, m)
{
    py::class_<TacticGlobalNode, std::shared_ptr<TacticGlobalNode>>(m, "TacticGlobalNode")
        .def("send_goal", &TacticGlobalNode::send_goal)
        .def("cancel_goal", &TacticGlobalNode::cancel_goal)
        .def("update_pose", &TacticGlobalNode::update_pose)
        .def("publish_pose_offset", &TacticGlobalNode::publish_pose_offset)
        .def("ax_move_goal", &TacticGlobalNode::ax_move_goal)
        .def("ax_hybrid_move_goal", &TacticGlobalNode::ax_hybrid_move_goal)
        .def("ax_bulk_move_goal", &TacticGlobalNode::ax_bulk_move_goal)
        .def("set_vacuum", &TacticGlobalNode::set_vacuum)
        .def("add_vacuum", &TacticGlobalNode::add_vacuum)
        .def("remove_vacuum", &TacticGlobalNode::remove_vacuum)
        .def_readonly("move_result_", &TacticGlobalNode::move_result_)
        .def_readonly("update_pose_result_", &TacticGlobalNode::update_pose_result_)
        .def_readonly("ax_move_result_", &TacticGlobalNode::ax_move_result_)
        .def_readonly("ax_hybrid_move_result_", &TacticGlobalNode::ax_hybrid_move_result_)
        .def_readonly("ax_hybrid_end_position_", &TacticGlobalNode::ax_hybrid_end_position_)
        .def_readonly("ax_bulk_move_result_", &TacticGlobalNode::ax_bulk_move_result_)
        .def_readonly("lift_front_id_", &TacticGlobalNode::lift_front_id_)
        .def_readonly("lift_back_id_", &TacticGlobalNode::lift_back_id_)
        .def_readonly("clan1_front_id_", &TacticGlobalNode::clan1_front_id_)
        .def_readonly("clan2_front_id_", &TacticGlobalNode::clan2_front_id_)
        .def_readonly("clan3_front_id_", &TacticGlobalNode::clan3_front_id_)
        .def_readonly("clan4_front_id_", &TacticGlobalNode::clan4_front_id_)
        .def_readonly("clan1_back_id_", &TacticGlobalNode::clan1_back_id_)
        .def_readonly("clan2_back_id_", &TacticGlobalNode::clan2_back_id_)
        .def_readonly("clan3_back_id_", &TacticGlobalNode::clan3_back_id_)
        .def_readonly("clan4_back_id_", &TacticGlobalNode::clan4_back_id_)
        .def_readonly("cursor_id_", &TacticGlobalNode::cursor_id_)
        .def_readonly("lift_up_pos_", &TacticGlobalNode::lift_up_pos_)
        .def_readonly("lift_down_pos_", &TacticGlobalNode::lift_down_pos_)
        .def_readonly("lift_carry_pos_", &TacticGlobalNode::lift_carry_pos_)
        .def_readonly("lift_rotating_pos_", &TacticGlobalNode::lift_rotating_pos_)
        .def_readonly("cursor_up_pos_", &TacticGlobalNode::cursor_up_pos_)
        .def_readonly("clanL_up_pos_", &TacticGlobalNode::clanL_up_pos_)
        .def_readonly("clanL_down_pos_", &TacticGlobalNode::clanL_down_pos_)
        .def_readonly("clanR_up_pos_", &TacticGlobalNode::clanR_up_pos_)
        .def_readonly("clanR_down_pos_", &TacticGlobalNode::clanR_down_pos_)
        .def_readonly("clanL_undep_pos_", &TacticGlobalNode::clanL_undep_pos_)
        .def_readonly("clanR_undep_pos_", &TacticGlobalNode::clanR_undep_pos_)
        .def_readwrite("cs_front_x", &TacticGlobalNode::cs_front_x)
        .def_readwrite("cs_front_y", &TacticGlobalNode::cs_front_y)
        .def_readwrite("cs_front_phi", &TacticGlobalNode::cs_front_phi)
        .def_readwrite("cs_back_x", &TacticGlobalNode::cs_back_x)
        .def_readwrite("cs_back_y", &TacticGlobalNode::cs_back_y)
        .def_readwrite("cs_back_phi", &TacticGlobalNode::cs_back_phi)
        .def_readwrite("cs_front_full", &TacticGlobalNode::cs_front_full)
        .def_readwrite("cs_back_full", &TacticGlobalNode::cs_back_full)
        .def_readonly("crates_back_", &TacticGlobalNode::crates_back_)
        .def_readonly("crates_front_", &TacticGlobalNode::crates_front_)
        .def_readwrite("consumed_front", &TacticGlobalNode::consumed_front_)
        .def_readwrite("consumed_back", &TacticGlobalNode::consumed_back_);

    m.def(
        "get_node_instance",
        [](TacticGlobalNode *node) { return std::shared_ptr<TacticGlobalNode>(node, [](TacticGlobalNode *) {}); },
        py::return_value_policy::reference);

    py::class_<AxMoveGoal>(m, "AxMoveGoal")
        .def(py::init<>())
        .def(py::init<const AxMoveGoal &>())
        .def_readwrite("id", &AxMoveGoal::id)
        .def_readwrite("position", &AxMoveGoal::position)
        .def_readwrite("velocity", &AxMoveGoal::velocity)
        .def_readwrite("position_tolerance", &AxMoveGoal::position_tolerance);

    // py::class_<std::vector<AxMoveGoal>>(m, "AxMoveGoalVector")
    //     .def(py::init<>())
    //     .def("clear", &std::vector<AxMoveGoal>::clear)
    //     .def("pop_back", &std::vector<AxMoveGoal>::pop_back)
    //     .def("push_back", &std::vector<AxMoveGoal>::push_back);
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