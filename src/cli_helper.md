## Move to XY action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: 1, x: 1.0, y: 0.5, phi: 0.0, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Move on Direction action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: 2, x: 1.0, y: 1.0, phi: 0.0, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Move on Direction Snapped action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: 3, x: 1.0, y: 1.0, phi: 1.57, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Move on Angle action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: 4, x: 0.5, y: 1.0, phi: 0.73, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Rot to Phi action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: -1, x: 1.0, y: 0.5, phi: 3.14, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Rot to XY action
ros2 action send_goal /move ros381_interfaces/action/Move "{type: -2, x: 1.0, y: 0.5, phi: 3.14, direction: 1, v_max: 2.0, w_max: 12.6, distance_tolerance_percentage: 1.0, angle_tolerance_percentage: 1.0, start_coeff_v: 1.0, start_coeff_w: 1.0, stop_coeff_v: 1.0, stop_coeff_w: 1.0}"
## Trigger Chich
ros2 topic pub --once /ros381/chinch_trigger example_interfaces/msg/Bool "{data: true}"
## Update Pose
ros2 service call /service_name example_srv/YourSrvName "{x: 1.0, y: 2.0, phi: 0.785, type: 1}"