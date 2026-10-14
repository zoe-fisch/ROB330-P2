
#!/usr/bin/env python3

from pathlib import Path
import sys

root = (
    Path(sys.argv[1]).expanduser().resolve()
    if len(sys.argv) > 1
    else Path.home() / 'P2/mbot_ros_labs/src/mbot_slam'
)

slam = root / 'src/slam_node.cpp'
cmake = root / 'CMakeLists.txt'
pkg = root / 'package.xml'

for p in (slam, cmake, pkg):
    if not p.is_file():
        raise SystemExit(f'Missing {p}')

a = slam.read_text()
b = cmake.read_text()
c = pkg.read_text()

if 'freeze_mapping' in a:
    raise SystemExit(
        'Freeze service already present; no changes made'
    )

replacements = [
    (
        '#include <rclcpp/rclcpp.hpp>',
        '#include <rclcpp/rclcpp.hpp>\n'
        '#include <std_srvs/srv/set_bool.hpp>'
    ),
    (
        '        // Subscriptions\n',
        '''        freeze_srv_ = create_service<std_srvs::srv::SetBool>(
            "/slam/freeze_mapping",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                mapping_frozen_ = request->data;
                response->success = true;
                response->message = mapping_frozen_
                    ? "Map frozen" : "Map resumed";
                RCLCPP_INFO(
                    get_logger(),
                    "%s",
                    response->message.c_str()
                );
            });

        // Subscriptions
'''
    ),
    (
        '    bool has_initialized_{false};',
        '''    bool has_initialized_{false};
    bool mapping_frozen_{false};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr freeze_srv_;'''
    ),
    (
        '            updateGrid(deskewed_scan);',
        '            if (!mapping_frozen_) updateGrid(deskewed_scan);'
    )
]

for old, new in replacements:
    if a.count(old) != 1:
        raise SystemExit(
            f'Unsafe patch: pattern count != 1: {old[:65]}'
        )
    a = a.replace(old, new)

if 'find_package(std_srvs REQUIRED)' not in b:
    if b.count('find_package(rclcpp REQUIRED)') != 1:
        raise SystemExit('CMake pattern mismatch')

    b = b.replace(
        'find_package(rclcpp REQUIRED)',
        'find_package(rclcpp REQUIRED)\n'
        'find_package(std_srvs REQUIRED)'
    )

    if b.count('set(DEPS rclcpp ') != 1:
        raise SystemExit('CMake dependencies mismatch')

    b = b.replace(
        'set(DEPS rclcpp ',
        'set(DEPS rclcpp std_srvs '
    )

if '<depend>std_srvs</depend>' not in c:
    if c.count('<depend>rclcpp</depend>') != 1:
        raise SystemExit('package.xml pattern mismatch')

    c = c.replace(
        '<depend>rclcpp</depend>',
        '<depend>rclcpp</depend>\n'
        '  <depend>std_srvs</depend>'
    )

for p in (slam, cmake, pkg):
    backup = p.with_suffix(
        p.suffix + '.competition-backup'
    )
    if backup.exists():
        raise SystemExit(
            f'Backup already exists: {backup}'
        )

for p, content in (
    (slam, a),
    (cmake, b),
    (pkg, c)
):
    backup = p.with_suffix(
        p.suffix + '.competition-backup'
    )
    backup.write_text(p.read_text())
    p.write_text(content)

print('Patched SLAM node and dependencies.')
print('Original files saved as *.competition-backup')
