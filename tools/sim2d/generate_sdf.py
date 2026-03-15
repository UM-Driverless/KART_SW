#!/usr/bin/env python3
"""Generate Gazebo SDF world file from track.py cone definitions.

Usage:
    python generate_sdf.py autocross > ../../src/kart_sim/worlds/autocross_track.sdf
    python generate_sdf.py hairpin   > ../../src/kart_sim/worlds/hairpin_track.sdf
    python generate_sdf.py "random:seed=42" > random_track.sdf
    python generate_sdf.py "random:seed=42" --save-track random42.json > random42.sdf
"""

import argparse
import sys

from track import get_track, track_to_json

SDF_HEADER = """\
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world_name}">

    <physics name="sim" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"></plugin>

    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.4 0.6 0.9 1</background>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include><uri>model://kart</uri><name>kart</name><pose>{spawn_x:.4f} {spawn_y:.4f} 0 0 0 {spawn_yaw:.4f}</pose></include>
"""

CONE_LINE = '    <include><uri>model://{model}</uri><name>{name}</name><pose>{x:.4f} {y:.4f} {z} 0 0 0</pose></include>'

SDF_FOOTER = """
    <gui fullscreen="0"><camera name="user_camera"><pose>{cam_x:.1f} {cam_y:.1f} {cam_z:.1f} 0 1.5708 0</pose></camera></gui>

  </world>
</sdf>
"""


def generate(track_name: str) -> str:
    track = get_track(track_name)
    world_name = f"{track.name}_track"
    lines = []

    lines.append(SDF_HEADER.format(
        world_name=world_name,
        spawn_x=track.spawn_x,
        spawn_y=track.spawn_y,
        spawn_yaw=track.spawn_yaw,
    ))

    lines.append('    <!-- Blue cones (left of travel) -->')
    for i, (x, y) in enumerate(track.blue_cones):
        lines.append(CONE_LINE.format(model='cone_blue', name=f'blue_{i:02d}', x=x, y=y, z='0.1517'))

    lines.append('')
    lines.append('    <!-- Yellow cones (right of travel) -->')
    for i, (x, y) in enumerate(track.yellow_cones):
        lines.append(CONE_LINE.format(model='cone_yellow', name=f'yellow_{i:02d}', x=x, y=y, z='0.1517'))

    lines.append('')
    lines.append('    <!-- Orange big cones at start/finish -->')
    for i, (x, y) in enumerate(track.orange_cones):
        lines.append(CONE_LINE.format(model='cone_orange_big', name=f'orange_{i:02d}', x=x, y=y, z='0.252'))

    # Camera centered above track, high enough to see everything
    all_x = list(track.blue_cones[:, 0]) + list(track.yellow_cones[:, 0])
    all_y = list(track.blue_cones[:, 1]) + list(track.yellow_cones[:, 1])
    cx = (min(all_x) + max(all_x)) / 2
    cy = (min(all_y) + max(all_y)) / 2
    span = max(max(all_x) - min(all_x), max(all_y) - min(all_y))
    cam_z = span * 1.0  # high enough to see entire track
    lines.append(SDF_FOOTER.format(cam_x=cx, cam_y=cy, cam_z=cam_z))

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='Generate Gazebo SDF from track.py')
    parser.add_argument('track',
                        help='Track: built-in name (oval, hairpin, autocross), '
                             'JSON path, or random spec (random:seed=42)')
    parser.add_argument('--save-track', metavar='PATH',
                        help='Save the track definition to a JSON file')
    args = parser.parse_args()

    if args.save_track:
        track = get_track(args.track)
        track_to_json(track, args.save_track)
        print(f"Saved track → {args.save_track}", file=sys.stderr)

    print(generate(args.track))


if __name__ == '__main__':
    main()
