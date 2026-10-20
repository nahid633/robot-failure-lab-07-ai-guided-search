"""Common instrumentation; fixed parked obstacle is moved by a verified service."""
from core import OBSTACLE_SIZE

POSE_PLUGIN='''<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
<publish_link_pose>false</publish_link_pose><publish_visual_pose>false</publish_visual_pose>
<publish_collision_pose>false</publish_collision_pose><publish_sensor_pose>false</publish_sensor_pose>
<publish_model_pose>true</publish_model_pose><publish_nested_model_pose>false</publish_nested_model_pose>
<use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>30</update_frequency></plugin>'''


def instrument_world(world):
    sx,sy,sz=OBSTACLE_SIZE
    extra=f'''<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
<model name="rfl05_obstacle"><static>true</static><pose>-8 -8 .35 0 0 0</pose>
<link name="body"><collision name="obstacle_collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="obstacle_visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material><diffuse>0.9 0.2 0.1 1</diffuse></material></visual>
<sensor name="obstacle_contacts" type="contact"><always_on>true</always_on><update_rate>50</update_rate><topic>/rfl05/contacts</topic>
<contact><collision>obstacle_collision</collision><topic>/rfl05/contacts</topic></contact></sensor></link>{POSE_PLUGIN}</model>'''
    assert world.count('</world>')==1
    return world.replace('</world>',extra+'\n</world>')
