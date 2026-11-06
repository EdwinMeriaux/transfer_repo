import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description='Prepare direct-force Gazebo world from an existing upstream BlueBoat installation')
parser.add_argument('--blueboat-model', type=Path, required=True, help='Upstream BlueBoat directory containing model.sdf and meshes')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--waves', choices=['calm', 'moderate', 'strong'], default='calm')
args = parser.parse_args()
if args.output.exists():
    raise FileExistsError(args.output)
source = args.blueboat_model.resolve()
model = ET.parse(source / 'model.sdf').getroot().find('model')
if model is None or model.attrib.get('name') != 'blueboat':
    raise ValueError('Expected upstream blueboat model')
for plugin in list(model.findall('plugin')):
    if plugin.get('name') == 'ArduPilotPlugin':
        model.remove(plugin)
if model.find("plugin[@filename='gz-waves1-hydrodynamics-system']") is None:
    raise ValueError('Expected asv_wave_sim hydrodynamics plugin')
for uri in model.findall('.//uri'):
    if 'blueboat/' not in uri.text:
        raise ValueError('Unexpected asset URI: ' + uri.text)
    uri.text = str(source / uri.text.split('blueboat/', 1)[1])
    if not Path(uri.text).exists():
        raise FileNotFoundError(uri.text)
odometry = model.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
ET.SubElement(odometry, 'odom_publish_frequency').text = '200'
ET.SubElement(odometry, 'odom_topic').text = '/model/blueboat/odometry'
root = ET.Element('sdf', version='1.9')
world = ET.SubElement(root, 'world', name='blueboat_transfer')
ET.SubElement(world, 'gravity').text = '0 0 -9.81'
physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
ET.SubElement(physics, 'max_step_size').text = '.005'
ET.SubElement(physics, 'real_time_factor').text = '0'
for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
    ET.SubElement(world, 'plugin', filename='gz-sim-' + filename + '-system', name='gz::sim::systems::' + name)
water = ET.SubElement(world, 'model', name='waves')
ET.SubElement(water, 'static').text = 'true'
plugin = ET.SubElement(water, 'plugin', filename='gz-waves1-waves-model-system', name='gz::sim::systems::WavesModel')
ET.SubElement(plugin, 'static').text = 'false'
ET.SubElement(plugin, 'update_rate').text = '200'
wave = ET.SubElement(plugin, 'wave')
amplitude, period, direction = {'calm': (0., 2.5, '.9396926207859084 .3420201433256687'), 'moderate': (.05, 2.5, '.8191520442889918 .573576436351046'), 'strong': (.1, 3., '.6427876096865394 .766044443118978')}[args.waves]
for name, value in dict(algorithm='sinusoid', tile_size='64', cell_count='64', amplitude=amplitude, period=period, number=1, steepness=0, direction=direction).items():
    ET.SubElement(wave, name).text = str(value)
link = ET.SubElement(water, 'link', name='water_visual')
visual = ET.SubElement(link, 'visual', name='water_surface')
plane = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
ET.SubElement(plane, 'normal').text = '0 0 1'
ET.SubElement(plane, 'size').text = '1000 1000'
material = ET.SubElement(visual, 'material')
for name in ('ambient', 'diffuse'):
    ET.SubElement(material, name).text = '.03 .2 .3 .6'
world.append(model)
args.output.parent.mkdir(parents=True, exist_ok=True)
ET.indent(root)
ET.ElementTree(root).write(args.output, encoding='utf-8', xml_declaration=True)
args.output.with_suffix('.json').write_text(json.dumps(dict(source_model_sha256=hashlib.sha256((source / 'model.sdf').read_bytes()).hexdigest(), waves=args.waves, source_assets='External upstream assets, not redistributed', ardupilot=False), indent=2) + '\n')
print(args.output)
