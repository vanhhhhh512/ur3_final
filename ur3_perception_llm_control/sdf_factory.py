"""Element-tree scene serialization using the existing physical dimensions."""
import itertools
import xml.etree.ElementTree as xml


def child(parent, tag, value=None, **attributes):
    element = xml.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def sequence(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def geometry(parent, dimensions):
    child(child(child(parent, 'geometry'), 'box'), 'size', sequence(dimensions))


def material(parent, rgba):
    paint = child(parent, 'material')
    for key in ('ambient', 'diffuse'):
        child(paint, key, sequence(rgba))


def serialize_box(name, row, role):
    document = xml.Element('sdf', version='1.7')
    body = child(document, 'model', name=name)
    dynamic = role == 'object'
    child(body, 'static', str(not dynamic).lower())
    link = child(body, 'link', name='link')
    child(link, 'gravity', 'true')
    dimensions = tuple(row['size'][axis] for axis in 'xyz')
    rgba = tuple(row['color'][axis] for axis in 'rgba')
    if dynamic:
        inertia = child(link, 'inertial')
        mass = row['mass']
        child(inertia, 'mass', mass)
        tensor = child(inertia, 'inertia')
        for axis, pair in zip('xyz', ((1, 2), (0, 2), (0, 1))):
            child(tensor, 'i' + axis + axis, mass * sum(dimensions[k] ** 2 for k in pair) / 12)
        for key in ('ixy', 'ixz', 'iyz'):
            child(tensor, key, 0)
    pieces = [('body', dimensions, (0, 0, 0), rgba)]
    if role == 'table':
        thickness = min(.04, dimensions[2] * .15)
        height = dimensions[2] - thickness
        pieces = [('surface', (*dimensions[:2], thickness), (0, 0, dimensions[2]/2-thickness/2), rgba)]
        for number, signs in enumerate(itertools.product((-1, 1), repeat=2)):
            point = (*(signs[k] * (dimensions[k]/2-.0175-.012) for k in range(2)), -thickness/2)
            pieces.append((f'leg_{number}', (.035, .035, height), point, (.22, .24, .27, 1)))
    for label, size, offset, colour in pieces:
        view = child(link, 'visual', name=label)
        child(view, 'pose', sequence((*offset, 0, 0, 0)))
        geometry(view, size)
        material(view, colour)
        if role != 'zone':
            physical = child(link, 'collision', name=label + '_collision')
            child(physical, 'pose', sequence((*offset, 0, 0, 0)))
            geometry(physical, size)
            if role != 'table':
                friction = child(child(child(physical, 'surface'), 'friction'), 'ode')
                for key in ('mu', 'mu2'):
                    child(friction, key, .8)
    return xml.tostring(document, encoding='unicode')


def serialize_camera(row):
    document = xml.Element('sdf', version='1.7')
    model = child(document, 'model', name=row['name'])
    child(model, 'static', 'true')
    sensor = child(child(model, 'link', name='camera_link'), 'sensor', name='rgb', type='camera')
    for key, value in {'always_on': 'true', 'update_rate': row['update_rate'], 'topic': row['image_topic'],
                       'camera_info_topic': row['camera_info_topic'], 'frame_id': row['frame_id']}.items():
        child(sensor, key, value)
    camera = child(sensor, 'camera')
    child(camera, 'horizontal_fov', row['horizontal_fov'])
    image = child(camera, 'image')
    for key, value in {'width': row['width'], 'height': row['height'], 'format': 'R8G8B8'}.items():
        child(image, key, value)
    clip = child(camera, 'clip')
    child(clip, 'near', .05)
    child(clip, 'far', 5)
    return xml.tostring(document, encoding='unicode')


def spawn_specs(layout):
    rows = [(layout.data[k]['name'], layout.data[k], k) for k in ('pedestal', 'table')]
    rows += [(key, row, 'zone') for key, row in layout.data['zones'].items()]
    rows += [(key, row, 'object') for key, row in layout.data['objects'].items()]
    result = [(name, serialize_box(name, row, role), row['pose']) for name, row, role in rows]
    camera = layout.data['camera']
    result.append((camera['name'], serialize_camera(camera), camera['pose']))
    return result
