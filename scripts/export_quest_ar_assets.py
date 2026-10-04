"""Export EVERY current URDF visual triangle as link-local GLB for Quest AR.

No decimation, component removal or collision mesh substitution. Only identical
position/normal vertices are indexed together, preserving every source face.
uv run --with numpy python scripts/export_quest_ar_assets.py
"""
import argparse
import hashlib
import gzip
import json
from pathlib import Path
import struct
import sys
import tarfile
import io
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import ar_config
from robot.f14_config import F14_URDF_PATH
from robot.appearance import PALETTE, link_finish, linear_color


def write_glb(path, meshes):
    doc = {'asset': {'version': '2.0', 'generator': 'F14 Quest AR full URDF visual exporter'},
           'scene': 0, 'scenes': [{'nodes': []}], 'nodes': [], 'meshes': [],
           'materials': [], 'accessors': [], 'bufferViews': [], 'buffers': []}
    binary = bytearray()

    def accessor(array, component, kind, target, bounds=False):
        array = np.ascontiguousarray(array)
        binary.extend(b'\0' * ((-len(binary)) % 4))
        view = len(doc['bufferViews'])
        doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(binary),
                                   'byteLength': array.nbytes, 'target': target})
        binary.extend(array.tobytes())
        record = {'bufferView': view, 'componentType': component, 'count': len(array), 'type': kind}
        if bounds:
            record.update(min=array.min(axis=0).tolist(), max=array.max(axis=0).tolist())
        index = len(doc['accessors'])
        doc['accessors'].append(record)
        return index

    link_nodes = {}
    for visual in meshes:
        name, link = visual['name'], visual['link']
        points, normals, faces = visual['points'], visual['normals'], visual['faces']
        points = points.astype('<f4')
        normals = normals.astype('<f4')
        component = 5123 if len(points) <= 65536 else 5125
        faces = faces.astype('<u2' if component == 5123 else '<u4')
        pos = accessor(points, 5126, 'VEC3', 34962, True)
        norm = accessor(normals, 5126, 'VEC3', 34962)
        indices = accessor(faces.reshape(-1), component, 'SCALAR', 34963)
        _, metallic, roughness = PALETTE[link_finish(link)]
        color = visual['rgba']
        material = len(doc['materials'])
        doc['materials'].append({'name': link_finish(link), 'doubleSided': True,
            'pbrMetallicRoughness': {'baseColorFactor': list(linear_color(color[:3])) + [color[3]],
                                   'metallicFactor': metallic, 'roughnessFactor': roughness}})
        if color[3] < 1:
            doc['materials'][-1]['alphaMode'] = 'BLEND'
        index = len(doc['meshes'])
        doc['meshes'].append({'name': name, 'primitives': [{'attributes': {'POSITION': pos, 'NORMAL': norm},
                                                         'indices': indices, 'material': material}]})
        if link not in link_nodes:
            link_nodes[link] = len(doc['nodes'])
            doc['scenes'][0]['nodes'].append(len(doc['nodes']))
            doc['nodes'].append({'name': link, 'children': []})
        doc['nodes'][link_nodes[link]]['children'].append(len(doc['nodes']))
        doc['nodes'].append({'name': name, 'mesh': index})
    doc['buffers'] = [{'byteLength': len(binary)}]
    binary.extend(b'\0' * ((-len(binary)) % 4))
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' ' * ((-len(encoded)) % 4)
    path.write_bytes(struct.pack('<4sII', b'glTF', 2, 12+8+len(encoded)+8+len(binary))
                     + struct.pack('<I4s', len(encoded), b'JSON') + encoded
                     + struct.pack('<I4s', len(binary), b'BIN\0') + binary)


def resolve_mesh(filename, urdf):
    if filename.startswith('package://'):
        # SolidWorks package name differs from this checkout's folder name.
        relative = filename[len('package://'):].split('/', 1)
        if len(relative) != 2:
            raise ValueError(f'Invalid URDF package mesh: {filename}')
        path = urdf.parent.parent / relative[1]
    elif filename.startswith('file://'):
        path = Path(filename[len('file://'):])
    else:
        path = urdf.parent / filename
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f'URDF visual mesh missing: {filename} -> {path}')
    return path


def read_stl(path):
    """Read full binary/ASCII STL triangles; never silently drop a face."""
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0] if len(raw) >= 84 else 0
    if count and len(raw) == 84 + count * 50:
        records = np.frombuffer(raw, dtype=np.dtype([
            ('normal','<f4',(3,)), ('points','<f4',(3,3)), ('attribute','<u2')]),
            count=count, offset=84)
        triangles = records['points'].astype(float)
        normals = records['normal'].astype(float)
    else:
        vertices, normals = [], []
        for line in raw.decode('ascii').splitlines():
            words = line.strip().split()
            if words[:2] == ['facet','normal']:
                normals.append([float(v) for v in words[2:]])
            elif words[:1] == ['vertex']:
                vertices.append([float(v) for v in words[1:]])
        triangles = np.asarray(vertices, dtype=float).reshape(-1,3,3)
        normals = np.asarray(normals, dtype=float).reshape(-1,3)
    if not len(triangles) or len(normals) != len(triangles) or not np.isfinite(triangles).all() or not np.isfinite(normals).all():
        raise ValueError(f'Invalid STL visual: {path}')
    missing = np.linalg.norm(normals, axis=1) < 1e-12
    normals[missing] = np.cross(triangles[missing,1]-triangles[missing,0],
                                triangles[missing,2]-triangles[missing,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-20)
    return triangles, normals


def visual_geometry(visual, urdf):
    geometry = visual.find('geometry')
    mesh = geometry.find('mesh') if geometry is not None else None
    if mesh is None:
        raise ValueError('Unsupported URDF visual geometry; refusing an incomplete export')
    path = resolve_mesh(mesh.attrib['filename'],urdf)
    if path.suffix.lower() != '.stl':
        raise ValueError(f'Unsupported visual mesh {path}; refusing an incomplete export')
    triangles, normals = read_stl(path)
    scale = np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale==0):
        raise ValueError('Invalid URDF mesh scale')
    origin = visual.find('origin')
    xyz = np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
    rpy = np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz,rpy]).all():
        raise ValueError('Invalid URDF visual origin')
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy = np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    rotation = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                         [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    triangles = (triangles*scale) @ rotation.T + xyz
    normals = (normals/scale) @ rotation.T
    normals /= np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-20)
    if np.prod(scale) < 0:
        triangles = triangles[:,::-1]
    # Lossless indexing includes normals so distinct hard edges stay distinct.
    attributes = np.column_stack((triangles.reshape(-1,3),np.repeat(normals,3,axis=0))).astype('<f4')
    unique, inverse = np.unique(attributes,axis=0,return_inverse=True)
    faces = inverse.reshape(-1,3)
    # Check every face, not just aggregate bounds/counts, before writing GLB.
    if not np.array_equal(unique[faces,:3],triangles.astype('<f4')):
        raise AssertionError(f'Visual triangles changed during indexing: {path}')
    return unique[:,:3],unique[:,3:],faces,path


def export(output, urdf=F14_URDF_PATH):
    urdf = Path(urdf).resolve()
    root = ET.parse(urdf).getroot()
    named_materials = {m.get('name'):m for m in root.findall('material')}
    meshes, report = [], []
    links = root.findall('link')
    if len(links) != 19:
        raise ValueError('This F14 mirror requires the 19 measured Isaac link frames; refusing partial export')
    for link in links:
        name = link.attrib['name']
        visuals = link.findall('visual')
        if not visuals:
            raise ValueError(f'No visual for {name}; refusing an incomplete F14 export')
        for i, visual in enumerate(visuals):
            points,normals,faces,path = visual_geometry(visual,urdf)
            material = visual.find('material')
            if material is not None and material.find('color') is None:
                material = named_materials.get(material.get('name'),material)
            color = material.find('color') if material is not None else None
            if material is not None and material.find('texture') is not None:
                raise ValueError(f'Unsupported URDF texture on {name}; refusing incomplete export')
            rgba = np.fromstring(color.get('rgba') if color is not None else '0.7 0.7 0.7 1',sep=' ')
            if rgba.shape != (4,) or not np.isfinite(rgba).all() or np.any((rgba<0)|(rgba>1)):
                raise ValueError(f'Invalid URDF color on {name}')
            visual_name = f'{name}_visual_{i}'
            meshes.append({'name':visual_name,'link':name,'points':points,'normals':normals,
                           'faces':faces,'rgba':rgba.tolist()})
            report.append({'name':visual_name,'link':name,'source_mesh':str(path.relative_to(urdf.parent.parent)),
                'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                'original_faces':len(faces),'faces':len(faces),'vertices':len(points),
                'bounds':[points.min(axis=0).tolist(),points.max(axis=0).tolist()], 'rgba':rgba.tolist()})
            print(f'{visual_name}: {len(faces)} / {len(faces)} original triangles preserved',flush=True)
    if len(report) != len(root.findall('link/visual')):
        raise AssertionError('URDF visual coverage incomplete')
    output.mkdir(parents=True, exist_ok=True)
    write_glb(output / 'robot.glb', meshes)
    # aiohttp FileResponse automatically serves the precompressed sibling to
    # gzip-capable browsers. This reduces transfer only; geometry is identical.
    with (output / 'robot.glb').open('rb') as source, (output / 'robot.glb.gz').open('wb') as target:
        with gzip.GzipFile(filename='',fileobj=target,mode='wb',compresslevel=6,mtime=0) as compressed:
            while chunk := source.read(1024*1024):
                compressed.write(chunk)
    glb_hash = hashlib.sha256((output / 'robot.glb').read_bytes()).hexdigest()
    manifest = {'version': 2, 'model_id': glb_hash, 'robot_url': f'/assets/robot.glb?v={glb_hash}',
                'source_urdf':str(urdf), 'source_urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),
                'geometry_mode':'full_urdf', 'simplified':False, 'visual_count':len(report),
                'triangle_count':sum(r['faces'] for r in report),
                'every_source_triangle_verified':True,
                'glb_bytes':(output/'robot.glb').stat().st_size,
                'gzip_bytes':(output/'robot.glb.gz').stat().st_size,
                'coordinates': 'Isaac Z-up, metres, link-local vertices',
                'links':[link.attrib['name'] for link in links], 'meshes':report,
                'three_version': ar_config.THREE_VERSION}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f"Exported {(output/'robot.glb').stat().st_size/1e6:.2f} MB, {sum(r['faces'] for r in report)} triangles")


def vendor_three(output):
    version = ar_config.THREE_VERSION
    url = f'https://registry.npmjs.org/three/-/three-{version}.tgz'
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    # Extract only the module and its loader dependencies, without trusting
    # archive paths. Pin files locally so Quest never needs an external CDN.
    names = ['build/three.module.js', 'build/three.core.js', 'LICENSE',
             'examples/jsm/loaders/GLTFLoader.js', 'examples/jsm/utils/BufferGeometryUtils.js']
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        for name in names:
            member = archive.getmember('package/' + name)
            target = output / 'vendor/three' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.extractfile(member).read())
    (output / 'vendor/three/version.txt').write_text(version + '\n')
    (output / 'vendor/three/package.json').write_text(json.dumps({
        'name': 'three', 'version': version, 'type': 'module',
        'exports': {'.': './build/three.module.js'}}) + '\n')
    print(f"Vendored Three.js {version} (MIT)")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ar_config.ASSET_ROOT)
    parser.add_argument('--urdf', type=Path, default=F14_URDF_PATH)
    parser.add_argument('--skip-vendor', action='store_true')
    args = parser.parse_args()
    export(args.output,args.urdf)
    if not args.skip_vendor:
        vendor_three(args.output)
