"""Full URDF visual export: offsets, hard edges and disconnected pieces."""
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from scripts.export_quest_ar_assets import visual_geometry, read_stl


def fixture_visual(tmp_path, triangles, normals, *, scale='1 1 1', xyz='0 0 0', rpy='0 0 0'):
    mesh_dir = tmp_path / 'meshes'
    mesh_dir.mkdir()
    mesh = mesh_dir / 'piece.stl'
    raw = bytearray(b'\0' * 80 + struct.pack('<I', len(triangles)))
    for points, normal in zip(triangles, normals):
        raw.extend(struct.pack('<12fH', *normal, *np.asarray(points).flatten(), 0))
    mesh.write_bytes(raw)
    urdf = tmp_path / 'urdf' / 'robot.urdf'
    visual = ET.fromstring(f'''<visual><origin xyz="{xyz}" rpy="{rpy}"/>
        <geometry><mesh filename="package://different_folder_name/meshes/piece.stl"
        scale="{scale}"/></geometry></visual>''')
    return visual, urdf, mesh


def test_small_disconnected_part_and_hard_edges_preserved(tmp_path):
    triangles = np.array([[[0,0,0], [1,0,0], [0,1,0]],
                          [[0,0,0], [0,1,0], [0,0,1]],
                          [[2,2,2], [2.0001,2,2], [2,2.0001,2]]])
    visual, urdf, mesh = fixture_visual(tmp_path,triangles,[[0,0,1], [1,0,0], [0,0,0]])
    points, normals, faces, path = visual_geometry(visual,urdf)
    assert path == mesh
    np.testing.assert_array_equal(points[faces],triangles.astype(np.float32))
    # The shared corner must keep two separate normals, so sharp edges survive.
    assert faces[0,0] != faces[1,0]
    np.testing.assert_array_equal(normals[faces[0,0]],[0,0,1])
    np.testing.assert_array_equal(normals[faces[1,0]],[1,0,0])
    np.testing.assert_array_equal(normals[faces[2,0]],[0,0,1])


def test_visual_origin_nonuniform_scale_and_reflection(tmp_path):
    triangle = [[[0,0,0],[1,0,0],[0,1,0]]]
    visual, urdf, _ = fixture_visual(tmp_path,triangle,[[0,0,1]],
                                    scale='-2 3 4',xyz='1 2 3',rpy=f'0 0 {np.pi/2}')
    points, normals, faces, _ = visual_geometry(visual,urdf)
    # Negative scaling reverses winding; yaw maps -X to -Y and +Y to -X.
    np.testing.assert_allclose(points[faces],[[[-2,2,3],[1,0,3],[1,2,3]]],atol=1e-6)
    np.testing.assert_allclose(normals,[[0,0,1]]*len(normals),atol=1e-6)


def test_ascii_stl_preserves_face(tmp_path):
    mesh = tmp_path/'piece.stl'
    mesh.write_text('''solid example
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 1 0 0
vertex 0 1 0
endloop
endfacet
endsolid example
''')
    points, normals = read_stl(mesh)
    np.testing.assert_array_equal(points,[[[0,0,0],[1,0,0],[0,1,0]]])
    np.testing.assert_array_equal(normals,[[0,0,1]])


def test_missing_or_unsupported_visual_fails_instead_of_omitting(tmp_path):
    urdf=tmp_path/'urdf'/'robot.urdf'
    with pytest.raises(FileNotFoundError):
        visual_geometry(ET.fromstring('<visual><geometry><mesh filename="missing.stl"/></geometry></visual>'),urdf)
    with pytest.raises(ValueError,match='incomplete export'):
        visual_geometry(ET.fromstring('<visual><geometry><box size="1 1 1"/></geometry></visual>'),urdf)
