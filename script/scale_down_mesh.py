import scale_up_mesh as sc
import sys
import open3d as o3d
import numpy as np

def scale_down_mesh(nodes, faces):
    vertices = list(map(lambda n: n.vertex(), nodes))
    triangles = list(map(lambda n: n.triangle(), faces))
    mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(vertices), triangles=o3d.utility.Vector3iVector(triangles))
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=round(len(faces) / 3))
    vertices = np.asarray(mesh.vertices).tolist()
    triangles = np.asarray(mesh.triangles).tolist()
    nodes = []
    faces = []
    for i in range(len(vertices)):
        nodes.append(sc.Node(i,*vertices[i]))
    for i in range(len(triangles)):
        faces.append(sc.Face(i, *triangles[i]))
    return nodes, faces


def main():
    if len(sys.argv) != 4:
        print("Usage: python scale_down_mesh.py <node_file> <face_file> <number_of_iterations>")
        return -1
    nodes, faces = sc.fetch_data(sys.argv[1], sys.argv[2])
    filename = sys.argv[1].split(".")[0]
    for i in range(1, int(sys.argv[3]) + 1):
        nodes, faces = scale_down_mesh(nodes, faces)
        sc.write_to_file(nodes, faces, f"{filename}_downscaled-{i}")


if __name__ == "__main__":
    main()