import regex
import sys

class Node:
    idx: int
    x: float
    y: float
    z: float

    def __init__(self, idx: int, x: float, y: float, z: float):
        self.idx = idx
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_line(line):
        index, *floats = line.split(" ")
        index = int(index)
        floats = list(map(float, floats))
        return Node(index, *floats)

    def __add__(self, other):
        return Node(-1, self.x + other.x, self.y + other.y, self.z + other.z)


class Face:
    idx: int
    v_idx1: int
    v_idx2: int
    v_idx3: int

    def __init__(self, idx: int, v_idx1: int, v_idx2: int, v_idx3: int):
        self.idx = idx
        self.v_idx1 = v_idx1
        self.v_idx2 = v_idx2
        self.v_idx3 = v_idx3

    def mid_point(self, nodes):
        node = nodes[self.v_idx1] + nodes[self.v_idx2] + nodes[self.v_idx3]
        node.idx = len(nodes)
        return node

    def generate_split_faces(self, node_idx, next_face_idx):
        return [
            Face(next_face_idx, self.v_idx1, self.v_idx2, node_idx),
            Face(next_face_idx + 1, self.v_idx2, self.v_idx3, node_idx),
            Face(next_face_idx + 2, self.v_idx3, self.v_idx1, node_idx),
        ]

    @staticmethod
    def from_line(line):
        results = list(map(int, line.split(" ")))
        return Face(*results)


def fetch_data(node_file, face_file):
    with open(node_file, "r") as f:
        node_lines = f.readlines()
    with open(face_file, "r") as f:
        face_lines = f.readlines()
    nodes = []
    faces = []
    skipped_first = False
    for line in node_lines:
        if line.strip().startswith("#"):
            continue
        if not skipped_first:
            skipped_first = True
            continue
        nodes.append(Node.from_line(line))

    skipped_first = False
    for line in face_lines:
        if line.strip().startswith("#"):
            continue
        if not skipped_first:
            skipped_first = True
            continue
        faces.append(Face.from_line(line))
    return nodes, faces


def scale_up_mesh(nodes, faces):
    new_faces = []
    for face in faces:
        node = face.mid_point(nodes)
        nodes.append(node)
        new_faces.extend(face.generate_split_faces(node.idx, len(new_faces)))
    return nodes, new_faces


def write_to_file(nodes, faces):
    with open("mesh_out.node", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Node count, 3 dimensions, no attribute, no boundary marker\n")
        f.write(f"{len(nodes)} 3 0 0\n")
        f.write("# Node index, node coordinates\n")

        for node in nodes:
            f.write(f"{node.idx} {node.x} {node.y} {node.z}\n")

    with open("mesh_out.face", "w") as f:
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Number of faces, boundary marker off\n")
        f.write(f"{len(faces)} 0\n")
        f.write("# Face index, nodes of face\n")

        for face in faces:
            f.write(f"{face.idx} {face.v_idx1} {face.v_idx2} {face.v_idx3}\n")


def main():
    if len(sys.argv) != 3:
        print("Usage: python scaleUpMesh.py <node_file> <face_file>")
        return -1
    nodes, faces = fetch_data(sys.argv[1], sys.argv[2])
    nodes, faces = scale_up_mesh(nodes, faces)
    write_to_file(nodes, faces)


if __name__ == "__main__":
    main()
