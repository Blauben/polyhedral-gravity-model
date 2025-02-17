"""This script is used to scale meshes up and down."""

import math
import sys

import numpy as np
import open3d as o3d


class Node:
    """
    This class represents a node in the mesh.
    """

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
        """
        Converts a line from the .node file to a Node object.
        Args:
            line (str): Line taken from a .node file.

        Returns:
            the constructed Node object.
        """

        index, *floats = line.split(" ")
        index = int(index)
        floats = list(map(float, floats))
        return Node(index, *floats)
    
    def vertex(self):
        """
        Used to convert the node's position vector to a list of vector coordinates.

        Returns:
            The position vector coordinates.
        """

        return [self.x,self.y,self.z]


class Face:
    """
    This class represents a face in the mesh.
    """
    idx: int
    v_idx1: int
    v_idx2: int
    v_idx3: int

    def __init__(self, idx: int, v_idx1: int, v_idx2: int, v_idx3: int):
        self.idx = idx
        self.v_idx1 = v_idx1
        self.v_idx2 = v_idx2
        self.v_idx3 = v_idx3

    @staticmethod
    def from_line(line):
        """
        Converts a line from the .face file to a Face object.
        Args:
            line (str): Line taken from a .face file.

        Returns:
            the constructed Face object.
        """
        results = list(map(int, line.split(" ")))
        return Face(*results)
    
    def triangle(self):
        """
        Returns the face's node indices in a list.
        Returns:
            the node indices in a list.
        """

        return [self.v_idx1, self.v_idx2, self.v_idx3]


def fetch_data(node_file, face_file):
    """
    Reads the mesh data from the .node and .face files.
    Args:
        node_file (str): The path to the node file.
        face_file (str): The path to the face file.

    Returns:
        The list of Node objects and the list of Face objects that make up the mesh.
    """

    with open(node_file, "r") as f:
        node_lines = f.readlines()
    with open(face_file, "r") as f:
        face_lines = f.readlines()
    nodes = []
    faces = []
    skipped_first = False
    for line in node_lines:
        # ignore comments
        if line.strip().startswith("#"):
            continue
        # The first line contains metadata not parseable as a node.
        if not skipped_first:
            skipped_first = True
            continue
        nodes.append(Node.from_line(line))

    skipped_first = False
    for line in face_lines:
        # ignore comments
        if line.strip().startswith("#"):
            continue
        # The first line contains metadata not parseable as a face.
        if not skipped_first:
            skipped_first = True
            continue
        faces.append(Face.from_line(line))
    return nodes, faces

def scale_mesh(nodes, faces, number_of_faces):
    """
    Scales the mesh to contain the given number of faces.

    The algorithm first scales the mesh up by steps of roughly four and then reduces again to the targeted amount of faces.
    Internally the open3d library is used.

    Args:
        nodes (list[Node]): The list of Node objects.
        faces (list[Face]): The list of Face objects.
        number_of_faces (int): The number of faces to scale the mesh to.

    Returns:
        The list of Node objects and the list of Face objects of the scaled mesh.
    """

    # convert Node and Face objects to their coordinates and indices.
    vertices = list(map(lambda n: n.vertex(), nodes))
    triangles = list(map(lambda n: n.triangle(), faces))
    # construct the open3d mesh instance.
    mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(vertices), triangles=o3d.utility.Vector3iVector(triangles))
    # calculate the minimal number of upscale iterations in order to exceed the target number of faces.
    iterations = max(0, math.ceil(math.log(number_of_faces / len(triangles)) / math.log(4)))
    # scale the mesh up.
    if iterations > 0:
        mesh = mesh.subdivide_loop(number_of_iterations=iterations)
    # scale the mesh to the exact number of faces required.
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=number_of_faces)
    # prune the resulting mesh
    mesh = mesh.remove_unreferenced_vertices()
    # extract vertex coordinates and triangle indices and convert them back to Node and Face objects.
    vertices = np.asarray(mesh.vertices).tolist()
    triangles = np.asarray(mesh.triangles).tolist()
    nodes = []
    faces = []
    for i in range(len(vertices)):
        nodes.append(Node(i,*vertices[i]))
    for i in range(len(triangles)):
        faces.append(Face(i, *triangles[i]))
    return nodes, faces


def write_to_file(nodes, faces, filename):
    """
    Writes the mesh to a file.
    Args:
        nodes: The list of Node objects that make up the mesh.
        faces: The list of Face objects that make up the mesh.
        filename: The path to the output file.

    Returns:
        None
    """

    with open(f"{filename}.node", "w") as f:
        # write the header and metadata
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Node count, 3 dimensions, no attribute, no boundary marker\n")
        f.write(f"{len(nodes)} 3 0 0\n")
        f.write("# Node index, node coordinates\n")

        # write the nodes
        for node in nodes:
            line = f"{node.idx} {node.x} {node.y} {node.z}\n"
            f.write(line)

    with open(f"{filename}.face", "w") as f:
        # write the header and metadata
        f.write("# Adapted from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes\n")
        f.write("# Number of faces, boundary marker off\n")
        f.write(f"{len(faces)} 0\n")
        f.write("# Face index, nodes of face\n")

        # write the faces
        for face in faces:
            line = f"{face.idx} {face.v_idx1} {face.v_idx2} {face.v_idx3}\n"
            f.write(line)


def main():
    """
    Main function scaling the mesh to different predefined sizes outputting them into their respective files.
    Returns:
        exit_code (int): The exit code of the program.
    """

    # print help in case of mismatched program arguments
    if len(sys.argv) != 3:
        print("Usage: python scale_mesh.py <node_file> <face_file>")
        return -1
    nodes, faces = fetch_data(sys.argv[1], sys.argv[2])
    filename = sys.argv[1].split(".")[0]
    # amend this line to scale to other sizes
    face_amounts = [round(1000* math.sqrt(3)**k) for k in range(10)]
    # scale to each requested face_amount and output the result into different files.
    for amount in face_amounts:
        nodes, faces = scale_mesh(nodes, faces, amount)
        write_to_file(nodes, faces, f"{filename}_scaled-{amount}")


if __name__ == "__main__":
    main()
