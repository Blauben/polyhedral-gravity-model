import regex


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
        results = regex.search(r"^(\d+)\s([+-]?[0-9]*\.[0-9]+(?:e-?\d+)?)\s([+-]?[0-9]*\.[0-9]+(?:e-?\d+)?)\s([+-]?[0-9]*\.[0-9]+(?:e-?\d+)?)", line)
        if results is None:
            raise RuntimeError(f"Regex parsing failed for line: <{line}>")
        return Node(int(results.group(1)), float(results.group(2)), float(results.group(3)), float(results.group(4)))

    def __add__(self, other):
        return Node(-1, self.x + other.x, self.y + other.y, self.z + other.z)


class Face:
    idx: int
    vIdx1: int
    vIdx2: int
    vIdx3: int

    def __init__(self, idx: int, vIdx1: int, vIdx2: int, vIdx3: int):
        self.idx = idx
        self.vIdx1 = vIdx1
        self.vIdx2 = vIdx2
        self.vIdx3 = vIdx3

    def midPoint(self, nodes):
        node = nodes[self.vIdx1] + nodes[self.vIdx2] + nodes[self.vIdx3]
        node.idx = len(nodes)
        return node

    def generateSplitFaces(self, nodeIdx, nextFaceIdx):
        return [Face(nextFaceIdx, self.vIdx1, self.vIdx2, nodeIdx), Face(nextFaceIdx + 1, self.vIdx2, self.vIdx3, nodeIdx), Face(nextFaceIdx + 2, self.vIdx3, self.vIdx1, nodeIdx)]


    @staticmethod
    def from_line(line):
        results = regex.search(r"^(\d+)\s(\d+)\s(\d+)\s(\d+)", line)
        if results is None:
            return None
        return Face(int(results.group(1)), int(results.group(2)), int(results.group(3)), int(results.group(4)))


def fetchData():
    with open("mesh.node", "r") as f:
        node_lines = f.readlines()
    with open("mesh.face", "r") as f:
        face_lines = f.readlines()
    nodes = []
    faces = []
    for i in range(5, len(node_lines)):
        nodes.append(Node.from_line(node_lines[i]))
    for i in range(5, len(face_lines)):
        faces.append(Face.from_line(face_lines[i]))
    return nodes, faces

def scaleUpMesh(nodes, faces):
    newFaces = []
    for face in faces:
        node = face.midPoint(nodes)
        nodes.append(node)
        newFaces.extend(face.generateSplitFaces(node.idx, len(newFaces)))
    return nodes, newFaces

def writeToFile(nodes, faces):
    with open("mesh_out.node", "w") as f:
        f.write("# Data taken from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes (last accessed: 07.04.2022)\n# and unpickled with /script/mesh.py\n# Node count, 3 dimensions, no attribute, no boundary marker\n")
        f.write(f"{len(nodes)} 3 0 0\n")
        f.write("# Node index, node coordinates\n")

        for node in nodes:
            f.write(f"{node.idx} {node.x} {node.y} {node.z}\n")

    with open("mesh_out.face", "w") as f:
        f.write("# Data taken from https://github.com/darioizzo/geodesyNets/tree/master/3dmeshes (last accessed: 07.04.2022)\n# and unpickled with /script/mesh.py\n# Number of faces, boundary marker off\n")
        f.write(f"{len(faces)} 0\n")
        f.write("# Face index, nodes of face\n")

        for face in faces:
            f.write(f"{face.idx} {face.vIdx1} {face.vIdx2} {face.vIdx3}\n")

def main():
    nodes, faces = fetchData()
    nodes, faces = scaleUpMesh(nodes, faces)
    writeToFile(nodes, faces)

if __name__ == "__main__":
    main()