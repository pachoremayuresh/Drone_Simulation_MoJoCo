import math

def read_obj_file(filename):
    vertices = []
    
    with open(filename, 'r') as file:
        for line in file:
            # Read lines starting with 'v', which represent vertices
            if line.startswith('v '):
                parts = line.split()
                # Convert the x, y, z coordinates to float and store them
                x, y, z = map(float, parts[1:4])
                vertices.append((x, y, z))
    
    return vertices

def calculate_distance(v1, v2):
    # Calculate the Euclidean distance between two 3D points (v1 and v2)
    return math.sqrt((v2[0] - v1[0]) ** 2 + (v2[1] - v1[1]) ** 2 + (v2[2] - v1[2]) ** 2)

def find_max_length(vertices):
    max_distance = 0
    # Compare all pairs of vertices
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            distance = calculate_distance(vertices[i], vertices[j])
            if distance > max_distance:
                max_distance = distance
    return max_distance

# Usage
obj_file = 'Example_2_Existing_URDF_model/fiberthex/assets/prop.obj'  # replace with your .obj file path
vertices = read_obj_file(obj_file)
max_length = find_max_length(vertices)

print(f"The maximum length (distance) between vertices is: {max_length}")