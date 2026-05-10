from alignment_visualizer import visualize_alignment
from matrix_generator import generate_matrix

if __name__ == "__main__":
    print("=== Step 1: Generating transformation matrix from IFC to PCD ===")
    generate_matrix()
    print("\n=== Step 2: Visualizing alignment with Open3D ===")
    visualize_alignment()
