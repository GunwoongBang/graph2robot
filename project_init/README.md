### `matrix_manager`
project_init prepares essential files for launching the packages
- `matrix_manager.py` generates a transformation matrix and visualize the result
    - `matrix_generator.py`: transforms the input IFC world file into a point cloud and performs ICP alignment with the input point cloud
    - `alignment_visualizer.py`: shows how well the alignment using the generated transformation matrix 
        - IFC: red color point cloud
        - PCD: blue color point cloud
- Output `transform_matrix.yaml` file is stored in `src/robot_task/config` directory
### `urdf_manager`
Not yet implemented or it will not..?
