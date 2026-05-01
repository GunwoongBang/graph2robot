tools/generate_transform.py -> run script to generate a transformation matrix
tools/visualize_alignment.py -> check the alignment visually
run task_publisher node to publish a topic
topic is an MEP element, retrieved from the Neo4j graph database and the position is transformed with the transformation matrix


In this package there are (currently) two nodes:
- task_publisher: publishes a target task (mep element) as a topic
- matrix_publisher: publishes the transformation matrix as a topic

Launch file binds the two nodes and deploy them together
