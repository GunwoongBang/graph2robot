import json
import os
import rclpy

from dotenv import load_dotenv
from neo4j import GraphDatabase
from rclpy.node import Node
from std_srvs.srv import Trigger

from .util import query_mep_elements

load_dotenv()


class GraphServer(Node):
    def __init__(self) -> None:
        super().__init__('graph_server')

        uri = os.getenv('NEO4J_URI')
        user = os.getenv('NEO4J_USER')
        password = os.getenv('NEO4J_PASSWORD')

        self._query_limit = 50
        self.driver = None

        if not uri or not user or not password:
            self.get_logger().error(
                'Missing Neo4j credentials. Please set NEO4J_URI, NEO4J_USER, and NEO4J_PASSWORD in the environment.')
        else:
            try:
                self.driver = GraphDatabase.driver(uri, auth=(user, password))
                with self.driver.session() as session:
                    session.run('RETURN 1').consume()
                self.get_logger().info(f'Connected to Neo4j at {uri}')
            except Exception as exc:
                if self.driver is not None:
                    self.driver.close()
                    self.driver = None
                self.get_logger().error(
                    f'Failed to connect to Neo4j at {uri}: {exc}.')

        # Currently, the node only provides one service to list MEP elements.
        # In the future, there could be more services for different types of elements.
        self._mep_elements_srv = self.create_service(
            Trigger, '/graph/list_mep_elements', self._handle_list_mep_elements)
        self.get_logger().info(
            'Service ready: /graph/list_mep_elements (std_srvs/Trigger)')

        # TODO: Add more services for updating the graph
        # self._update_graph_srv = self.create_service(...)

    def _handle_list_mep_elements(
            self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.driver is None:
            response.success = False
            response.message = 'Neo4j driver is not connected.'
            return response

        try:
            elements = query_mep_elements(self.driver, self._query_limit)
        except Exception as exc:
            self.get_logger().error(f'Failed to query MEP elements: {exc}')
            response.success = False
            response.message = f'Query failed: {exc}'
            return response

        response.success = True
        response.message = json.dumps(elements)
        self.get_logger().info(
            f'Returned {len(elements)} MEP elements to client.')
        return response

    def destroy_node(self) -> bool:
        if self.driver is not None:
            self.driver.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GraphServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
