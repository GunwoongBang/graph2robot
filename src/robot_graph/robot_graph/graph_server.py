import json
import os
import rclpy

from dotenv import load_dotenv
from neo4j import GraphDatabase
from rclpy.node import Node
from std_srvs.srv import Trigger

from .util import (
    query_spaces,
    query_walls,
    query_layers,
    query_mep_elements,
)

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

        # Servers
        self._space_srv = self.create_service(
            Trigger, '/graph/list_spaces', self._handle_list_spaces)
        self._wall_srv = self.create_service(
            Trigger, '/graph/list_walls', self._handle_list_walls)
        self._layer_srv = self.create_service(
            Trigger, '/graph/list_layers', self._handle_list_layers)
        self._mep_element_srv = self.create_service(
            Trigger, '/graph/list_mep_elements', self._handle_list_mep_elements)

    def _handle_list_spaces(
            self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.driver is None:
            response.success = False
            response.message = 'Neo4j driver is not connected.'
            return response

        try:
            spaces = query_spaces(self.driver, self._query_limit)
        except Exception as exc:
            self.get_logger().error(f'Failed to query spaces: {exc}')
            response.success = False
            response.message = f'Query failed: {exc}'
            return response

        response.success = True
        response.message = json.dumps(spaces)
        self.get_logger().info(
            f'Returned {len(spaces)} spaces to client.')
        return response

    def _handle_list_walls(
            self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.driver is None:
            response.success = False
            response.message = 'Neo4j driver is not connected.'
            return response

        try:
            walls = query_walls(self.driver, self._query_limit)
        except Exception as exc:
            self.get_logger().error(f'Failed to query walls: {exc}')
            response.success = False
            response.message = f'Query failed: {exc}'
            return response

        response.success = True
        response.message = json.dumps(walls)
        self.get_logger().info(
            f'Returned {len(walls)} walls to client.')
        return response

    def _handle_list_layers(
            self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.driver is None:
            response.success = False
            response.message = 'Neo4j driver is not connected.'
            return response

        try:
            layers = query_layers(
                self.driver, self._query_limit)  # Placeholder
        except Exception as exc:
            self.get_logger().error(f'Failed to query layers: {exc}')
            response.success = False
            response.message = f'Query failed: {exc}'
            return response

        response.success = True
        response.message = json.dumps(layers)
        self.get_logger().info(
            f'Returned {len(layers)} layers to client.')
        return response

    def _handle_list_mep_elements(
            self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.driver is None:
            response.success = False
            response.message = 'Neo4j driver is not connected.'
            return response

        try:
            mep_elements = query_mep_elements(self.driver, self._query_limit)
        except Exception as exc:
            self.get_logger().error(f'Failed to query MEP elements: {exc}')
            response.success = False
            response.message = f'Query failed: {exc}'
            return response

        response.success = True
        response.message = json.dumps(mep_elements)
        self.get_logger().info(
            f'Returned {len(mep_elements)} MEP elements to client.')
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
