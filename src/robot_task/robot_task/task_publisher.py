from __future__ import annotations

import json
import os
import rclpy

from dotenv import load_dotenv
from neo4j import GraphDatabase
from rclpy.node import Node
from typing import Any
from std_msgs.msg import String

load_dotenv()


class TaskPublisher(Node):
    def __init__(self) -> None:
        super().__init__('task_publisher')

        uri = os.getenv('NEO4J_URI')
        user = os.getenv('NEO4J_USER')
        password = os.getenv('NEO4J_PASSWORD')

        self._query_limit = 50

        if not uri or not user or not password:
            self.get_logger().error(
                'Missing Neo4j credentials. Please set NEO4J_URI, NEO4J_USER, and NEO4J_PASSWORD in the environment.')

        self.driver = None
        if uri and user and password:
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
                    f'Failed to connect to Neo4j at {uri}: {exc}. ')

        self.publisher = self.create_publisher(String, '/task', 10)
        self._printed_task_names_and_ids = False
        self._warned_no_tasks = False
        self._selected_task: dict[str, Any] | None = None

    def query_mep_elements(self) -> list[dict[str, Any]]:
        if not self.driver:
            return []

        query = """
        MATCH (n:MEPElement)
        RETURN n.id AS id,
               n.name AS name,
               n.ifcClass AS ifcClass
        ORDER BY n.name
        LIMIT $limit
        """

        elements: list[dict[str, Any]] = []
        with self.driver.session() as session:
            result = session.run(query, limit=self._query_limit)
            for record in result:
                elements.append({
                    'id': record['id'],
                    'name': record['name'],
                    'ifcClass': record['ifcClass'],
                })

        return elements

    def query_selected_task_details(self, element_id: str) -> dict[str, Any] | None:
        if not self.driver:
            return None

        query = """
        MATCH (e:MEPElement {id: $element_id})
        OPTIONAL MATCH (e)<-[c:CONTAINS]-(system:MEPSystem)
        OPTIONAL MATCH (e)-[p:PASSES_THROUGH]->(wall:Wall)
        WITH e,
             collect(DISTINCT CASE
                 WHEN system IS NULL THEN NULL
                 ELSE {
                     id: system.id,
                     name: system.name,
                     ifcClass: system.ifcClass
                 }
             END) AS systems,
             collect(DISTINCT CASE
                 WHEN wall IS NULL OR p IS NULL THEN NULL
                 ELSE {
                     wall: {
                         id: wall.id,
                         name: wall.name,
                         ifcClass: wall.ifcClass,
                         center: wall.center,
                         bbox_min: wall.bbox_min,
                         bbox_max: wall.bbox_max,
                         axis2: wall.axis2,
                         layerCount: wall.layerCount,
                         directionSense: wall.directionSense
                     },
                     penetration: properties(p)
                 }
             END) AS wall_relations
        RETURN e {
            .id,
            .name,
            .ifcClass,
            .shapeType,
            systems: [item IN systems WHERE item IS NOT NULL],
            wall_relations: [item IN wall_relations WHERE item IS NOT NULL]
        } AS task
        LIMIT 1
        """

        try:
            with self.driver.session() as session:
                record = session.run(query, element_id=element_id).single()
                if record is None:
                    return None
                return record['task']
        except Exception as exc:
            self.get_logger().error(
                f'Failed to query details for selected task {element_id}: {exc}')
            return None

    def choose_task_from_terminal(self, elements: list[dict[str, Any]]) -> dict[str, Any] | None:
        if not elements:
            return None

        print('\nAvailable MEP tasks:')
        for index, element in enumerate(elements, start=1):
            print(f"  {index}. {element['name']} | id={element['id']}")

        while True:
            try:
                selection = input(
                    'Select one task number to publish: ').strip()
                selected_index = int(selection)
                if 1 <= selected_index <= len(elements):
                    selected = elements[selected_index - 1]
                    print(
                        f"Selected task: {selected['name']} | id={selected['id']}")
                    return selected
                print(f'Please enter a number between 1 and {len(elements)}.')
            except ValueError:
                print('Please enter a valid integer.')

    def select_task(self) -> None:
        elements = self.query_mep_elements()
        self._selected_task = self.choose_task_from_terminal(elements)
        if self._selected_task is None:
            self.get_logger().warn('No task selected; /task will stay empty.')
            return

        self._printed_task_names_and_ids = True
        self.get_logger().info(
            f"Publishing only selected task: {self._selected_task['name']} | id={self._selected_task['id']}")

    def publish_task(self) -> None:
        if self._selected_task is None:
            return

        payload = {
            'count': 1,
            'element': self.query_selected_task_details(self._selected_task['id']),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published selected task: {self._selected_task['name']} | id={self._selected_task['id']}")

    def destroy_node(self) -> bool:
        if self.driver is not None:
            self.driver.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TaskPublisher()
    try:
        node.select_task()
        node.publish_task()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
