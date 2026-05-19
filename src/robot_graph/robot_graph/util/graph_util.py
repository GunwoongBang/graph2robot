from typing import Any
from neo4j import Driver

_QUERY_SPACES = """
MATCH (s:Space)
RETURN s.id AS id
ORDER BY s.name
LIMIT $limit
"""

_QUERY_WALLS = """
MATCH (w:Wall)
OPTIONAL MATCH (s:Space)-[b:BOUNDED_BY]->(w)
WITH w,
     collect(CASE WHEN s IS NULL THEN NULL ELSE {
         id: s.id, side: b.side
     } END) AS raw
WITH w, [x IN raw WHERE x IS NOT NULL] AS space
RETURN w.id AS id,
       w.axis2 AS axis2,
       space
ORDER BY w.name
LIMIT $limit
"""

_QUERY_MEP_ELEMENTS = """
MATCH (me:MEPElement)
OPTIONAL MATCH (s:Space)-[:HOSTS]->(me)
WITH me,
     collect(CASE WHEN s IS NULL THEN NULL ELSE {
         id: s.id, name: s.name
     } END) AS s_raw
WITH me, head([x IN s_raw WHERE x IS NOT NULL]) AS space
OPTIONAL MATCH (w:Wall)-[p:PENETRATED_BY]->(me)
WITH me, space,
     collect(CASE WHEN w IS NULL THEN NULL ELSE {
         id: w.id, center: p.penetrationCenter
     } END) AS w_raw
WITH me, space, head([x IN w_raw WHERE x IS NOT NULL]) AS wall
RETURN me.id AS id,
       me.name AS name,
       me.face AS face,
       wall,
       space
ORDER BY me.name
LIMIT $limit
"""


def query_spaces(driver: Driver, limit: int) -> list[dict[str, Any]]:
    spaces: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_SPACES, limit=limit)
        for record in result:
            spaces.append({
                'id': record['id'],
            })
    return spaces


def query_walls(driver: Driver, limit: int) -> list[dict[str, Any]]:
    walls: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_WALLS, limit=limit)
        for record in result:
            walls.append({
                'id': record['id'],
                'axis2': record['axis2'],
                'space': record['space'],
            })
    return walls


def query_mep_elements(driver: Driver, limit: int) -> list[dict[str, Any]]:
    elements: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_MEP_ELEMENTS, limit=limit)
        for record in result:
            elements.append({
                'id': record['id'],
                'name': record['name'],
                'face': record['face'],
                'wall': record['wall'],
                'space': record['space'],
            })
    return elements
