from typing import Any
from neo4j import Driver

_QUERY_MEP_ELEMENTS = """
MATCH (e:MEPElement)
OPTIONAL MATCH (e)-[p:PASSES_THROUGH]->(w:Wall)
WITH e,
     collect(CASE WHEN w IS NULL THEN NULL ELSE {
         wall: {id: w.id, name: w.name},
         center: p.penetrationCenter
     } END) AS rels
WITH e, [x IN rels WHERE x IS NOT NULL] AS me_w_rels
RETURN e.id AS id,
       e.ifcClass AS ifcClass,
       e.face AS face,
       head(me_w_rels).wall AS wall,
       head(me_w_rels).center AS center
ORDER BY e.name
LIMIT $limit
"""

_QUERY_WALLS = """
MATCH (w:Wall)
RETURN w.id AS id,
       w.axis2 AS axis2
ORDER BY w.name
LIMIT $limit
"""


def query_mep_elements(driver: Driver, limit: int) -> list[dict[str, Any]]:
    elements: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_MEP_ELEMENTS, limit=limit)
        for record in result:
            elements.append({
                'id': record['id'],
                'ifcClass': record['ifcClass'],
                'face': record['face'],
                'wall': record['wall'],
                'center': record['center'],
            })
    return elements


def query_walls(driver: Driver, limit: int) -> list[dict[str, Any]]:
    walls: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_WALLS, limit=limit)
        for record in result:
            walls.append({
                'id': record['id'],
                'axis2': record['axis2'],
            })
    return walls
