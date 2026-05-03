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
WITH e, [x IN rels WHERE x IS NOT NULL] AS valid_rels
RETURN e.id AS id,
       e.name AS name,
       e.ifcClass AS ifcClass,
       head(valid_rels).wall AS wall,
       head(valid_rels).center AS center
ORDER BY e.name
LIMIT $limit
"""


def query_mep_elements(driver: Driver, limit: int) -> list[dict[str, Any]]:
    elements: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_MEP_ELEMENTS, limit=limit)
        for record in result:
            elements.append({
                'id': record['id'],
                'name': record['name'],
                'ifcClass': record['ifcClass'],
                'wall': record['wall'],
                'center': record['center'],
            })
    return elements
