-- name: QUERY_SPACES
MATCH (s:Space)
RETURN s.id AS id
ORDER BY s.name
LIMIT $limit

-- name: QUERY_WALLS
MATCH (w:Wall)
OPTIONAL MATCH (s:Space)-[b:BOUNDED_BY]->(w)
WITH w,
     collect(CASE WHEN s IS NULL THEN NULL ELSE {
         id: s.id, side: b.side
     } END) AS raw
WITH w, [x IN raw WHERE x IS NOT NULL] AS space
OPTIONAL MATCH (w)-[:HAS_LAYER]->(l:Layer)
WITH w, space,
     collect(CASE WHEN l IS NULL THEN NULL ELSE {
         id: l.id, name: l.name, layerIndex: l.layerIndex, thickness: l.thickness
     } END) AS l_raw
WITH w, space, [x IN l_raw WHERE x IS NOT NULL] AS layers
RETURN w.id AS id,
       w.axis2 AS axis2,
       w.center AS center,
       w.bbox_max AS bbox_max,
       w.bbox_min AS bbox_min,
       w.directionSense AS directionSense,
       space,
       layers
ORDER BY w.name
LIMIT $limit

-- name: QUERY_LAYERS
MATCH (w:Wall)-[:HAS_LAYER]->(l:Layer)
RETURN l.id AS id,
       l.name AS name,
       l.layerIndex AS layerIndex,
       l.thickness AS thickness,
       w.id AS wall_id
ORDER BY l.name
LIMIT $limit

-- name: QUERY_MEP_ELEMENTS
MATCH (me:MEPElement)
OPTIONAL MATCH (s:Space)-[:HOSTS]->(me:MEPElement)
WITH me,
     collect(CASE WHEN s IS NULL THEN NULL ELSE {
         id: s.id, name: s.name
     } END) AS s_raw
WITH me, head([x IN s_raw WHERE x IS NOT NULL]) AS space
OPTIONAL MATCH (w:Wall)-[p:PENETRATED_BY]->(me:MEPElement)
WITH me, space,
     collect(CASE WHEN w IS NULL THEN NULL ELSE {
         id: w.id,
         center: p.penetrationCenter,
         length: p.penetrationLength,
         radius: p.penetrationRadius,
         sizeX: p.penetrationSizeX,
         sizeY: p.penetrationSizeY,
         sizeZ: p.penetrationSizeZ
     } END) AS w_raw
WITH me, space, head([x IN w_raw WHERE x IS NOT NULL]) AS wall
RETURN me.id AS id,
       me.name AS name,
       me.center AS center,
       me.bbox_max AS bbox_max,
       me.bbox_min AS bbox_min,
       me.length AS length,
       me.radius AS radius,
       me.sizeX AS sizeX,
       me.sizeY AS sizeY,
       me.sizeZ AS sizeZ,
       me.shapeType AS shapeType,
       wall,
       space
ORDER BY me.name
LIMIT $limit
