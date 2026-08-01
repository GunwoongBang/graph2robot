-- name: QUERY_BUILDINGS
MATCH (b:Building)
OPTIONAL MATCH (b)-[:HAS_STOREY]->(st:Storey)
WITH b,
     collect(CASE WHEN st IS NULL THEN NULL ELSE {
         type: 'has_storey', id: st.id
     } END) AS storey_raw
RETURN b.id AS id,
       b.ifcClass AS type,
       {
           name: b.name
       } AS attributes,
       [x IN storey_raw WHERE x IS NOT NULL] AS relationship
ORDER BY b.name
LIMIT $limit

-- name: QUERY_STOREYS
MATCH (st:Storey)
OPTIONAL MATCH (st)-[:HAS_SPACE]->(s:Space)
WITH st,
     collect(CASE WHEN s IS NULL THEN NULL ELSE {
         type: 'has_space', id: s.id
     } END) AS space_raw
RETURN st.id AS id,
       st.ifcClass AS type,
       {
           name: st.name
       } AS attributes,
       [x IN space_raw WHERE x IS NOT NULL] AS relationship
ORDER BY st.name
LIMIT $limit

-- name: QUERY_SPACES
MATCH (s:Space)
OPTIONAL MATCH (s)-[b:BOUNDED_BY]->(w:Wall)
WITH s,
     collect(CASE WHEN w IS NULL THEN NULL ELSE {
         type: 'bounded_by', id: w.id, side: b.side
     } END) AS wall_raw
OPTIONAL MATCH (s)-[:INTERSECTS]->(me:MEPElement)
WITH s, wall_raw,
     collect(CASE WHEN me IS NULL THEN NULL ELSE {
         type: 'intersects', id: me.id
     } END) AS mep_raw
RETURN s.id AS id,
       s.ifcClass AS type,
       {
           name:     s.name,
           longName: s.longName,
           centroid: s.centroid,
           bbox_min: s.bbox_min,
           bbox_max: s.bbox_max
       } AS attributes,
       [x IN wall_raw WHERE x IS NOT NULL] +
       [x IN mep_raw  WHERE x IS NOT NULL] AS relationship
ORDER BY s.name
LIMIT $limit

-- name: QUERY_WALLS
MATCH (w:Wall)
OPTIONAL MATCH (w)-[:HAS_LAYER]->(l:Layer)
WITH w,
     collect(CASE WHEN l IS NULL THEN NULL ELSE {
         type: 'has_layer', id: l.id
     } END) AS layer_raw
OPTIONAL MATCH (w)-[p:PENETRATED_BY]->(me:MEPElement)
WITH w, layer_raw,
     collect(CASE WHEN me IS NULL THEN NULL ELSE {
         type:    'penetrated_by',
         id:       me.id,
         center:   p.penetrationCenter,
         depth_mm: p.penetrationLength,
         radius:   p.penetrationRadius,
         sizeX:    p.penetrationSizeX,
         sizeY:    p.penetrationSizeY,
         sizeZ:    p.penetrationSizeZ
     } END) AS penet_raw
RETURN w.id AS id,
       w.ifcClass AS type,
       {
           axis2:          w.axis2,
           center:         w.center,
           bbox_min:       w.bbox_min,
           bbox_max:       w.bbox_max,
           directionSense: w.directionSense
       } AS attributes,
       [x IN layer_raw WHERE x IS NOT NULL] +
       [x IN penet_raw WHERE x IS NOT NULL] AS relationship
ORDER BY w.name
LIMIT $limit

-- name: QUERY_LAYERS
MATCH (l:Layer)
RETURN l.id AS id,
       l.ifcClass AS type,
       {
           name:       l.name,
           layerIndex: l.layerIndex,
           thickness:  l.thickness
       } AS attributes,
       [] AS relationship
ORDER BY l.name
LIMIT $limit

-- name: QUERY_MEP_SYSTEMS
MATCH (ms:MEPSystem)
RETURN ms.id AS id,
       ms.ifcClass AS type,
       {
           name: ms.name
       } AS attributes,
       [] AS relationship
ORDER BY ms.name
LIMIT $limit

-- name: QUERY_MEP_ELEMENTS
MATCH (me:MEPElement)
RETURN me.id AS id,
       me.ifcClass AS type,
       {
           name:      me.name,
           center:    me.center,
           shapeType: me.shapeType,
           direction: me.direction,
           radius:    me.radius,
           length:    me.length,
           sizeX:     me.sizeX,
           sizeY:     me.sizeY,
           sizeZ:     me.sizeZ,
           bbox_min:  me.bbox_min,
           bbox_max:  me.bbox_max
       } AS attributes,
       [] AS relationship
ORDER BY me.name
LIMIT $limit
