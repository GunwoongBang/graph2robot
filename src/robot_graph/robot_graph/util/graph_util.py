from pathlib import Path
from typing import Any
from neo4j import Driver


def _load_queries(path: Path) -> dict[str, str]:
    queries: dict[str, str] = {}
    current_name: str | None = None
    current_lines: list[str] = []
    for line in path.read_text().splitlines():
        if line.startswith('-- name:'):
            if current_name is not None:
                queries[current_name] = '\n'.join(current_lines).strip()
            current_name = line[len('-- name:'):].strip()
            current_lines = []
        else:
            current_lines.append(line)
    if current_name is not None:
        queries[current_name] = '\n'.join(current_lines).strip()
    return queries


_QUERIES = _load_queries(Path(__file__).parent / 'query_handler.cypher')

_QUERY_SPACES = _QUERIES['QUERY_SPACES']
_QUERY_WALLS = _QUERIES['QUERY_WALLS']
_QUERY_LAYERS = _QUERIES['QUERY_LAYERS']
_QUERY_MEP_ELEMENTS = _QUERIES['QUERY_MEP_ELEMENTS']


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
                'center': record['center'],
                'bbox_max': record['bbox_max'],
                'bbox_min': record['bbox_min'],
                'directionSense': record['directionSense'],
                'space': record['space'],
                'layers': record['layers'],
            })
    return walls


def query_layers(driver: Driver, limit: int) -> list[dict[str, Any]]:
    layers: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_LAYERS, limit=limit)
        for record in result:
            layers.append({
                'id': record['id'],
                'name': record['name'],
                'layerIndex': record['layerIndex'],
                'thickness': record['thickness'],
                'wall_id': record['wall_id'],
            })
    return layers


def query_mep_elements(driver: Driver, limit: int) -> list[dict[str, Any]]:
    elements: list[dict[str, Any]] = []
    with driver.session() as session:
        result = session.run(_QUERY_MEP_ELEMENTS, limit=limit)
        for record in result:
            elements.append({
                'id': record['id'],
                'name': record['name'],
                'center': record['center'],
                'bbox_max': record['bbox_max'],
                'bbox_min': record['bbox_min'],
                'length': record['length'],
                'radius': record['radius'],
                'sizeX': record['sizeX'],
                'sizeY': record['sizeY'],
                'sizeZ': record['sizeZ'],
                'shapeType': record['shapeType'],
                'wall': record['wall'],
                'space': record['space'],
            })
    return elements
