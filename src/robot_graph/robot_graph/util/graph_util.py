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

_QUERY_BUILDINGS = _QUERIES['QUERY_BUILDINGS']
_QUERY_STOREYS = _QUERIES['QUERY_STOREYS']
_QUERY_SPACES = _QUERIES['QUERY_SPACES']
_QUERY_WALLS = _QUERIES['QUERY_WALLS']
# _QUERY_OPENINGS = _QUERIES['QUERY_OPENINGS']
_QUERY_LAYERS = _QUERIES['QUERY_LAYERS']
# _QUERY_MEP_SYSTEMS = _QUERIES['QUERY_MEP_SYSTEMS']
_QUERY_MEP_ELEMENTS = _QUERIES['QUERY_MEP_ELEMENTS']


def _run_query(driver: Driver, query: str, limit: int) -> list[dict[str, Any]]:
    with driver.session() as session:
        result = session.run(query, limit=limit)
        return [
            {
                'id': record['id'],
                'type': record['type'],
                'attributes': dict(record['attributes'] or {}),
                'relationship': [dict(r) for r in (record['relationship'] or [])],
            }
            for record in result
        ]


def query_buildings(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_BUILDINGS, limit)


def query_storeys(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_STOREYS, limit)


def query_spaces(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_SPACES, limit)


def query_walls(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_WALLS, limit)


def query_layers(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_LAYERS, limit)


def query_mep_elements(driver: Driver, limit: int) -> list[dict[str, Any]]:
    return _run_query(driver, _QUERY_MEP_ELEMENTS, limit)
