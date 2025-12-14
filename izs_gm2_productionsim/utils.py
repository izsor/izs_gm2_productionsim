import json
from typing import Any, Dict

def loads_json(s: str) -> Dict[str, Any]:
    return json.loads(s)

def dumps_json(obj: Dict[str, Any]) -> str:
    return json.dumps(obj, ensure_ascii=False)
