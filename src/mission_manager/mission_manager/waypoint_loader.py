"""从 map.yaml 加载区域并计算导航航点。"""
import yaml
import os
from typing import Dict, List, Tuple


def load_waypoints(map_yaml_path: str) -> Dict[str, Tuple[float, float]]:
    """解析 map.yaml，返回 {zone_name: (center_x, center_y)} 字典。

    只提取 type == 'task_area' 或 'start_area' 的区域作为可导航目标。
    """
    with open(map_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    waypoints: Dict[str, Tuple[float, float]] = {}

    for zone in data.get('zones', []):
        zone_type = zone.get('type', '')
        if zone_type not in ('task_area', 'start_area'):
            continue

        name = zone.get('name', '')
        polygon = zone.get('polygon', [])
        if not name or not polygon:
            continue

        # 计算多边形几何中心
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        waypoints[name] = (cx, cy)

    return waypoints


def get_zone_names(map_yaml_path: str) -> List[str]:
    """返回所有可导航区域名称列表。"""
    waypoints = load_waypoints(map_yaml_path)
    return sorted(waypoints.keys())
