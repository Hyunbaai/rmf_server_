import yaml
import argparse
from yaml.representer import SafeRepresenter
from supabase import create_client, Client

# Supabase 프로젝트 URL과 키
SUPABASE_URL = "https://wbjwefiaglmtwdtulhbo.supabase.co"
SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6IndiandlZmlhZ2xtdHdkdHVsaGJvIiwicm9sZSI6ImFub24iLCJpYXQiOjE3MjE0NzAzMjksImV4cCI6MjAzNzA0NjMyOX0.vKQBE2k0qU2SsRMH8sVN0RKdByWGLQ6d_MxgB8J5EOE"
# Supabase 클라이언트 생성
supabase = create_client(SUPABASE_URL, SUPABASE_KEY)

# 커스텀 YAML 표현 함수 정의
class FlowStyleList(list):
    pass

def flow_style_list_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)

yaml.add_representer(FlowStyleList, flow_style_list_representer, Dumper=yaml.SafeDumper)

# 중괄호 형식을 지원하는 사용자 정의 클래스
class FlowStyleDict(dict):
    pass

# 중괄호 형식을 지원하는 representer 정의
def flow_style_dict_representer(dumper, data):
    return dumper.represent_mapping("tag:yaml.org,2002:map", data, flow_style=True)

# YAML에 커스텀 representer 등록
yaml.add_representer(FlowStyleDict, flow_style_dict_representer, Dumper=yaml.SafeDumper)

# SafeDumper에 커스텀 들여쓰기 적용
class IndentedDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(IndentedDumper, self).increase_indent(flow, False)

# 여러 pathId 데이터를 가져오는 함수
def fetch_waypoints_for_multiple_paths(path_ids):
    all_waypoints = []
    for path_id in path_ids:
        response = (
            supabase.table("Waypoint")
            .select("*")
            .filter("pathId", "eq", path_id)
            .order("id", desc=False)
            .execute()
        )
        waypoints = response.data
        if waypoints:
            all_waypoints.extend(waypoints)
    return all_waypoints

# 명령줄에서 pathId 입력받기
def get_path_ids_from_args():
    parser = argparse.ArgumentParser(description="Generate building.yaml for specified pathIds.")
    parser.add_argument(
        "--path_ids",
        type=int,
        nargs="+",
        required=True,
        help="List of pathIds to process (e.g., --path_ids 126 127).",
    )
    args = parser.parse_args()
    return args.path_ids

def generate_yaml():

    # 명령줄에서 pathId 입력받기
    path_ids = get_path_ids_from_args()

    # 여러 pathId에 해당하는 Waypoints 가져오기
    waypoints = fetch_waypoints_for_multiple_paths(path_ids)

    # building.yaml 기본 구조
    building_template = {
        "coordinate_system": "wgs84",
        "crowd_sim": {
            "agent_groups": [FlowStyleDict(
                {  # 중괄호 형식으로 표현
                    "agents_name": [],
                    "agents_number": 0,
                    "group_id": 0,
                    "profile_selector": "external_agent",
                    "state_selector": "external_static",
                    "x": 0,
                    "y": 0,
                }
            )],
            "agent_profiles": [FlowStyleDict(
                {
                    "ORCA_tau": 1,
                    "ORCA_tauObst": 0.4,
                    "class": 1,
                    "max_accel": 0,
                    "max_angle_vel": 0,
                    "max_neighbors": 10,
                    "max_speed": 0,
                    "name": "external_agent",
                    "neighbor_dist": 5,
                    "obstacle_set": 1,
                    "pref_speed": 0,
                    "r": 0.25,
                }
            )],
            "enable": 0,
            "goal_sets": [],
            "model_types": [],
            "obstacle_set": FlowStyleDict({"class": 1, "file_name": "L1_navmesh.nav", "type": "nav_mesh"}),
            "states": [FlowStyleDict(
                {
                    "final": 1,
                    "goal_set": -1,
                    "name": "external_static",
                    "navmesh_file_name": "",
                }
            )],
            "transitions": [],
            "update_time_step": 0.10000000000000001,
        },
        "graphs": {},
        "levels": {
            "L1": {
                "elevation": 0,
                "lanes": [],
                "layers": {
                    "campus_reference": {
                        "color": FlowStyleList([0.114, 0.157, 1, 0.5]),
                        "filename": "campus_reference.png",
                        "transform": {
                            "scale": 0.10000000000000001,           # 지도 이미지의 비율을 실제 크기와 맞추기 위한 비율
                            "translation_lat": 36.1032682,          # GPS 안테나 설치 위치 (위도)
                            "translation_lon": 129.3913574,         # GPS 안테나 설치 위치 (경도)
                            "yaw": -1.5707963267948966,             # 지도 이미지의 방향을 맞추기 위한 회전 각도
                        },
                        "visible": True,
                    }
                },
                "vertices": [],  # Waypoint 데이터를 삽입할 부분
                "x_meters": 10,
                "y_meters": 10,
            }
        },
        "lifts": {},
        "name": "campus",
        "parameters": {
            "generate_crs": FlowStyleList([1, "EPSG:5179"]),
            "suggested_offset_x": FlowStyleList([3, 1169784]),
            "suggested_offset_y": FlowStyleList([3, 1791267]),
        },
        "reference_level_name": "L1",
    }

    # Supabase 데이터를 기반으로 vertices 생성
    def add_waypoints_to_vertices(waypoints):
        vertices = [
            FlowStyleList([waypoint["longitude"], waypoint["latitude"], 0, ""])
            for waypoint in waypoints
        ]

        return vertices

    building_template["levels"]["L1"]["vertices"] = add_waypoints_to_vertices(waypoints)

    # building.yaml 파일 저장
    output_path = "/home/zeta/rmf_docker/src/rmf_server/rmf_demos_maps/maps/handong/handong_evasion.building.yaml"
    with open(output_path, "w") as file:
        yaml.dump(building_template, file, default_flow_style=False, sort_keys=False, Dumper=IndentedDumper)

    print("building.yaml 파일 생성 완료!")

def main():
    generate_yaml()
