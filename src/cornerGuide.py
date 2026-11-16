from vincenty import vincenty_inverse

# ===== Corner guidance parameters =====

CORNER_ENTER_DIST = 15.0
CORNER_EXIT_DIST = 18.0

CORNER_NOTICE_DIST = 15.0      # [m]
CORNER_HEADING_THRESH = 30.0   # ユーザがむいてる向きと経路の向きの差の最大許容値[deg]


_corner_active = False
_passed_idx = -1

def reset_corner_state():
    global _corner_active, _passed_idx
    _corner_active = False
    _passed_idx = -1

def blink_level(remain_m):
    if remain_m <= 5.0:
        return "blink_fast"
    elif remain_m <= 10.0:
        return "blink"
    else:
        return "blink_slow"

def route_remaining_to_corner(pos,cur_edge_idx,route,corner):
    """現在位置から次コーナーまでの経路上残距離[m]"""
    coords = route["coords"]
    cur_edge_end = coords[cur_edge_idx + 1]
    remain = vincenty_inverse(pos,cur_edge_end)["distance"]
    remain += (corner["cumul_dist_m"] - route["distances"][cur_edge_idx + 1])

    return remain

def get_next_corner(route,cur_idx):
    """現在エッジより先に存在する最初のコーナー"""
    if not route["corners"]:
        return None
    cur_dist =  route["distances"][cur_idx]
    future = [c for c in route["corners"] if c["cumul_dist_m"] > cur_dist]

    if not future:
        return None

    return min(future,key=lambda c: c["cumul_dist_m"])

def get_guidance_mode(pos, cur_edge_idx, route):
    global _corner_active, _passed_idx
    corner = get_next_corner(route, cur_edge_idx)
    if corner is None or corner["corner_idx"] <= _passed_idx:
        _corner_active = False
        return "solid"
    remain = route_remaining_to_corner(pos, cur_edge_idx, route, corner)
    if remain <= 2:
        _passed_idx = corner["corner_idx"]
        _corner_active = False
        return "solid"

    if not _corner_active and remain <= CORNER_ENTER_DIST:
        _corner_active = True
    elif _corner_active and remain >= CORNER_EXIT_DIST:
        _corner_active = False

    return blink_level(remain) if _corner_active else "solid"