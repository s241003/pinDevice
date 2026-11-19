from vincenty import vincenty_inverse

# ===== Corner guidance parameters =====

CORNER_ENTER_DIST = 15.0
CORNER_EXIT_DIST = 18.0

CORNER_NOTICE_DIST = 15.0      # [m]
CORNER_HEADING_THRESH = 30.0   # ユーザがむいてる向きと経路の向きの差の最大許容値[deg]


_corner_active = False

def reset_corner_state():
    global _corner_active
    _corner_active = False

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

def get_next_corner(route,cur_idx,pos):
    """位置から見て、まだ過ぎていない最初のコーナーを返す。

    通過したコーナーを記録する方式 (_passed_idx) はやめた。
    コーナーまでの残距離 remain が正である間は「まだ先にある」と判断し、
    残距離が 0 以下になったら「過ぎた」と判断する。毎回位置から決め直すので、
    コーナーを過ぎてから引き返して再び近づけば、警告は復活する。
    """
    if not route["corners"]:
        return None
    for corner in sorted(route["corners"], key=lambda c: c["cumul_dist_m"]):
        if route_remaining_to_corner(pos, cur_idx, route, corner) > 0:
            return corner
    return None

def get_guidance_mode(pos, cur_edge_idx, route, enter_dist=None):
    global _corner_active
    corner = get_next_corner(route, cur_edge_idx, pos)
    if corner is None:
        _corner_active = False
        return "solid"
    remain = route_remaining_to_corner(pos, cur_edge_idx, route, corner)

    enter = CORNER_ENTER_DIST if enter_dist is None else enter_dist
    exit_d = enter + (CORNER_EXIT_DIST - CORNER_ENTER_DIST)
    if not _corner_active and remain <= enter:
        _corner_active = True
    elif _corner_active and remain >= exit_d:
        _corner_active = False

    return blink_level(remain) if _corner_active else "solid"
